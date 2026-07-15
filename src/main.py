import time
import numpy as np
import threading
from pymavlink import mavutil

from fastapi.staticfiles import StaticFiles


class VectorFieldController:
    def __init__(self, buoy_lat, buoy_lon, radius, active=False, connection_string='udpin:0.0.0.0:14551'):
        self.buoy_lat = buoy_lat
        self.buoy_lon = buoy_lon
        self.radius = radius
        self.active = active
        
        # Real-time state variables for the frontend dashboard
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        self.distance_to_buoy = 0.0
        self.angular_error = 0.0
        self.int_angular_error = 0.0  # Integral term for PID control
        
        # 1. Establish MAVLink Connection
        # We listen on a UDP port. In BlueOS, we will route telemetry to this container's port.
        print(f"Connecting to MAVLink on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        
        # Start the background telemetry and control loop
        self.loop_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.loop_thread.start()

    def _control_loop(self):
        """Background thread running at ~10Hz to process telemetry and send control efforts."""
        
        print("Background thread waiting for MAVLink heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat detected! BlueBoat connected.")
        
        while True:
            # Fetch GLOBAL_POSITION_INT (Provides both GPS and Heading)
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            if not msg:
                continue
                
            # Extract and scale state
            self.current_lat = msg.lat / 1e7
            self.current_lon = msg.lon / 1e7
            self.current_heading = msg.hdg * np.pi / 18000.0  # radians
            
            # Compute distance and vector field continuously for dashboard monitoring
            self._update_kinematics()
            
            # Only send thruster commands if the user has engaged the controller
            if self.active:
                self._send_control_commands()
                
            time.sleep(0.1)

    def _update_kinematics(self):
        """Calculate the local tangent plane distances and your analytical vector field."""
        # Standard flat-earth approximation. Perfectly sufficient for local maritime 
        R_EARTH = 6378137.0
        dx = np.radians(self.current_lon - self.buoy_lon) * R_EARTH * np.cos(np.radians(self.buoy_lat))
        dy = np.radians(self.current_lat - self.buoy_lat) * R_EARTH
        self.distance_to_buoy = np.sqrt(dx**2 + dy**2)
        
        # Prevent division by zero
        scaled_dx = dx / self.radius
        scaled_dy = dy / self.radius

        ux = - scaled_dx**3 - scaled_dx * scaled_dy**2 + scaled_dx - scaled_dy
        uy = - scaled_dy**3 - scaled_dx**2 * scaled_dy + scaled_dx + scaled_dy
        
        # Convert to standard compass heading (0 = North, 90 = East)
        desired_heading = np.arctan2(ux, uy)

        # Calculate angular error
        self.angular_error = (desired_heading - self.current_heading + np.pi) % (2 * np.pi) - np.pi
        self.int_angular_error += self.angular_error * 0.1  # Integral term for PID

    def _send_control_commands(self):
        """Map the vector field output to BlueBoat actuators."""
        # Simple P-controller for yaw rate based on heading error.
        # Since you're dealing with custom control laws, this can easily be swapped 
        # out for a more robust interval analysis or bounded-error approach later.
        kp = 200.0
        ki = 50.0
        yaw_effort = max(-500, min(500, int(kp * self.angular_error + ki * self.int_angular_error)))
        
        # Base forward throttle (1500 is neutral, 1900 is full ahead)
        base_throttle = 300 
        
        # ArduRover Skid Steering mapping via RC Overrides:
        # Channel 3: Throttle
        # Channel 4: Yaw / Steering
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1500 + yaw_effort,         # Ch1 (Yaw)
            0,                         # Ch2
            1500 + base_throttle,      # Ch3 (Throttle)
            0, 0, 0, 0, 0                 # Ch4, Ch5, Ch6, Ch7, Ch8
        )

    def start_controller(self):
        # 1. Set vehicle mode to MANUAL
        mode_id = self.master.mode_mapping().get('MANUAL')
        if mode_id is not None:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
        # 2. ARM the vehicle (1 = Arm, 0 = Disarm)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        self.active = True

    def stop_controller(self):
        """Disengage and halt the boat."""
        self.active = False

        # Send neutral commands to stop the thrusters instantly
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1500, 0, 1500, 0, 0, 0, 0, 0
        )

        # 2. ARM the vehicle (1 = Arm, 0 = Disarm)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
# ==========================================
# 2. THE FASTAPI WEB SERVER
# ==========================================
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn
from contextlib import asynccontextmanager
import os

controller = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global controller
    print("Initializing MAVLink background thread...")
    
    # Read from environment, default back to SITL TCP if not found
    conn_string = os.environ.get("MAVLINK_CONN", "tcp:127.0.0.1:5760")
    
    controller = VectorFieldController(
        buoy_lat=0.0, 
        buoy_lon=0.0, 
        radius=15.0, 
        connection_string=conn_string 
    )
    
    # The web server runs while yielded here
    yield 
    
    # This executes when you press Ctrl+C to shut down the server
    print("\nShutting down Web Server and halting Vector Field Controller...")
    if controller and controller.active:
        controller.stop_controller()

# Initialize FastAPI with the lifespan manager
app = FastAPI(title="BlueBoat Vector Field Controller", lifespan=lifespan)

class TargetUpdate(BaseModel):
    lat: float
    lon: float
    radius: float

@app.get("/api/state")
def get_state():
    if not controller:
        return {"error": "Controller not initialized"}
    return {
        "active": controller.active,
        "boat_lat": controller.current_lat,
        "boat_lon": controller.current_lon,
        "distance_to_buoy": round(controller.distance_to_buoy, 2),
        "angular_error": round(controller.angular_error * 180 / np.pi, 2),
        "target_lat": controller.buoy_lat,
        "target_lon": controller.buoy_lon,
        "radius": controller.radius
    }

@app.post("/api/target")
def update_target(target: TargetUpdate):
    controller.buoy_lat = target.lat
    controller.buoy_lon = target.lon
    controller.radius = target.radius
    return {"status": "Target updated"}

@app.post("/api/toggle")
def toggle_controller():
    if controller.active:
        controller.stop_controller()
    else:
        # Dynamic placement for testing: sets buoy slightly North of current position
        if controller.buoy_lat == 0.0:
            controller.buoy_lat = controller.current_lat + 0.0005 
            controller.buoy_lon = controller.current_lon
        controller.start_controller()
    return {"active": controller.active}

# MOUNT STATIC FILES LAST (Prevents 404 errors on API routes)
app.mount("/", StaticFiles(directory="src/static", html=True), name="static")

# ==========================================
# 3. THE EXECUTION BLOCK
# ==========================================
if __name__ == "__main__":
    print("Starting Web Server on port 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000)