# BlueBoat Vector Field Controller

A custom BlueOS Docker extension designed to control a Blue Robotics BlueBoat (or any ArduRover skid-steering vehicle) using a Lyapunov-stable vector field. 

This plugin bypasses standard ArduRover L1 navigation, directly pushing the autopilot into `MANUAL` mode and overriding the RC channels (Ch1 Steering, Ch3 Throttle) to guide the uncrewed surface vehicle (USV) into a stable, counter-clockwise circular orbit around a designated target buoy.

Developed at ENSTA Brest.

## Features
* **Vector Field:** Convergence to a circular limit cycle from any starting position.
* **FastAPI Backend:** Asynchronous `pymavlink` background thread interacting with a lightweight web server.
* **Live Telemetry Dashboard:** TailwindCSS-powered frontend to adjust target coordinates, orbit radius, and monitor real-time distance and angular error at 5Hz.
* **BlueOS Native:** Packaged with required BlueOS Docker labels for automatic sidebar integration and host network permissions.

## Mathematical Formulation
The controller utilizes a normalized target vector field $\mathbf{f}(\mathbf{x})$ relative to the boat. Let the relative position vector to the buoy be $\tilde{\mathbf{x}}$ with distance $r = \|\tilde{\mathbf{x}}\|$ and target orbit radius $R$. 

$$u_x= - x^3 - x * y^2 + x - y$$
$$u_y= - y^3 - x^2 * y + x + y$$

---

## 1. Local Simulation Testing (SITL)

You do not need the physical BlueBoat to test the logic. We use the ArduPilot Software-In-The-Loop (SITL) container configured for **skid-steering** (`rover-skid`).

### Start the Simulator
Run the following Docker command to spin up the simulated vehicle. It will broadcast MAVLink over TCP on port `5760`.
```bash
docker run -it --rm -p 5760:5760 --env VEHICLE=APMrover2 --env MODEL=rover-skid radarku/ardupilot-sitl
```

### Run the Controller
In a separate terminal, install the dependencies and run the Python backend. The script defaults to the SITL TCP port if no environment variables are set.
```bash
pip install -r src/requirements.txt
python src/main.py
```

### Access the Dashboard
Open your web browser and navigate to `http://localhost:8000`. You can update the target buoy coordinates, set the radius, and engage the controller.

---

## 2. Deployment on BlueOS (Physical BlueBoat)

This repository is configured with a GitHub Actions workflow that automatically cross-compiles the Docker image for `linux/arm/v7` and `linux/arm64` (Raspberry Pi compatible) upon pushing to the `main` branch.

### Installation via Pirate Mode
Because this is a custom lab extension and not yet published on the official Blue Robotics store, you must install it manually using BlueOS Pirate Mode.

1. Power on the BlueBoat and connect your computer to its WiFi network.
2. Navigate to the BlueOS web interface (usually `http://blueos.local` or `192.168.2.1`).
3. In the left sidebar, go to **Extensions** -> **Installed**.
4. Click the **Pirate Mode** toggle (the skull and crossbones icon) in the top right corner to reveal advanced developer options.
5. Under the *Install extension from registry* section, input your Docker Hub image tag. For example:
   ```text
   your_dockerhub_username/blueboat-vector-field:latest
   ```
6. Click **Install**. BlueOS will pull the ARM image from Docker Hub.

### Operation
Once installed, BlueOS will read the metadata labels and automatically create a new **Vector Field Controller** tab in the left sidebar. 

1. Click the new tab to open the built-in web dashboard.
2. Enter the coordinates of your target buoy (e.g., in the Rade de Brest).
3. Ensure the physical BlueBoat area is clear.
4. Click **START VECTOR CONTROLLER**. The script will automatically arm the vehicle and begin differential thrust execution.