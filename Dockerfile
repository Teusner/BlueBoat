# Lightweight Python base image
FROM python:3.11-slim

# ==========================================
# BLUEOS EXTENSION METADATA
# ==========================================
# These labels tell BlueOS how to display your plugin in the sidebar
LABEL company="ENSTA Brest"
LABEL project="Vector Field Controller"
LABEL type="navigation"
LABEL authors="Quentin Brateau"
LABEL version="1.0.0"

# BlueOS Permissions API: 
# We request Host Network access so we can read the live MAVLink stream directly
LABEL permissions='\
{\
    "NetworkMode": "host",\
    "HostConfig": {\
        "Privileged": true\
    }\
}'

# ==========================================
# CONTAINER BUILD INSTRUCTIONS
# ==========================================
WORKDIR /app

# Install Python dependencies
COPY src/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy our source code into the container
COPY src/ ./src/

# Expose the FastAPI web server port
EXPOSE 8000

# Set the environment variable for the real BlueBoat UDP endpoint
ENV MAVLINK_CONN="udpin:0.0.0.0:14551"

# Launch the FastAPI server
CMD ["python", "-u", "src/main.py"]