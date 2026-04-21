# Autoware Universe + CARLA Leaderboard Bridge Setup
## Complete Instruction Set (From Scratch)

This document describes how to set up Autoware Universe 1.7.1 with the
`autoware_carla_leaderboard` bridge for scenario-based testing with CARLA 0.9.16.

**Assumptions:**
- CARLA is running with native DDS enabled (see CARLA Native DDS Setup guide)
- Docker is installed on the host
- `rocker` is installed on the host
- On vedant@Minerva: base path is `/data/vedant/Ubuntu/Swaraj/`

---

## Prerequisites

### Install rocker (one-time)

```bash
pip install rocker
rocker --version
```

If you see `Docker permission error`, add your user to the docker group:

```bash
sudo usermod -aG docker $USER
newgrp docker   # applies immediately without logout
```

---

## Part 1: autoware_carla_leaderboard Bridge Setup (One-Time)

### Step 1: Clone the repository

```bash
cd /data/vedant/Ubuntu/Swaraj
git clone https://github.com/TUMFTM/autoware_carla_leaderboard.git
```

### Step 2: Pull the bridge Docker image

```bash
docker pull tumgeka/autoware_carla_leaderboard:0.9.16
```

### Step 3: Launch the bridge container

```bash
rocker --nvidia --x11 --privileged --net=host \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard \
    --name aw_carla_leaderboard_0916 \
    -- tumgeka/autoware_carla_leaderboard:0.9.16
```

### Step 4: Import repos and build (inside the container)

If the `src/external` directory was copied via `scp` rather than cloned, fix git
ownership first:

```bash
git config --global --add safe.directory '*'
```

Then import and build:

```bash
cd /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard
mkdir -p src/external \
    && vcs import --recursive src/external < docker/carla.repos \
    && vcs import --recursive src/external < docker/ros2.repos

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The build is complete when you see `34 packages finished`. Two stderr warnings
are expected and harmless: `tier4_auto_msgs_converter` and `autoware_carla_cpp_bridge`.

### Step 5: Install the CARLA wheel (one-time per container build)

**Critical:** The bridge container ships with a generic CARLA Python package
incompatible with your CARLA binary. This causes `client.get_world()` to crash
with `std::bad_alloc`. Replace it with the wheel from your CARLA installation.

Copy the wheel from `carla_non_source`:

```bash
cp /data/vedant/Ubuntu/Swaraj/carla_non_source/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl \
    /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard/
```

Then inside the bridge container:

```bash
pip3 install \
    /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard/carla-0.9.16-cp310-cp310-linux_x86_64.whl \
    --force-reinstall
```

**This must be repeated every time the container restarts.**

Verify connectivity:

```bash
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
print('Server:', client.get_server_version())
world = client.get_world()
print('Map:', world.get_map().name)
"
```

### Step 6: Patch the leaderboard evaluator to skip redundant map reloads

The leaderboard evaluator always calls `client.load_world()` even if the correct
map is already loaded. This causes a 5000ms timeout crash. Apply this patch once:

```bash
sed -i 's/        self.world = self.client.load_world(town, reset_settings=False)/        current_map = self.client.get_world().get_map().name.split("\/")[-1]\n        if current_map == town:\n            print(f"Map {town} already loaded, skipping reload.")\n            self.world = self.client.get_world()\n        else:\n            self.world = self.client.load_world(town, reset_settings=False)/' \
    /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard/src/external/leaderboard/leaderboard/leaderboard_evaluator.py
```

Verify the patch was applied correctly:

```bash
sed -n '240,255p' \
    /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard/src/external/leaderboard/leaderboard/leaderboard_evaluator.py
```

You should see the `current_map == town` check before the `load_world` call.

**Why this patch is necessary:** Without it, the evaluator reloads the map every
run even when CARLA already has the correct map loaded. This triggers shader
recompilation and causes a 5-second timeout crash on slower hardware.

### Step 7: Configure the scenario

Edit `config/config.yaml`:

```yaml
evaluation:
  agent_setup:
    agent: src/autoware_agent/aw_e2e.py
  general_parameters:
    host: localhost
    port: 2000
    traffic-manager-port: 8001   # Use 8001 to avoid bind errors from previous runs
    debug: 0
    timeout: 300.0
  simulation_setup:
    routes: resources/routes/short_route_2_Town10.xml
```

**Available routes and their required maps:**

| Route file | Town | Notes |
|---|---|---|
| `short_route_2_Town10.xml` | Town10HD_Opt | Default CARLA map, recommended |
| `town10_parking_crossing.xml` | Town10HD_Opt | Default CARLA map |
| `town01_parking_pedestrian_crossing_short.xml` | Town01 | Requires map switch |

**Note on traffic-manager-port:** Always use `8001` instead of the default `8000`
to avoid `RuntimeError: bind error` from Traffic Manager ports left open by
previous runs that didn't clean up properly.

---

## Part 2: Autoware Universe 1.7.1 Setup (One-Time)

### Step 1: Pull the Autoware Docker image

```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-devel-cuda-1.7.1
```

### Step 2: Transfer the Autoware workspace

If setting up on a new machine, transfer from an existing setup:

```bash
scp -r ~/autoware vedant@10.20.110.48:/data/vedant/Ubuntu/Swaraj/
scp -r ~/autoware_maps vedant@10.20.110.48:/data/vedant/Ubuntu/Swaraj/
scp -r ~/autoware_data vedant@10.20.110.48:/data/vedant/Ubuntu/Swaraj/
```

**Important:** The maps directory is `autoware_maps` (with an 's'). Always verify:

```bash
ls /data/vedant/Ubuntu/Swaraj/autoware_maps/carla/Town10/
# Must show: lanelet2_map.osm  map_projector_info.yaml  pointcloud_map.pcd
```

### Step 3: Clone the Carla Audi e-tron vehicle model (if not transferred)

```bash
git clone https://github.com/TUMFTM/carla_audi_etron.git \
    /data/vedant/Ubuntu/Swaraj/autoware/src/carla_audi_etron
```

### Step 4: Launch the Autoware container and build

```bash
rocker --nvidia --x11 --privileged --net=host \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume /data/vedant/Ubuntu/Swaraj/autoware \
    --volume /data/vedant/Ubuntu/Swaraj/autoware_maps \
    --volume /data/vedant/Ubuntu/Swaraj/autoware_data \
    --name autoware_1_7_1 \
    -- ghcr.io/autowarefoundation/autoware:universe-devel-cuda-1.7.1
```

Inside the container:

```bash
apt-get update
apt-get install -y ros-humble-rviz-2d-overlay-plugins

cd /data/vedant/Ubuntu/Swaraj/autoware
rm -rf build/autoware_string_stamped_rviz_plugin build/autoware_overlay_rviz_plugin

MAKEFLAGS="-j4" colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --parallel-workers 4 \
    --packages-skip \
    acados \
    autoware_tensorrt_bevformer \
    autoware_tensorrt_plugins \
    autoware_lidar_frnet \
    autoware_ptv3 \
    negotiated_examples \
    eagleye_fix2kml \
    eagleye_geo_pose_converter \
    trt_batched_nms
```

### Step 5: Create the autoware_data symlink (every container restart)

```bash
ln -s /data/vedant/Ubuntu/Swaraj/autoware_data /root/autoware_data 2>/dev/null || true
```

---

## Part 3: Running the Full Stack

### Terminal Layout

- **Terminal 1**: CARLA (host)
- **Terminal 2**: Bridge container — C++ bridge
- **Terminal 3**: Bridge container — leaderboard evaluator
- **Terminal 4**: Autoware container — Autoware launch
- **Terminal 5**: Host — monitoring

### Terminal 1: Launch CARLA

```bash
cd /data/vedant/Ubuntu/Swaraj/carla_non_source
./CarlaUE4.sh --ros2
```

CARLA loads Town10HD_Opt by default. Do not switch maps unless your route requires it.

### Terminal 2: Start the bridge

```bash
rocker --nvidia --x11 --privileged --net=host \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard \
    --name aw_carla_leaderboard_0916 \
    -- tumgeka/autoware_carla_leaderboard:0.9.16
```

Inside the container — run these every restart:

```bash
cd /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard
source install/setup.bash
source carla_envs.sh
pip3 install carla-0.9.16-cp310-cp310-linux_x86_64.whl --force-reinstall
ros2 launch autoware_carla_cpp_bridge aw_carla_cpp_bridge.launch.py
```

### Terminal 3: Open a second shell in the bridge container

```bash
docker exec -it aw_carla_leaderboard_0916 bash
cd /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard
source install/setup.bash
source carla_envs.sh
pip3 install carla-0.9.16-cp310-cp310-linux_x86_64.whl --force-reinstall
```

### Terminal 4: Launch Autoware

```bash
rocker --nvidia --x11 --privileged --net=host \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume /data/vedant/Ubuntu/Swaraj/autoware \
    --volume /data/vedant/Ubuntu/Swaraj/autoware_maps \
    --volume /data/vedant/Ubuntu/Swaraj/autoware_data \
    --name autoware_1_7_1 \
    -- ghcr.io/autowarefoundation/autoware:universe-devel-cuda-1.7.1
```

Inside the container:

```bash
ln -s /data/vedant/Ubuntu/Swaraj/autoware_data /root/autoware_data 2>/dev/null || true
source /data/vedant/Ubuntu/Swaraj/autoware/install/setup.bash

# For Town10 routes:
ros2 launch autoware_launch e2e_simulator.launch.xml \
    vehicle_model:=carla_audi_etron_vehicle \
    sensor_model:=carla_audi_etron_sensor_kit \
    map_path:=/data/vedant/Ubuntu/Swaraj/autoware_maps/carla/Town10

# For Town01 routes:
ros2 launch autoware_launch e2e_simulator.launch.xml \
    vehicle_model:=carla_audi_etron_vehicle \
    sensor_model:=carla_audi_etron_sensor_kit \
    map_path:=/data/vedant/Ubuntu/Swaraj/autoware_maps/carla/Town01
```

### Terminal 3: Run the leaderboard evaluator

Only after Autoware is fully launched (50+ nodes via `ros2 node list | wc -l`):

```bash
python3 src/external/leaderboard/leaderboard/leaderboard_evaluator.py \
    --conf_file_path=config/config.yaml
```

Expected output on successful run:
```
========= Preparing RouteScenario_0 (repetition 0) =========
> Loading the world
Map Town10HD_Opt already loaded, skipping reload.
> Setting up the agent
> Running the route
=== [Agent] -- Wallclock = ... -- System time = ... -- Game time = ...
```

---

## Part 4: Between-Run Cleanup

**Always clean up between runs** to prevent stale LiDAR data from causing NDT
localization failures on the next run.

### Step 1: Destroy all CARLA actors (Terminal 3)

Run this after the evaluator finishes and before starting the next run:

```bash
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
actors = world.get_actors()
count = 0
for actor in actors:
    if actor.type_id != 'spectator':
        actor.destroy()
        count += 1
print(f'Destroyed {count} actors')
"
```

### Step 2: Verify all actors are gone before next spawn

```bash
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
actors = [a for a in world.get_actors() if a.type_id != 'spectator']
print(f'Remaining actors: {len(actors)}')
for a in actors:
    print(f'  {a.type_id} id={a.id}')
"
```

Only proceed to the next run when `Remaining actors: 0`.

### Step 3: Restart the bridge (Terminal 2)

Ctrl+C the bridge, then relaunch:

```bash
ros2 launch autoware_carla_cpp_bridge aw_carla_cpp_bridge.launch.py
```

### Step 4: Restart Autoware (Terminal 4)

Ctrl+C Autoware, then relaunch with the same command as before. This clears
all topic buffers and EKF state, ensuring a clean localization initialization.

---

## Part 5: LiDAR Debugging — NDT Score Below Threshold

If Terminal 4 shows recurring warnings like:
```
[autoware_ndt_scan_matcher_node]: Score is below the threshold. Score: 1.2, Threshold: 2.3
[autoware_ndt_scan_matcher_node]: No InputSource. Please check the input lidar topic
```

Follow this diagnostic chain to find the break in the LiDAR pipeline:

### Step 1: Verify the full LiDAR topic chain

Check each topic in order — the first one with no data is the break point:

```bash
# CARLA raw output (should have data ~1Hz when vehicle is spawned)
ros2 topic hz /carla/ego_vehicle/sensor/lidar/front/point_cloud

# Bridge output (should have data ~5-6Hz)
ros2 topic hz /sensor/lidar/front

# After crop box filters (should match bridge output)
ros2 topic hz /sensing/lidar/front/mirror_cropped/pointcloud_ex
ros2 topic hz /sensing/lidar/front/self_cropped/pointcloud_ex

# Final output to NDT (must have data for localization to work)
ros2 topic hz /sensing/lidar/front/pointcloud
ros2 topic hz /sensing/lidar/concatenated/pointcloud
```

### Step 2: Check for nullptr errors

If Terminal 4 shows:
```
transformed_raw_points[/sensing/lidar/front/mirror_cropped/pointcloud_ex] is nullptr
```

This means stale data from a previous run is interfering. **Restart the bridge
and Autoware** following the between-run cleanup steps above.

### Step 3: Check if the ego vehicle is actually spawned

The LiDAR topics will be empty until the leaderboard evaluator spawns the ego
vehicle. Verify the evaluator has printed `> Running the route` before checking
LiDAR Hz. Topics will only have data after that point.

### Step 4: Check NDT input topic

NDT subscribes to `/localization/util/downsample/pointcloud`. Verify it has data:

```bash
ros2 topic hz /localization/util/downsample/pointcloud
```

If this is empty but `/sensing/lidar/concatenated/pointcloud` has data, the
downsampler node has failed. Restart Autoware.

### Step 5: Root cause — stale data from previous run

The most common cause of NDT score failures is residual LiDAR point clouds
from a previous run's ego vehicle still being buffered when the new run starts.
The fix is always a full cleanup and restart between runs as described in Part 4.

---

## Architecture Overview

```
vedant@Minerva (all components on same machine)
├── Terminal 1: CARLA 0.9.16 (carla_non_source, native DDS via --ros2)
│   └── Publishes sensor data via FastDDS on localhost
│
├── Terminal 2/3: autoware_carla_leaderboard container (CycloneDDS)
│   ├── C++ bridge: FastDDS topics → CycloneDDS ROS2 topics
│   └── Leaderboard evaluator: spawns ego vehicle, runs scenarios
│
└── Terminal 4: Autoware 1.7.1 container (CycloneDDS)
    └── Subscribes to bridge topics, runs planning/control
```

**Why all components on the same machine?**
DDS multicast discovery is unreliable over WiFi. Running everything on
vedant@Minerva eliminates cross-machine DDS issues entirely. All topics
are discovered via localhost multicast which is always reliable.

---

## Container Restart Checklist

**Bridge container (Terminals 2 and 3) — every restart:**
```bash
cd /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard
source install/setup.bash
source carla_envs.sh
pip3 install carla-0.9.16-cp310-cp310-linux_x86_64.whl --force-reinstall
```

**Autoware container (Terminal 4) — every restart:**
```bash
ln -s /data/vedant/Ubuntu/Swaraj/autoware_data /root/autoware_data 2>/dev/null || true
source /data/vedant/Ubuntu/Swaraj/autoware/install/setup.bash
```

---

## Known Issues

### Town01 Map Switch
Switching from Town10HD_Opt to Town01 triggers shader recompilation in CARLA.
On hardware with limited VRAM (RTX 4080 Laptop, 12GB) this causes a GPU renderer
crash. On vedant@Minerva (RTX 5070 Ti, 16GB) this works correctly.

### Simulation Speed
Game time to system time ratio depends on hardware and synchronous mode tick rate.
With all components on vedant@Minerva, expect significantly better ratios than
the 0.036x seen when running across machines over WiFi.

### Traffic Vehicle Spawn Failures
```
WARNING: Cannot spawn actor vehicle.* at position Location(x=...) 
```
Traffic vehicles sometimes fail to spawn if their spawn point is occupied by the
ego vehicle. This is a known leaderboard issue and does not affect the evaluation
of the ego vehicle's behavior.

### pointcloud_map_loader Missing
If `/map/pointcloud_map` has `Publisher count: 0`, the map loader failed to start.
The most common cause is the wrong `map_path` — always use `autoware_maps` (with
an 's'), not `autoware_map`. Verify the path contains `pointcloud_map.pcd` before
launching Autoware.

---

## File Structure

```
/data/vedant/Ubuntu/Swaraj/
├── carla_non_source/              # Pre-built CARLA 0.9.16 binary
│   └── PythonAPI/carla/dist/      # CARLA Python wheel
├── autoware_carla_leaderboard/    # Bridge repo + built workspace
│   ├── config/config.yaml         # Scenario configuration
│   ├── carla-0.9.16-*.whl         # CARLA wheel (copied from carla_non_source)
│   └── src/external/leaderboard/  # Patched leaderboard evaluator
├── autoware/                      # Autoware 1.7.1 source + built workspace
│   └── src/carla_audi_etron/      # Vehicle + sensor kit
├── autoware_maps/                 # Map assets (note: with 's')
│   └── carla/
│       ├── Town01/
│       └── Town10/
└── autoware_data/                 # ML model artifacts
    ├── lidar_centerpoint/
    ├── tensorrt_yolox/
    ├── traffic_light_classifier/
    └── traffic_light_fine_detector/
```