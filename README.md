# Autoware Universe + CARLA Leaderboard Bridge Setup
## Complete Instruction Set (From Scratch)

This document describes how to set up Autoware Universe 1.7.1 with the
`autoware_carla_leaderboard` bridge for scenario-based testing with CARLA 0.9.16.

**Assumptions:**
- CARLA is running with native DDS enabled (see [CARLA Native DDS Setup](carla_native_dds_setup.md) guide)
- Docker is installed on the host
- `rocker` is installed on the host
- Host machine: `/home/skodas/`

---

## Prerequisites

### Install rocker (one-time)

```bash
pip install rocker
rocker --version
```

---

## Part 1: autoware_carla_leaderboard Bridge Setup (One-Time)

### Step 1: Clone the repository

```bash
cd ~
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
    --volume ~/autoware_carla_leaderboard \
    --name aw_carla_leaderboard_0916 \
    -- tumgeka/autoware_carla_leaderboard:0.9.16
```

### Step 4: Import repos and build (inside the container)

Once inside the container shell:

```bash
cd /home/skodas/autoware_carla_leaderboard
mkdir -p src/external \
    && vcs import --recursive src/external < docker/carla.repos \
    && vcs import --recursive src/external < docker/ros2.repos

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

This build takes approximately 1-2 minutes. Two warnings are expected and harmless:
- `tier4_auto_msgs_converter`: header install destination deprecation notice
- `autoware_carla_cpp_bridge`: unused parameter warning in C++

The build is complete when you see `34 packages finished`.

### Step 5: Install the source-built CARLA wheel (one-time per container build)

**Critical:** The bridge container ships with a generic CARLA 0.9.16 Python package
that is incompatible with your source-built CARLA server. This causes `client.get_world()`
to crash with `std::bad_alloc`. You must replace it with your source-built wheel.

First, copy the wheel to the mounted directory on the **host**:

```bash
cp /home/skodas/carla/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl \
    /home/skodas/autoware_carla_leaderboard/
```

Then inside the **bridge container**:

```bash
pip3 install \
    /home/skodas/autoware_carla_leaderboard/carla-0.9.16-cp310-cp310-linux_x86_64.whl \
    --force-reinstall
```

**Important:** This must be repeated every time the container restarts since the
container is ephemeral and resets to its original image on restart.

### Step 6: Patch the leaderboard evaluator to skip redundant map reloads

The leaderboard evaluator always calls `client.load_world()` even if the correct
map is already loaded. This causes a 5000ms timeout crash when CARLA already has
the target map loaded. Patch it on the host:

```bash
sed -i 's/        self.world = self.client.load_world(town, reset_settings=False)/        current_map = self.client.get_world().get_map().name.split("\/")[-1]\n        if current_map == town:\n            print(f"Map {town} already loaded, skipping reload.")\n            self.world = self.client.get_world()\n        else:\n            self.world = self.client.load_world(town, reset_settings=False)/' \
    /home/skodas/autoware_carla_leaderboard/src/external/leaderboard/leaderboard/leaderboard_evaluator.py
```

Verify the patch:

```bash
sed -n '240,255p' \
    /home/skodas/autoware_carla_leaderboard/src/external/leaderboard/leaderboard/leaderboard_evaluator.py
```

### Step 7: Configure the scenario

Edit `config/config.yaml` to set your agent and route. Use Town10HD_Opt routes
since that is what CARLA loads by default — switching maps causes GPU crashes:

```yaml
evaluation:
  agent_setup:
    agent: src/autoware_agent/aw_e2e.py   # E2E mode
    # agent: src/autoware_agent/aw_priviliged.py  # Privileged mode
  general_parameters:
    debug: 0    # Keep at 0 to minimize disk writes
  simulation_setup:
    routes: resources/routes/short_route_2_Town10.xml
```

**Available routes and their required maps:**

| Route file | Town | Notes |
|---|---|---|
| `short_route_2_Town10.xml` | Town10HD_Opt | Default CARLA map, no switching needed |
| `town10_parking_crossing.xml` | Town10HD_Opt | Default CARLA map, no switching needed |
| `town01_parking_pedestrian_crossing_short.xml` | Town01 | Requires map switch, may crash GPU |

**Always use Town10HD_Opt routes** to avoid the GPU renderer crash that occurs
when CARLA switches maps dynamically.

---

## Part 2: Autoware Universe 1.7.1 Setup (One-Time)

### Step 1: Pull the Autoware Docker image

```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-devel-cuda-1.7.1
```

### Step 2: Clone the Autoware 1.7.1 repository

```bash
cd ~
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
git checkout 1.7.1
```

### Step 3: Import all Autoware repos

```bash
cd ~/autoware
mkdir -p src
vcs import src < repositories/autoware.repos
```

This clones approximately 30 repositories pinned to specific versions. The
`detached HEAD` messages are expected and normal.

### Step 4: Fix acados submodules

The `acados` package requires submodules that are not cloned by default:

```bash
cd ~/autoware/src/universe/external/acados
git submodule update --init --recursive
cd ~/autoware
```

### Step 5: Clone the Carla Audi e-tron vehicle model

```bash
git clone https://github.com/TUMFTM/carla_audi_etron.git \
    ~/autoware/src/carla_audi_etron
```

This provides the `carla_audi_etron_vehicle` and `carla_audi_etron_sensor_kit`
that Autoware needs when launching with CARLA.

### Step 6: Download map assets (one-time)

Download the CARLA Lanelet2 maps for Town01, Town07, and Town10:

```bash
mkdir -p ~/autoware_map/carla
# Download from: https://github.com/TUMFTM/autoware_carla_leaderboard/releases/tag/v1.0.0-maps
# Extract into ~/autoware_map/carla/ so the structure is:
# ~/autoware_map/carla/Town01/lanelet2_map.osm
# ~/autoware_map/carla/Town01/pointcloud_map.pcd
# ~/autoware_map/carla/Town01/map_projector_info.yaml
# ~/autoware_map/carla/Town10/...
```

### Step 7: Download Autoware ML model artifacts (one-time)

Autoware requires ML model files for perception. Run this script once:

```bash
cat > ~/download_autoware_data.sh << 'EOF'
#!/bin/bash
DATA_DIR=~/autoware_data
BASE_URL=https://awf.ml.dev.web.auto/perception/models

mkdir -p $DATA_DIR/lidar_centerpoint
mkdir -p $DATA_DIR/tensorrt_yolox
mkdir -p $DATA_DIR/traffic_light_classifier
mkdir -p $DATA_DIR/traffic_light_fine_detector

# lidar_centerpoint
wget -P $DATA_DIR/lidar_centerpoint $BASE_URL/centerpoint/v3/centerpoint_tiny_ml_package.param.yaml
wget -P $DATA_DIR/lidar_centerpoint $BASE_URL/centerpoint/v3/pts_voxel_encoder_centerpoint_tiny.onnx
wget -P $DATA_DIR/lidar_centerpoint $BASE_URL/centerpoint/v3/pts_backbone_neck_head_centerpoint_tiny.onnx
wget -P $DATA_DIR/lidar_centerpoint $BASE_URL/centerpoint/v3/centerpoint_ml_package.param.yaml
wget -P $DATA_DIR/lidar_centerpoint $BASE_URL/centerpoint/v3/detection_class_remapper.param.yaml
wget -P $DATA_DIR/lidar_centerpoint $BASE_URL/centerpoint/v3/deploy_metadata.yaml

# tensorrt_yolox
wget -P $DATA_DIR/tensorrt_yolox $BASE_URL/tl_detector_yolox_s/v1/yolox_s_car_ped_tl_detector_960_960_batch_1.onnx
wget -P $DATA_DIR/tensorrt_yolox $BASE_URL/tl_detector_yolox_s/v1/car_ped_tl_detector_labels.txt

# traffic_light_classifier
wget -P $DATA_DIR/traffic_light_classifier $BASE_URL/traffic_light_classifier/v4/traffic_light_classifier_mobilenetv2_batch_6.onnx
wget -P $DATA_DIR/traffic_light_classifier $BASE_URL/traffic_light_classifier/v4/ped_traffic_light_classifier_mobilenetv2_batch_6.onnx
wget -P $DATA_DIR/traffic_light_classifier $BASE_URL/traffic_light_classifier/v4/lamp_labels.txt
wget -P $DATA_DIR/traffic_light_classifier $BASE_URL/traffic_light_classifier/v4/lamp_labels_ped.txt

# traffic_light_fine_detector
wget -P $DATA_DIR/traffic_light_fine_detector $BASE_URL/tlr_yolox_s/v3/tlr_car_ped_yolox_s_batch_6.onnx
wget -P $DATA_DIR/traffic_light_fine_detector $BASE_URL/tlr_yolox_s/v3/tlr_labels.txt

echo "Done. autoware_data downloaded to $DATA_DIR"
EOF
chmod +x ~/download_autoware_data.sh
~/download_autoware_data.sh
```

### Step 8: Launch the Autoware container and build

Launch the container with all required volumes mounted:

```bash
rocker --nvidia --x11 --privileged --net=host \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume ~/autoware \
    --volume ~/autoware_map \
    --volume ~/autoware_data \
    --name autoware_1_7_1 \
    -- ghcr.io/autowarefoundation/autoware:universe-devel-cuda-1.7.1
```

Inside the container, install the missing RViz dependency first:

```bash
apt-get update
apt-get install -y ros-humble-rviz-2d-overlay-plugins
```

Then build the workspace, skipping packages that consistently fail due to missing
GPU dependencies or external model files:

```bash
cd /home/skodas/autoware
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

**Note:** Use `-j4` and `--parallel-workers 4` to limit memory usage. If the build
crashes with `Killed signal terminated program cc1plus`, reduce to `-j2 --parallel-workers 2`.

**Known build issues and fixes:**

| Issue | Fix |
|---|---|
| `acados` missing `blasfeo`/`hpipm` | Run `git submodule update --init --recursive` in `src/universe/external/acados/` on the host, then skip acados in the build |
| `autoware_string_stamped_rviz_plugin` missing header | Run `apt-get install -y ros-humble-rviz-2d-overlay-plugins` inside container, delete `build/autoware_string_stamped_rviz_plugin/` and rebuild |
| `g++: fatal error: Killed signal` | Out of memory — reduce parallel workers |
| `ros-humble-rviz-2d-overlay-plugins not found` via apt | Use `apt-get update` first, then install |

### Step 9: Create the autoware_data symlink (every container restart)

The Autoware launch files expect model files at `/root/autoware_data/` but the
volume is mounted at `/home/skodas/autoware_data/`. Create a symlink:

```bash
ln -s /home/skodas/autoware_data /root/autoware_data 2>/dev/null || true
```

**This must be done every time the Autoware container is restarted.**

---

## Part 3: Running the Full Stack

Once all one-time setup is complete, this is the complete launch sequence every time.

### Terminal Layout

Use 5 terminals:
- **Terminal 1**: CARLA (host)
- **Terminal 2**: Bridge container — C++ bridge
- **Terminal 3**: Bridge container — leaderboard evaluator
- **Terminal 4**: Autoware container — Autoware launch
- **Terminal 5**: Host — monitoring and fixes

### Terminal 1: Launch CARLA with native DDS

```bash
cd /home/skodas/carla
make launch-dds
# Press Play in the CARLA editor once it opens
# CARLA loads Town10HD_Opt by default — do NOT switch maps
```

### Terminal 2: Start the bridge container and C++ bridge

If the container is not already running:

```bash
rocker --nvidia --x11 --privileged --net=host \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume ~/autoware_carla_leaderboard \
    --name aw_carla_leaderboard_0916 \
    -- tumgeka/autoware_carla_leaderboard:0.9.16
```

Inside the container — run these every restart:

```bash
cd /home/skodas/autoware_carla_leaderboard
source install/setup.bash
source carla_envs.sh

# CRITICAL: Reinstall source-built wheel every container restart
pip3 install carla-0.9.16-cp310-cp310-linux_x86_64.whl --force-reinstall

ros2 launch autoware_carla_cpp_bridge aw_carla_cpp_bridge.launch.py
```

The bridge will print `[DEBUG] Calling timer` every 100ms while waiting for CARLA.
Once CARLA Play is pressed, ROS2 topics will start appearing.

### Terminal 3: Open a second shell in the bridge container

```bash
docker exec -it aw_carla_leaderboard_0916 bash
cd /home/skodas/autoware_carla_leaderboard
source install/setup.bash
source carla_envs.sh

# CRITICAL: Reinstall source-built wheel every container restart
pip3 install carla-0.9.16-cp310-cp310-linux_x86_64.whl --force-reinstall
```

Verify the bridge is working:

```bash
ros2 topic list
```

You should see topics including `/carla/clock`, `/carla/ego_vehicle/sensor/camera/image`,
`/sensing/gnss/pose_with_covariance`, `/sensing/imu/imu_data`, etc.

Verify CARLA connectivity:

```bash
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
print('Server:', client.get_server_version())
print('Client:', client.get_client_version())
world = client.get_world()
print('Map:', world.get_map().name)
"
```

Both versions should report `0.9.16` and `get_world()` must succeed without crashing.

### Terminal 4: Launch the Autoware container and Autoware

If the container is not already running:

```bash
rocker --nvidia --x11 --privileged --net=host \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume ~/autoware \
    --volume ~/autoware_map \
    --volume ~/autoware_data \
    --name autoware_1_7_1 \
    -- ghcr.io/autowarefoundation/autoware:universe-devel-cuda-1.7.1
```

Inside the container — run these every restart:

```bash
# Create autoware_data symlink (required every restart)
ln -s /home/skodas/autoware_data /root/autoware_data 2>/dev/null || true

# Source the workspace
source /home/skodas/autoware/install/setup.bash
```

Launch Autoware with Town10 map (matches Town10HD_Opt in CARLA):

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
    vehicle_model:=carla_audi_etron_vehicle \
    sensor_model:=carla_audi_etron_sensor_kit \
    map_path:=/home/skodas/autoware_map/carla/Town10
```

**RViz notes:**
- Initial Fixed Frame error "Frame [map] does not exist" is expected — set Fixed Frame
  to `base_link` temporarily
- Once localization initializes, switch back to `map`
- Add PointCloud2 displays for `/sensing/lidar/front/pointcloud` and
  `/sensing/lidar/rear/pointcloud` to visualize LiDAR data

Wait for Autoware to fully initialize (50+ nodes visible via `ros2 node list | wc -l`)
before running the evaluator.

### Terminal 3: Run the leaderboard evaluator

Only after Autoware is fully launched:

```bash
python3 src/external/leaderboard/leaderboard/leaderboard_evaluator.py \
    --conf_file_path=config/config.yaml
```

Expected output on first successful run:
```
========= Preparing RouteScenario_0 (repetition 0) =========
> Loading the world
Map Town10HD_Opt already loaded, skipping reload.
> Setting up the agent
> Running the route
=== [Agent] -- Wallclock = ... -- System time = ... -- Game time = ...
```

---

## Architecture Overview

```
Host Machine
├── Terminal 1: CARLA 0.9.16 (source build, native DDS via -ros2 flag)
│   └── Publishes sensor data via FastDDS
│
├── Terminal 2/3: autoware_carla_leaderboard container (CycloneDDS)
│   ├── C++ bridge: subscribes FastDDS → republishes as ROS2 CycloneDDS topics
│   └── Leaderboard evaluator: runs scenarios, spawns ego vehicle
│
└── Terminal 4: Autoware 1.7.1 container (CycloneDDS)
    └── Subscribes to bridge topics, runs planning/control, publishes commands
```

**Why two DDS implementations?**
CARLA 0.9.16 native DDS uses FastDDS. Autoware and the bridge use CycloneDDS.
The C++ bridge (`aw_carla_cpp_bridge`) translates between them. This is why
`ros2 topic list` on the host does not show CARLA topics directly — topics
only appear inside the bridge container after the C++ bridge is running.

---

## Known Issues and Current Status

### Pointcloud Map Not Loading
The `pointcloud_map_loader` component is missing from the map container. This
causes localization to fail since NDT scan matching has no map to match against.
As a result:
- No `map` → `base_link` transform is established
- Autoware cannot determine vehicle position
- Route planning fails
- `RouteCompletionTest` reports 0% completion

**Status:** Under investigation. The `pointcloud_map_loader` package needs to be
verified as built and loading correctly into the map container.

**Workaround:** Use privileged mode (`aw_priviliged.py`) which provides ground truth
localization from CARLA directly, bypassing NDT scan matching entirely.

### GPU Renderer Crash on Map Switch
Dynamically switching CARLA maps (e.g., from Town10HD to Town01) causes a
segmentation fault in `FDistanceFieldVolumeTexture` and the UE4 deferred
shading renderer. This is a GPU memory issue with the RTX 4080 Laptop (12GB VRAM)
running UE4Editor mode.

**Workaround:** Always use Town10HD_Opt routes — this is the default CARLA map
and requires no switching.

**Permanent fix:** Package CARLA (`make package`) to run as a standalone binary
instead of UE4Editor, which significantly reduces VRAM usage.

### Simulation Speed
The simulation runs at approximately 4.5% real-time speed (0.045x ratio). This
is caused by running CARLA in UE4Editor mode simultaneously with Autoware.
Packaging CARLA would improve this significantly.

---

## File Structure After Setup

```
~/ (home directory)
├── autoware/                          # Autoware 1.7.1 source + built workspace
│   └── src/carla_audi_etron/          # Vehicle + sensor kit for CARLA
├── autoware_carla_leaderboard/        # Bridge repo + built workspace
│   ├── config/config.yaml             # Scenario configuration
│   └── carla-0.9.16-cp310-cp310-linux_x86_64.whl  # Source-built CARLA wheel
├── autoware_map/
│   └── carla/
│       ├── Town01/                    # Lanelet2 map assets
│       ├── Town07/
│       └── Town10/
├── autoware_data/                     # ML model artifacts
│   ├── lidar_centerpoint/
│   ├── tensorrt_yolox/
│   ├── traffic_light_classifier/
│   └── traffic_light_fine_detector/
└── download_autoware_data.sh          # Script to re-download artifacts if needed
```

---

## Container Restart Checklist

Every time a container restarts it resets to its original image. Always run:

**Bridge container (Terminals 2 and 3):**
```bash
cd /home/skodas/autoware_carla_leaderboard
source install/setup.bash
source carla_envs.sh
pip3 install carla-0.9.16-cp310-cp310-linux_x86_64.whl --force-reinstall
```

**Autoware container (Terminal 4):**
```bash
ln -s /home/skodas/autoware_data /root/autoware_data 2>/dev/null || true
source /home/skodas/autoware/install/setup.bash
```

---

## Python Dependency Notes

During the source build attempt (before switching to Docker), the following Python
packages were required. These are already pre-installed in the Docker images but
are noted here for reference:

| Package | Version | Purpose |
|---|---|---|
| `setuptools` | `==58.2.0` | Legacy `setup.py` / colcon compatibility |
| `jinja2` | latest | `autoware_system_designer` launch file generation |
| `typeguard` | latest | Required by `generate-parameter-library-py` |
| `jsonschema` | latest | Required by system design packages |