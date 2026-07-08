# dashboard_flask

A single ROS2 node combining rclpy + Flask/SocketIO into one web dashboard:

- **Camera** — MJPEG stream of `sensor_msgs/Image`
- **Map** — `nav_msgs/OccupancyGrid` rendered live as a PNG
- **AMCL pose** — `geometry_msgs/PoseWithCovarianceStamped`, pushed to the browser over WebSocket and drawn as a marker+heading arrow on top of the map
- **cmd_vel joystick** — drag pad in the browser publishes `geometry_msgs/Twist`

## Install

Drop this folder into your ROS2 workspace `src/`:

```
your_ws/
  src/
    dashboard_flask/
      package.xml
      setup.py
      setup.cfg
      resource/dashboard_flask
      dashboard_flask/
        __init__.py
        flask_node.py
        templates/index.html
```

Python deps (on the machine that runs the Flask node — the RDK X5 or your dev box):

```bash
pip install flask flask-socketio
```

(`rclpy`, `cv_bridge`, `sensor_msgs`, `nav_msgs`, `geometry_msgs` come from your ROS2 install — make sure you've sourced it before building.)

Build:

```bash
cd your_ws
colcon build --packages-select dashboard_flask --symlink-install
source install/setup.bash
```

## Run

Either directly:

```bash
ros2 run dashboard_flask flask_node
```

Or with the included launch script, which sources ROS2 (+ TogetherROS on the X5, if present) and the workspace, then starts the node with topic/port overrides as flags:

```bash
chmod +x launch_dashboard.sh   # first time only
./launch_dashboard.sh
./launch_dashboard.sh --image-topic /rdk_camera/image_raw --port 8080
./launch_dashboard.sh --help   # full list of flags
```

The script assumes it lives at `<ws>/src/dashboard_flask/launch_dashboard.sh` so it can find `install/setup.bash` two directories up. If you move it, pass `--ws-setup /path/to/install/setup.bash` explicitly. It also looks for `/opt/tros/humble/setup.bash` (RDK X5's TogetherROS layout) and sources it automatically if present — override with `--tros-setup` or the `TROS_SETUP` env var if your setup differs.

Then open `http://<robot-ip>:5000/` (or whatever `--port` you chose) from your laptop/phone on the same network.

## Remapping topics

Defaults assume `/camera/image_raw`, `/map`, `/amcl_pose`, `/cmd_vel`. Override via ROS2 params, e.g.:

```bash
ros2 run dashboard_flask flask_node --ros-args \
  -p image_topic:=/rdk_camera/image_raw \
  -p map_topic:=/map \
  -p pose_topic:=/amcl_pose \
  -p cmd_vel_topic:=/cmd_vel \
  -p jpeg_quality:=60
```

Or set them in a launch file / YAML params file the same way.

## Notes / gotchas

- **Map QoS**: Nav2's map topic is typically published `RELIABLE` + `TRANSIENT_LOCAL` (latched). The subscription QoS in `flask_node.py` matches this — if you swap in a different map source, check its QoS or you'll get nothing.
- **cv_bridge encoding**: assumes `bgr8`. If your camera publishes `rgb8`, `mono8`, or a compressed topic, either remap to a raw `bgr8`-compatible topic or extend `on_image()` accordingly (compressed input is a small change: subscribe to `sensor_msgs/CompressedImage` and skip cv_bridge entirely, decoding the JPEG bytes with `cv2.imdecode` instead).
- **Joystick send rate**: fixed at 10 Hz while dragging, and always sends one final zero-velocity command on release — no server-side deadman timeout is implemented, so if the browser tab crashes mid-drag the last nonzero cmd_vel keeps being the last thing published. Add a watchdog timer in the node (stop publishing / auto-zero after N ms without a socket event) before trusting this on a robot that can hurt something.
- **Multiple clients**: `cmd_vel` from any connected browser tab is accepted — there's no arbitration. Fine for one-operator use; add a "control lock" concept if more than one person might open the page at once.
- **Performance**: MJPEG loop is capped at ~20 fps server-side (`time.sleep(0.05)` in `mjpeg_generator`). Lower `jpeg_quality` or this interval if you're also running Nav2/YOLO/VLM on the same X5 and need to free up CPU.
