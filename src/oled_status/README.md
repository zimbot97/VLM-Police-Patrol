# oled_status

SSD1306 0.96" (128×64, I2C) status HUD for the **VLM-Police-Patrol** robot on the RDK X5.

Top line shows the robot IP, the line below shows running status, and the main area
animates by state. Everything is driven off topics that already exist in the workspace —
no changes to `suspect_matcher` or `dashboard_flask` are required for the core behaviour.

## States (priority high → low)

| State | Trigger | Graphic |
|---|---|---|
| SUSPECT MATCH | `/suspect_feature_match` = `True` | police badge + flashing beacon dot (latched ~6 s) |
| NO MATCH | `/suspect_feature_match` = `False` | blinking cross (latched ~6 s) |
| ANALYZING… | `/prompt_text` seen (VLM in flight) | rotating spinner |
| CAPTURING… | `/oled/state` = `"capturing"` (optional) | rotating spinner |
| HUMAN DETECTED | `/yolo/detections` targets > 0, or `/human_present` = `True` | person icon |
| MONITORING | default | CCTV lens animation |

VLM loading is auto-detected from `/prompt_text` and cleared when the match Bool
arrives or after `vlm_timeout_sec`. The distinct CAPTURING screen only appears if
something publishes `"capturing"` to `/oled/state` (see optional hook below).

## Subscriptions
- `/suspect_feature_match` (std_msgs/Bool)
- `/prompt_text` (std_msgs/String)
- `/yolo/detections` (ai_msgs/PerceptionTargets) — optional, guarded import
- `/human_present` (std_msgs/Bool) — fallback if ai_msgs unavailable
- `/oled/state` (std_msgs/String) — optional override: `capturing` | `idle`

## Parameters
| Param | Default | Notes |
|---|---|---|
| `i2c_bus` | `5` | I2C5 on RDK X5 (pins 3/5) |
| `i2c_address` | `0x3C` | some panels enumerate at `0x3D` |
| `ip_interface` | `wlan0` | preferred iface; auto-falls back to others |
| `fps` | `15.0` | render rate |
| `enable_display` | `true` | set false to run headless (logs state only) |
| `vlm_timeout_sec` | `900.0` | matches suspect_matcher launch arg |
| `show_stats` | `true` | right-side CPU%/CPU°C/BPU°C column |

## Install & build
```bash
sudo pip3 install luma.oled          # pulls luma.core + Pillow
i2cdetect -y -r 5                    # confirm 0x3C on bus 5

cd ~/ros2_ws                         # your workspace root
colcon build --merge-install --packages-select oled_status
source install/setup.bash
```

## Run
```bash
# via launch (recommended)
ros2 launch oled_status oled_status.launch.py ip_interface:=wlan0

# or directly
ros2 run oled_status oled_status --ros-args \
  -p i2c_bus:=5 -p i2c_address:=0x3c -p ip_interface:=wlan0
```

## Optional: distinct CAPTURING screen from the dashboard
Add around the `/capture_crop` call in `flask_node.py`:
```python
# once, in __init__:
self.oled_state_pub = self.create_publisher(String, '/oled/state', 10)

# around the capture:
self.oled_state_pub.publish(String(data='capturing'))
# ... call /capture_crop ...
self.oled_state_pub.publish(String(data='idle'))
```
Or from a shell for testing:
```bash
ros2 topic pub -1 /oled/state std_msgs/String "{data: capturing}"
```

## System stats (right column)

When `show_stats:=true` the right side shows live **CPU usage**, **CPU temp**, and
**BPU temp**, read on the RDK X5 from:

```
/proc/stat                              # CPU % (delta-based, sampled 1 Hz)
/sys/class/hwmon/hwmon0/temp3_input     # CPU temp (millideg C)
/sys/class/hwmon/hwmon0/temp2_input     # BPU temp (millideg C)
```

The BPU sensor only powers up while the BPU is running, so BPU temp shows `--`
when idle and a value once InternVL/YOLO inference is active. The node auto-discovers
the correct `hwmonN` dir (falls back to `hwmon0`). Disable the column with
`show_stats:=false` to centre the icon full-width.

## Notes
- Runs on a single-threaded executor; subscriptions update state, a timer renders.
- If luma/Pillow or the panel are missing, the node logs state transitions and
  keeps running (useful for dev-machine testing).
