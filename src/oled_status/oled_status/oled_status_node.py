#!/usr/bin/env python3
"""
oled_status_node — SSD1306 0.96" (128x64) status HUD for the VLM-Police-Patrol robot.

Layout (128x64), with stats column enabled:
    y 0..9    IP address (top, full width)             + live blink dot
    y 10      horizontal divider
    LEFT  (x 0..83) : status label (y12) + animated icon (centred ~x42,y40)
    RIGHT (x 86..127): CPU% / CPU degC / BPU degC   (vertical divider at x84)

State priority (highest wins):
    MATCH    -> police badge  + flashing beacon dot    (/suspect_feature_match = True)
    NOMATCH  -> blinking cross                         (/suspect_feature_match = False)
    ANALYZE  -> spinner   (VLM in flight, from /prompt_text)
    CAPTURE  -> spinner   (optional /oled/state = "capturing")
    HUMAN    -> person icon   (/yolo/detections targets, or /human_present)
    PATROL   -> CCTV lens animation (default)

RDK X5 sensors (D-Robotics docs), millidegrees C, /sys/class/hwmon/hwmon0/:
    temp1_input = DDR, temp2_input = BPU, temp3_input = CPU
    (BPU only reads valid while the BPU is running.)
CPU usage is derived from /proc/stat deltas.

Bus: I2C5 -> Linux bus 5 on this board (proven via play_bin_x5.py --port 5). Addr 0x3C.
Deps: sudo pip3 install luma.oled   (luma.core + Pillow)
Degrades to headless logging if luma/Pillow or the panel are missing.
"""

import glob
import math
import os
import socket
import struct
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    from PIL import Image, ImageDraw, ImageFont
    HAVE_DISPLAY = True
except Exception as _e:              # noqa: BLE001
    HAVE_DISPLAY = False
    _DISPLAY_IMPORT_ERR = _e

try:
    from ai_msgs.msg import PerceptionTargets
    HAVE_AI_MSGS = True
except Exception:                    # noqa: BLE001
    HAVE_AI_MSGS = False

try:
    import fcntl
    SIOCGIFADDR = 0x8915
    HAVE_FCNTL = True
except Exception:                    # noqa: BLE001
    HAVE_FCNTL = False


# --------------------------------------------------------------------------
# Network + system-metric helpers
# --------------------------------------------------------------------------
def _iface_ip(ifname: str):
    if not HAVE_FCNTL:
        return None
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = struct.pack('256s', ifname[:15].encode('utf-8'))
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFADDR, packed)[20:24])
    except OSError:
        return None
    finally:
        s.close()


def get_ip(preferred_iface: str):
    for ifn in [preferred_iface, 'wlan0', 'ap0', 'uap0', 'eth0', 'usb0']:
        if not ifn:
            continue
        ip = _iface_ip(ifn)
        if ip and not ip.startswith('127.'):
            return ip
    return None


def find_hwmon():
    """Return the hwmon dir that has the X5 temp sensors (temp3_input = CPU)."""
    for d in sorted(glob.glob('/sys/class/hwmon/hwmon*')):
        if os.path.exists(os.path.join(d, 'temp3_input')):
            return d
    return '/sys/class/hwmon/hwmon0'


def read_millideg(path):
    try:
        with open(path) as f:
            v = int(f.read().strip())
        return v / 1000.0 if v > 0 else None
    except Exception:                # noqa: BLE001
        return None


def read_cpu_times():
    """(total, idle) jiffies from the aggregate line of /proc/stat."""
    try:
        with open('/proc/stat') as f:
            parts = f.readline().split()[1:]
        vals = [int(x) for x in parts]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle + iowait
        return sum(vals), idle
    except Exception:                # noqa: BLE001
        return None, None


# --------------------------------------------------------------------------
# Node
# --------------------------------------------------------------------------
class OledStatusNode(Node):
    RESULT_HOLD = 6.0
    HUMAN_HOLD = 1.2
    CAPTURE_TIMEOUT = 15.0
    IP_REFRESH = 3.0

    LABELS = {
        'MATCH': 'MATCH', 'NOMATCH': 'NO MATCH',
        'ANALYZING': 'ANALYZE', 'CAPTURING': 'CAPTURE',
        'HUMAN': 'HUMAN', 'NORMAL': 'PATROL',
    }

    def __init__(self):
        super().__init__('oled_status')

        # ---- params ----
        self.bus = self.declare_parameter('i2c_bus', 5).value
        self.addr = self.declare_parameter('i2c_address', 0x3C).value
        self.iface = self.declare_parameter('ip_interface', 'wlan0').value
        self.fps = float(self.declare_parameter('fps', 15.0).value)
        self.enable_display = bool(self.declare_parameter('enable_display', True).value)
        self.vlm_timeout = float(self.declare_parameter('vlm_timeout_sec', 900.0).value)
        self.show_stats = bool(self.declare_parameter('show_stats', True).value)

        # ---- state ----
        self.frame = 0
        self.ip_str = 'NO NETWORK'
        self._last_ip_read = 0.0

        self.match_val = None
        self.match_t = 0.0
        self.vlm_t = 0.0
        self.capture_t = 0.0
        self.human_t = 0.0
        self._last_logged_state = None

        # ---- system metrics ----
        self.cpu_pct = None
        self.cpu_temp = None
        self.bpu_temp = None
        self._prev_cpu = read_cpu_times()
        self._hwmon = find_hwmon()

        # ---- subscriptions ----
        self.create_subscription(Bool, '/suspect_feature_match', self._on_match, 10)
        self.create_subscription(String, '/prompt_text', self._on_prompt, 10)
        self.create_subscription(String, '/oled/state', self._on_oled_state, 10)
        self.create_subscription(Bool, '/human_present', self._on_human_bool, 10)
        if HAVE_AI_MSGS:
            self.create_subscription(PerceptionTargets, '/yolo/detections',
                                     self._on_detections, 10)
        else:
            self.get_logger().warn(
                'ai_msgs not found — human detection uses /human_present (Bool).')

        # ---- display ----
        self.device = None
        self.font_ip = None
        self.font_lbl = None
        self.font_stat = None
        if self.enable_display:
            self._init_display()
        else:
            self.get_logger().warn('enable_display=false — running headless.')

        # ---- timers ----
        self.create_timer(1.0 / max(1.0, self.fps), self._render)
        self.create_timer(1.0, self._sample_stats)
        self._sample_stats()

        self.get_logger().info(
            f'oled_status up (bus={self.bus} addr=0x{self.addr:02X} '
            f'stats={"on" if self.show_stats else "off"} '
            f'hwmon={self._hwmon} display={"on" if self.device else "off"}).')

    # ---------------- display init ----------------
    def _init_display(self):
        if not HAVE_DISPLAY:
            self.get_logger().error(
                f'luma.oled / Pillow unavailable ({_DISPLAY_IMPORT_ERR}); headless. '
                'Install: sudo pip3 install luma.oled')
            return
        try:
            serial = i2c(port=self.bus, address=self.addr)
            self.device = ssd1306(serial, width=128, height=64)
        except Exception as e:                        # noqa: BLE001
            self.get_logger().error(
                f'Cannot open SSD1306 on bus {self.bus}: {e}. '
                f'Check "i2cdetect -y {self.bus}". Headless.')
            self.device = None
            return
        try:
            path = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
            self.font_ip = ImageFont.truetype(path, 9)
            self.font_lbl = ImageFont.truetype(path, 11)
            self.font_stat = ImageFont.truetype(path, 8)
        except Exception:                             # noqa: BLE001
            self.font_ip = self.font_lbl = self.font_stat = ImageFont.load_default()

    # ---------------- callbacks ----------------
    def _on_match(self, msg):
        self.match_val = bool(msg.data)
        self.match_t = time.time()
        self.vlm_t = 0.0

    def _on_prompt(self, _msg):
        self.vlm_t = time.time()
        self.match_val = None

    def _on_oled_state(self, msg):
        s = (msg.data or '').strip().lower()
        if s == 'capturing':
            self.capture_t = time.time()
        elif s in ('idle', 'clear', 'normal'):
            self.capture_t = 0.0
            self.vlm_t = 0.0
            self.match_val = None

    def _on_human_bool(self, msg):
        if msg.data:
            self.human_t = time.time()

    def _on_detections(self, msg):
        n = 0
        try:
            for t in msg.targets:
                if (not getattr(t, 'type', '')) or 'person' in t.type.lower():
                    n += 1
        except Exception:            # noqa: BLE001
            n = len(getattr(msg, 'targets', []))
        if n > 0:
            self.human_t = time.time()

    # ---------------- metric sampling (1 Hz) ----------------
    def _sample_stats(self):
        total, idle = read_cpu_times()
        if total is not None and self._prev_cpu[0] is not None:
            dt = total - self._prev_cpu[0]
            di = idle - self._prev_cpu[1]
            if dt > 0:
                self.cpu_pct = max(0.0, min(100.0, 100.0 * (dt - di) / dt))
        self._prev_cpu = (total, idle)
        self.cpu_temp = read_millideg(os.path.join(self._hwmon, 'temp3_input'))
        self.bpu_temp = read_millideg(os.path.join(self._hwmon, 'temp2_input'))

    # ---------------- state machine ----------------
    def _state(self):
        now = time.time()
        if self.match_val is not None and (now - self.match_t) < self.RESULT_HOLD:
            return 'MATCH' if self.match_val else 'NOMATCH'
        if self.vlm_t and (now - self.vlm_t) < self.vlm_timeout:
            return 'ANALYZING'
        if self.capture_t and (now - self.capture_t) < self.CAPTURE_TIMEOUT:
            return 'CAPTURING'
        if (now - self.human_t) < self.HUMAN_HOLD:
            return 'HUMAN'
        return 'NORMAL'

    # ---------------- render ----------------
    def _render(self):
        now = time.time()
        if now - self._last_ip_read > self.IP_REFRESH:
            self._last_ip_read = now
            ip = get_ip(self.iface)
            self.ip_str = ip if ip else 'NO NETWORK'

        state = self._state()
        if state != self._last_logged_state:
            self.get_logger().info(f'state -> {state}')
            self._last_logged_state = state

        self.frame += 1
        if self.device is None:
            return

        img = Image.new('1', (128, 64), 0)
        d = ImageDraw.Draw(img)

        # header
        d.text((0, 0), self.ip_str, font=self.font_ip, fill=1)
        if not self.show_stats and int(now * 1.5) % 2:
            d.ellipse([122, 1, 126, 5], fill=1)
        d.line([(0, 10), (127, 10)], fill=1)

        # left zone geometry
        left_w = 84 if self.show_stats else 128
        cx = 42 if self.show_stats else 64
        cy = 40

        self._text_center(d, self.LABELS[state], 12, self.font_lbl, x0=0, zone_w=left_w)

        if state == 'NORMAL':
            self._draw_cctv(d, cx, cy)
        elif state == 'HUMAN':
            self._draw_human(d, cx, cy)
        elif state in ('ANALYZING', 'CAPTURING'):
            self._draw_spinner(d, cx, cy)
        elif state == 'MATCH':
            self._draw_badge(d, cx, cy, now)
        elif state == 'NOMATCH':
            self._draw_cross(d, cx, cy, now)

        if self.show_stats:
            self._draw_stats(d)

        self.device.display(img)

    # ---------------- drawing helpers ----------------
    def _text_center(self, d, text, y, font, x0=0, zone_w=128):
        try:
            bbox = d.textbbox((0, 0), text, font=font)
            w = bbox[2] - bbox[0]
        except Exception:            # noqa: BLE001
            w = len(text) * 6
        d.text((x0 + (zone_w - w) // 2, y), text, font=font, fill=1)

    def _draw_stats(self, d):
        d.line([(84, 11), (84, 63)], fill=1)
        x = 88
        f = self.font_stat

        def fmt_t(v):
            return f'{v:.0f}C' if v is not None else '--'

        def fmt_p(v):
            return f'{v:.0f}%' if v is not None else '--'

        d.text((x, 13), 'CPU', font=f, fill=1)
        d.text((x + 2, 22), fmt_p(self.cpu_pct), font=f, fill=1)
        d.text((x + 2, 31), fmt_t(self.cpu_temp), font=f, fill=1)
        d.text((x, 45), 'BPU', font=f, fill=1)
        d.text((x + 2, 54), fmt_t(self.bpu_temp), font=f, fill=1)

    def _draw_cctv(self, d, cx, cy):
        R = 16
        pulse = 2 if (self.frame // 6) % 2 == 0 else 1
        d.ellipse([cx - R, cy - R, cx + R, cy + R], outline=1, width=pulse)
        d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], outline=1)
        d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=1)
        a = math.radians((self.frame * 12) % 360)
        gx, gy = cx + int(12 * math.cos(a)), cy + int(12 * math.sin(a))
        d.ellipse([gx - 2, gy - 2, gx + 2, gy + 2], fill=1)
        for k in range(4):
            ta = math.radians(k * 90 + 45)
            x1, y1 = cx + int(10 * math.cos(ta)), cy + int(10 * math.sin(ta))
            x2, y2 = cx + int(14 * math.cos(ta)), cy + int(14 * math.sin(ta))
            d.line([(x1, y1), (x2, y2)], fill=1)

    def _draw_spinner(self, d, cx, cy):
        R = 14
        a = (self.frame * 22) % 360
        d.arc([cx - R, cy - R, cx + R, cy + R], a, a + 270, fill=1, width=3)
        d.ellipse([cx - 2, cy - 2, cx + 2, cy + 2], fill=1)

    def _draw_human(self, d, cx, cy):
        hr = 6
        d.ellipse([cx - hr, cy - 16, cx + hr, cy - 16 + 2 * hr], fill=1)
        d.polygon([(cx - 4, cy - 3), (cx + 4, cy - 3),
                   (cx + 11, cy + 15), (cx - 11, cy + 15)], fill=1)

    def _draw_badge(self, d, cx, cy, now):
        hw = 15
        top, side_top, mid, bot = cy - 17, cy - 13, cy + 4, cy + 19
        pts = [(cx - hw, side_top), (cx - hw + 2, top), (cx + hw - 2, top),
               (cx + hw, side_top), (cx + hw, mid), (cx, bot), (cx - hw, mid)]
        d.polygon(pts, outline=1)
        d.polygon(self._star(cx, cy - 2, 8, 3.2), fill=1)
        if int(now * 3) % 2:
            d.ellipse([cx - 25, cy - 3, cx - 19, cy + 3], fill=1)
        else:
            d.ellipse([cx + 19, cy - 3, cx + 25, cy + 3], fill=1)

    def _draw_cross(self, d, cx, cy, now):
        R = 16
        d.ellipse([cx - R, cy - R, cx + R, cy + R], outline=1, width=2)
        if int(now * 2) % 2:
            r = 8
            for off in (-1, 0, 1):
                d.line([(cx - r, cy - r + off), (cx + r, cy + r + off)], fill=1)
                d.line([(cx - r, cy + r + off), (cx + r, cy - r + off)], fill=1)

    @staticmethod
    def _star(cx, cy, r_out, r_in, points=5):
        pts = []
        for i in range(points * 2):
            r = r_out if i % 2 == 0 else r_in
            ang = math.radians(-90 + i * (360.0 / (points * 2)))
            pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
        return pts

    def clear(self):
        if self.device is not None:
            try:
                self.device.clear()
            except Exception:        # noqa: BLE001
                pass


def main():
    rclpy.init()
    node = OledStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
