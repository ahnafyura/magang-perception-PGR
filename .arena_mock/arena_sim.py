#!/usr/bin/env python3
"""Arena 2D + ASV tiruan untuk misi Pengiriman Kapal Otonom (tanpa Gazebo/VRX, ringan, cukup RViz).

Frame: map (x ke Spot 2, y ke atas, origin pojok barat daya kolam, satuan meter).

Topic keluar
  /asv/odom (nav_msgs/Odometry), /asv/pose (PoseStamped), TF map->base_link
  /asv/battery (sensor_msgs/BatteryState)
  /arena/markers (visualization_msgs/MarkerArray)
  /mission_state (String), /mission/events (String JSON), /ftp/score (String JSON)
Topic masuk
  /asv/cmd_vel (geometry_msgs/Twist): linear.x (m/s) dan angular.z (rad/s), badan kapal
Service
  Trigger: /mission/start /mission/stop /mission/reset /estop /asv/pick /asv/place
  SetBool: /asv/buzzer /asv/spray
Parameter: capacity (barang per trip, default 1), tau_v, tau_w, current_x, current_y, viz_scale
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))
A = json.load(open(os.path.join(HERE, "arena.json")))

COL = dict(green=(0.18, 0.62, 0.31), red=(0.82, 0.2, 0.17), yellow=(0.95, 0.76, 0.19), black=(0.1, 0.1, 0.1),
           wood=(0.55, 0.43, 0.31), water=(0.55, 0.75, 0.9), white=(0.95, 0.95, 0.95), navy=(0.04, 0.12, 0.18), blue=(0.12, 0.43, 0.71))
FTP_X0, FTP_X1 = 4.5, 19.5        # garis masuk dan keluar area FTP
FTP_Y0, FTP_Y1 = 2.6, 9.4         # batas samping area FTP
MAX_ATTEMPT = 3
REACH = 1.4                       # jangkauan mekanisme (m) dari pusat kapal


def rgba(c, a=1.0):
    return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(a))


def yaw_q(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class Arena:
    """Geometri statis arena."""

    def __init__(self):
        a = A
        self.W, self.H = a["W"], a["H"]
        dw, dh, sd, sw = a["dock_w"], a["dock_h"], a["slot_d"], a["slot_w"]
        cy = self.H / 2
        self.slot1 = (dw - sd, cy - sw / 2, dw, cy + sw / 2)
        self.slot2 = (self.W - dw, cy - sw / 2, self.W - dw + sd, cy + sw / 2)
        side = (dh - sw) / 2
        self.blocks = [(0, cy - dh / 2, dw - sd, cy + dh / 2), (dw - sd, cy - dh / 2, dw, cy - sw / 2), (dw - sd, cy + sw / 2, dw, cy + dh / 2),
                       (self.W - dw + sd, cy - dh / 2, self.W, cy + dh / 2), (self.W - dw, cy - dh / 2, self.W - dw + sd, cy - sw / 2), (self.W - dw, cy + sw / 2, self.W - dw + sd, cy + dh / 2)]
        self.buoys = [(x, y, "green", a["buoy_r"]) for x, y in a["green"]] + [(x, y, "red", a["buoy_r"]) for x, y in a["red"]] + \
                     [(o["x"], o["y"], o["color"], a["obs_r"]) for o in a["obstacles"]]
        self.items = [(i["shape"], i["x"], cy + i["dy"]) for i in a["items"]]
        self.drops = [(i["shape"], self.W - i["x"], cy + i["dy"]) for i in a["items"]]
        self.gates = [tuple(g) for g in a["gates"]]


def inside(r, x, y, m=0.0):
    return r[0] - m <= x <= r[2] + m and r[1] - m <= y <= r[3] + m


class ArenaSim(Node):
    def __init__(self):
        super().__init__("arena_sim")
        self.ar = Arena()
        for k, v in (("capacity", 1), ("tau_v", 0.6), ("tau_w", 0.4), ("v_max", 1.0), ("w_max", 1.5), ("current_x", 0.0), ("current_y", 0.0), ("viz_scale", 2.0)):
            self.declare_parameter(k, v)
        self.L, self.B = A["asv_L"], A["asv_B"]
        self.reset_all()
        self.cmd = Twist()
        self.last_cmd_t = 0.0
        self.create_subscription(Twist, "/asv/cmd_vel", self.on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, "/asv/odom", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/asv/pose", 10)
        self.pub_bat = self.create_publisher(BatteryState, "/asv/battery", 10)
        self.pub_mark = self.create_publisher(MarkerArray, "/arena/markers", 10)
        self.pub_state = self.create_publisher(String, "/mission_state", 10)
        self.pub_ev = self.create_publisher(String, "/mission/events", 50)
        self.pub_score = self.create_publisher(String, "/ftp/score", 10)
        self.tf = TransformBroadcaster(self)
        for n, f in (("/mission/start", self.s_start), ("/mission/stop", self.s_stop), ("/mission/reset", self.s_reset), ("/estop", self.s_estop),
                     ("/asv/pick", self.s_pick), ("/asv/place", self.s_place)):
            self.create_service(Trigger, n, f)
        self.create_service(SetBool, "/asv/buzzer", self.s_buzzer)
        self.create_service(SetBool, "/asv/spray", self.s_spray)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_markers)
        self.create_timer(1.0, self.publish_slow)
        self.get_logger().info("arena_sim siap. Kapal ada di slot Spot 1. Kirim /asv/cmd_vel, lalu panggil /mission/start.")

    # ------------------------------------------------------------ keadaan
    def reset_all(self):
        a = self.ar
        sx = (a.slot1[0] + a.slot1[2]) / 2 - 0.2
        self.x, self.y, self.yaw = sx, self.ar.H / 2, 0.0
        self.v = self.w = 0.0
        self.running, self.estop, self.done = False, False, False
        self.item_pos = {i: (s, x, y) for i, (s, x, y) in enumerate(a.items)}   # di dermaga Spot 1
        self.item_state = {i: "spot1" for i in range(len(a.items))}              # spot1, carried, spot2
        self.carried = []
        self.buzzer = self.spray = False
        self.buzzer_t = self.spray_t = None
        self.last_place_t = None
        self.touching = set()
        self.collisions_total = 0
        self.run = None
        self.attempts = {1: 0, -1: 0}
        self.runs_log = []
        self.ftp_locked = {1: False, -1: False}
        self.battery = 100.0
        self.trail = []
        self.t_start = None
        self.summary = None

    def ev(self, kind, **kw):
        d = dict(t=round(self.now(), 2), event=kind, **kw)
        self.pub_ev.publish(String(data=json.dumps(d)))
        self.get_logger().info(json.dumps(d))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------ service
    def s_start(self, _q, r):
        if self.estop:
            r.success, r.message = False, "E-STOP aktif, panggil /mission/reset"
            return r
        self.running = True
        self.t_start = self.t_start or self.now()
        self.ev("start")
        r.success, r.message = True, "misi dimulai"
        return r

    def s_stop(self, _q, r):
        self.running = False
        self.ev("stop")
        r.success, r.message = True, "misi dihentikan"
        return r

    def s_reset(self, _q, r):
        self.reset_all()
        self.ev("reset")
        r.success, r.message = True, "arena direset"
        return r

    def s_estop(self, _q, r):
        self.estop, self.running = True, False
        self.v = self.w = 0.0
        self.ev("estop")
        r.success, r.message = True, "E-STOP aktif"
        return r

    def docked(self, slot):
        return inside(slot, self.x, self.y) and abs(self.yaw) < math.radians(35) and abs(self.v) < 0.15

    def s_pick(self, _q, r):
        cap = int(self.get_parameter("capacity").value)
        if not self.running or self.estop:
            r.success, r.message = False, "misi belum jalan atau E-STOP"
            return r
        if not inside(self.ar.slot1, self.x, self.y, 0.1):
            r.success, r.message = False, "kapal tidak di dalam slot Spot 1"
            return r
        if len(self.carried) >= cap:
            r.success, r.message = False, f"kapasitas penuh ({cap})"
            return r
        best, bd = None, REACH
        for i, (s, ix, iy) in self.item_pos.items():
            if self.item_state[i] == "spot1":
                d = math.hypot(ix - self.x, iy - self.y)
                if d < bd:
                    best, bd = i, d
        if best is None:
            r.success, r.message = False, f"tidak ada barang dalam jangkauan {REACH} m"
            return r
        self.item_state[best] = "carried"
        self.carried.append(best)
        self.ev("pick", item=self.item_pos[best][0], carried=len(self.carried))
        r.success, r.message = True, f"mengambil {self.item_pos[best][0]}"
        return r

    def s_place(self, _q, r):
        if not self.running or self.estop:
            r.success, r.message = False, "misi belum jalan atau E-STOP"
            return r
        if not inside(self.ar.slot2, self.x, self.y, 0.1):
            r.success, r.message = False, "kapal tidak di dalam slot Spot 2"
            return r
        if not self.carried:
            r.success, r.message = False, "tidak membawa barang"
            return r
        free = [(k, d) for k, d in enumerate(self.ar.drops) if all(self.item_pos[i][0] != d[0] or self.item_state[i] != "spot2" for i in self.item_pos)]
        free = [(k, d) for k, d in free if math.hypot(d[1] - self.x, d[2] - self.y) < REACH]
        if not free:
            r.success, r.message = False, f"tidak ada zona taruh dalam jangkauan {REACH} m"
            return r
        i = self.carried.pop(0)
        shape = self.item_pos[i][0]
        mine = [f for f in free if f[1][0] == shape]
        k, d = (mine or free)[0]
        self.item_pos[i] = (shape, d[1], d[2])
        self.item_state[i] = "spot2"
        self.last_place_t = self.now()
        n = sum(1 for s in self.item_state.values() if s == "spot2")
        self.ev("place", item=shape, placed=n)
        if n == len(self.item_state):
            self.ev("last_item")
        r.success, r.message = True, f"menaruh {shape}"
        return r

    def s_buzzer(self, q, r):
        self.buzzer = q.data
        self.buzzer_t = self.now() if q.data else self.buzzer_t
        self.ev("buzzer", on=q.data)
        r.success, r.message = True, "ok"
        return r

    def s_spray(self, q, r):
        self.spray = q.data
        if q.data:
            self.spray_t = self.now()
            hit = self.docked(self.ar.slot2)
            self.ev("spray", on=True, hit_banner=hit)
        else:
            self.ev("spray", on=False)
        r.success, r.message = True, "ok"
        return r

    def on_cmd(self, m):
        self.cmd, self.last_cmd_t = m, self.now()

    # ------------------------------------------------------------ fisika
    def corners_samples(self, x, y, yaw, n=8):
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for u in np.linspace(-self.L / 2, self.L / 2, n):
            for v in (-self.B / 2, self.B / 2):
                pts.append((x + u * c - v * s, y + u * s + v * c))
        for v in np.linspace(-self.B / 2, self.B / 2, 4):
            for u in (-self.L / 2, self.L / 2):
                pts.append((x + u * c - v * s, y + u * s + v * c))
        return pts

    def blocked(self, x, y, yaw):
        for px, py in self.corners_samples(x, y, yaw):
            if not (0 <= px <= self.ar.W and 0 <= py <= self.ar.H):
                return True
            for b in self.ar.blocks:
                if inside(b, px, py):
                    return True
        return False

    def buoy_hits(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hit = set()
        for k, (bx, by, _col, r) in enumerate(self.ar.buoys):
            dx, dy = bx - self.x, by - self.y
            u, v = dx * c + dy * s, -dx * s + dy * c
            cu, cv = min(max(u, -self.L / 2), self.L / 2), min(max(v, -self.B / 2), self.B / 2)
            if math.hypot(u - cu, v - cv) < r:
                hit.add(k)
        return hit

    def step(self):
        dt, g = self.dt, self.get_parameter
        t = self.now()
        cmd = self.cmd if (t - self.last_cmd_t) < 0.5 else Twist()       # watchdog 0,5 s
        if self.estop or not self.running:
            cmd = Twist()
        vmax, wmax = g("v_max").value, g("w_max").value
        vc, wc = max(-0.3, min(vmax, cmd.linear.x)), max(-wmax, min(wmax, cmd.angular.z))
        self.v += (vc - self.v) * dt / max(g("tau_v").value, 1e-3)
        self.w += (wc - self.w) * dt / max(g("tau_w").value, 1e-3)
        nyaw = self.yaw + self.w * dt
        nx = self.x + (self.v * math.cos(nyaw) + g("current_x").value) * dt
        ny = self.y + (self.v * math.sin(nyaw) + g("current_y").value) * dt
        if self.blocked(nx, ny, nyaw):
            if self.v != 0:
                self.ev("bump_dock_or_wall")
            self.v = self.w = 0.0
        else:
            px, py = self.x, self.y
            self.x, self.y, self.yaw = nx, ny, nyaw
            self.track_ftp(px, py)
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        # tabrakan buoy: dihitung sekali per sentuhan
        hit = self.buoy_hits()
        for k in hit - self.touching:
            self.collisions_total += 1
            if self.run is not None:
                self.run["collisions"] += 1
            col = self.ar.buoys[k][2]
            self.ev("collision", buoy=col, in_ftp=self.run is not None)
        self.touching = hit
        self.battery = max(0.0, self.battery - dt * (0.01 + 0.08 * abs(self.v) + 0.04 * abs(self.w)))
        if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.15:
            self.trail.append((self.x, self.y))
            self.trail = self.trail[-1500:]
        self.check_done()
        self.publish_pose()

    # ------------------------------------------------------------ aturan FTP
    def track_ftp(self, px, py):
        x, y = self.x, self.y
        if self.run is None:
            for d, xs in ((1, FTP_X0), (-1, FTP_X1)):
                crossed = (px < xs <= x) if d == 1 else (px > xs >= x)
                if crossed and FTP_Y0 <= y <= FTP_Y1 and self.running and not self.ftp_locked[d]:
                    self.attempts[d] += 1
                    self.run = dict(d=d, attempt=self.attempts[d], gates=0, collisions=0)
                    self.ev("ftp_start", direction="Spot1_ke_Spot2" if d == 1 else "Spot2_ke_Spot1", attempt=self.attempts[d])
            return
        run, d = self.run, self.run["d"]
        order = self.ar.gates if d == 1 else self.ar.gates[::-1]
        if not (FTP_Y0 <= y <= FTP_Y1):
            return self.ftp_end(False, "keluar_area")
        if run["gates"] < len(order):
            gx, gy, gw = order[run["gates"]]
            crossed = (px < gx <= x) if d == 1 else (px > gx >= x)
            if crossed:
                r = A["buoy_r"]
                if gy - gw / 2 + r <= y <= gy + gw / 2 - r:
                    run["gates"] += 1
                    self.ev("gate_pass", gate=run["gates"])
                else:
                    return self.ftp_end(False, "meleset_gerbang")
        xe = FTP_X1 if d == 1 else FTP_X0
        if (d == 1 and x >= xe) or (d == -1 and x <= xe):
            return self.ftp_end(run["gates"] == len(order), "selesai" if run["gates"] == len(order) else "gerbang_terlewat")
        xs = FTP_X0 if d == 1 else FTP_X1
        if (d == 1 and x < xs - 0.3) or (d == -1 and x > xs + 0.3):
            return self.ftp_end(False, "mundur_keluar")

    def ftp_end(self, ok, why):
        run, d = self.run, self.run["d"]
        score = max(0, 100 - 20 * run["collisions"]) if ok else 0
        rec = dict(direction=d, attempt=run["attempt"], collisions=run["collisions"], status="selesai" if ok else "gagal", reason=why, score=score)
        self.runs_log.append(rec)
        self.ev("ftp_end", **rec)
        self.pub_score.publish(String(data=json.dumps(rec)))
        self.run = None
        if ok:
            self.attempts[d] = 0
        elif self.attempts[d] >= MAX_ATTEMPT:
            self.ftp_locked[d] = True
            self.ev("ftp_locked", direction=d, note="3 kali gagal, FTP arah ini tidak bisa diulang")

    # ------------------------------------------------------------ selesai
    def check_done(self):
        if self.done or not self.running:
            return
        if all(s == "spot2" for s in self.item_state.values()) and self.docked(self.ar.slot2):
            t0 = self.last_place_t or 0
            buzz_ok = self.buzzer and self.buzzer_t is not None and self.buzzer_t >= t0 - 1.0 and self.buzzer_t - t0 < 3.0
            spray_ok = self.spray and self.spray_t is not None and self.spray_t >= t0 - 1.0 and self.spray_t - t0 < 3.0
            if buzz_ok and spray_ok:
                self.done, self.running = True, False
                ftp = [r for r in self.runs_log if r["status"] == "selesai"]
                self.summary = dict(waktu_s=round(self.now() - self.t_start, 1), tabrakan=self.collisions_total, ftp_selesai=len(ftp), ftp_skor=[r["score"] for r in ftp], barang=len(self.item_state))
                self.ev("mission_done", **self.summary)

    def phase(self):
        if self.estop:
            return "ESTOP"
        if self.done:
            return "DONE"
        if not self.running:
            return "IDLE"
        if self.run is not None:
            return "FTP_KE_SPOT2" if self.run["d"] == 1 else "FTP_KE_SPOT1"
        if inside(self.ar.slot1, self.x, self.y):
            return "DI_SLOT_SPOT1"
        if inside(self.ar.slot2, self.x, self.y):
            return "DI_SLOT_SPOT2"
        return "TRANSIT"

    # ------------------------------------------------------------ publish
    def publish_pose(self):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_q(self.yaw)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "map", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "map", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.pub_odom.publish(o)
        p = PoseStamped()
        p.header = o.header
        p.pose = o.pose.pose
        self.pub_pose.publish(p)

    def publish_slow(self):
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.percentage = self.battery / 100.0
        b.voltage = 14.0 + 2.8 * b.percentage
        self.pub_bat.publish(b)
        self.pub_state.publish(String(data=self.phase()))

    def mk(self, ns, i, typ, x, y, z, sx, sy, sz, c, a=1.0, frame="map"):
        m = Marker()
        m.header.frame_id, m.ns, m.id, m.type, m.action = frame, ns, i, typ, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = rgba(c, a)
        return m

    def publish_markers(self):
        ar, ms = self.ar, MarkerArray()
        vs = float(self.get_parameter("viz_scale").value)
        ms.markers.append(self.mk("air", 0, Marker.CUBE, ar.W / 2, ar.H / 2, -0.02, ar.W, ar.H, 0.02, COL["water"], 0.9))
        for i, b in enumerate(ar.blocks):
            ms.markers.append(self.mk("dermaga", i, Marker.CUBE, (b[0] + b[2]) / 2, (b[1] + b[3]) / 2, 0.1, b[2] - b[0], b[3] - b[1], 0.2, COL["wood"]))
        for i, s in enumerate((ar.slot1, ar.slot2)):
            ms.markers.append(self.mk("slot", i, Marker.CUBE, (s[0] + s[2]) / 2, (s[1] + s[3]) / 2, 0.005, s[2] - s[0], s[3] - s[1], 0.01, COL["blue"], 0.35))
        for i, (bx, by, col, r) in enumerate(ar.buoys):
            ms.markers.append(self.mk("buoy", i, Marker.SPHERE, bx, by, r * 0.35, 2 * r, 2 * r, 2 * r, COL[col]))
        for i, (s, x, y) in enumerate(ar.drops):
            ms.markers.append(self.mk("zona_taruh", i, Marker.CUBE, x, y, 0.21, 0.15 * vs * 1.3, 0.15 * vs * 1.3, 0.005, COL["white"], 0.5))
        for i, (s, x, y) in self.item_pos.items():
            if self.item_state[i] == "carried":
                k = self.carried.index(i)
                c, sn = math.cos(self.yaw), math.sin(self.yaw)
                u, v = -0.15 + 0.17 * k, 0.0
                x, y = self.x + u * c - v * sn, self.y + u * sn + v * c
                z = 0.28
            else:
                z = 0.2 + 0.075 * vs
            ms.markers.append(self.item_marker(i, s, x, y, z, 0.15 * vs))
        # kapal: dua lambung dan dek, ikut base_link
        for k, sgn in enumerate((1, -1)):
            ms.markers.append(self.mk("asv", k, Marker.CUBE, 0.0, sgn * self.B * 0.38, 0.1, self.L, self.B * 0.24, 0.2, COL["navy"], frame="base_link"))
        ms.markers.append(self.mk("asv", 2, Marker.CUBE, -0.05, 0.0, 0.22, self.L * 0.7, self.B * 0.6, 0.06, COL["blue"], frame="base_link"))
        ar_m = self.mk("asv", 3, Marker.ARROW, 0.0, 0.0, 0.3, 0.7, 0.08, 0.08, COL["red"], frame="base_link")
        ms.markers.append(ar_m)
        # jejak
        tr = Marker()
        tr.header.frame_id, tr.ns, tr.id, tr.type, tr.action = "map", "jejak", 0, Marker.LINE_STRIP, Marker.ADD
        tr.scale.x, tr.color, tr.pose.orientation.w = 0.06, rgba(COL["blue"], 0.8), 1.0
        tr.points = [Point(x=float(a), y=float(b), z=0.03) for a, b in self.trail[::3]]
        ms.markers.append(tr)
        # teks status
        carried = len(self.carried)
        placed = sum(1 for s in self.item_state.values() if s == "spot2")
        txt = f"{self.phase()} | bawa {carried} | di Spot 2: {placed}/{len(self.item_state)} | tabrakan {self.collisions_total} | baterai {self.battery:.0f}%"
        t = self.mk("status", 0, Marker.TEXT_VIEW_FACING, ar.W / 2, ar.H + 0.8, 0.5, 0.0, 0.0, 0.6, COL["white"])
        t.text = txt
        ms.markers.append(t)
        # tepi kolam
        e = Marker()
        e.header.frame_id, e.ns, e.id, e.type, e.action = "map", "tepi", 0, Marker.LINE_STRIP, Marker.ADD
        e.scale.x, e.color, e.pose.orientation.w = 0.1, rgba(COL["white"]), 1.0
        e.points = [Point(x=float(a), y=float(b), z=0.02) for a, b in ((0, 0), (ar.W, 0), (ar.W, ar.H), (0, ar.H), (0, 0))]
        ms.markers.append(e)
        self.pub_mark.publish(ms)

    def item_marker(self, i, shape, x, y, z, s):
        c = COL["yellow"]
        if shape == "cube":
            return self.mk("barang", i, Marker.CUBE, x, y, z, s, s, s, c)
        if shape == "sphere":
            return self.mk("barang", i, Marker.SPHERE, x, y, z, s, s, s, c)
        m = self.mk("barang", i, Marker.TRIANGLE_LIST, x, y, z - s / 2, 1, 1, 1, c)
        h = s / 2
        p = [(-h, -h, 0), (h, -h, 0), (0, -h, s), (-h, h, 0), (0, h, s), (h, h, 0),            # dua sisi segitiga
             (-h, -h, 0), (-h, h, 0), (h, h, 0), (-h, -h, 0), (h, h, 0), (h, -h, 0),            # alas
             (-h, -h, 0), (0, -h, s), (0, h, s), (-h, -h, 0), (0, h, s), (-h, h, 0),            # miring kiri
             (h, -h, 0), (h, h, 0), (0, h, s), (h, -h, 0), (0, h, s), (0, -h, s)]               # miring kanan
        m.points = [Point(x=float(a), y=float(b), z=float(cc)) for a, b, cc in p]
        return m


def main():
    rclpy.init()
    rclpy.spin(ArenaSim())


if __name__ == "__main__":
    main()
