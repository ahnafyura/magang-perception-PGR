#!/usr/bin/env python3
"""Membuat dataset YOLO berlabel otomatis dari world VRX (alat bantu pemateri).

Kamera bounding box Gazebo merusak tampilan air VRX, jadi dipakai DUA simulasi:
  partition "rgb"   : world asli, kamera RGB biasa          -> gambar
  partition "label" : world yang sama, kamera bounding box  -> label
Kedua kamera dipindah ke pose acak yang SAMA, lalu gambar dan label disimpan.
Buoy di world berlabuh (tidak hanyut), hanya naik turun beberapa cm karena ombak.

Langkah (VRX sudah di-source, tiga terminal):
    W=$(ros2 pkg prefix barunastra_perception)/share/barunastra_perception/worlds/perception_regatta.sdf
    GZ_PARTITION=rgb   gz sim -s -r $W
    GZ_PARTITION=label gz sim -s -r $W
    python3 tools/make_dataset_gt.py --out ~/dataset_vrx --n 800
"""
import argparse
import math
import os
import random
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node, NodeOptions

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_world import COURSE, START_YAW, to_world  # noqa: E402

WORLD = 'perception_regatta'
W, H = 1280, 720
SENSOR = """<sdf version="1.9"><model name="dataset_cam"><static>true</static><link name="link">
<sensor name="cam" type="{type}"><update_rate>5</update_rate><always_on>true</always_on>
<topic>{topic}</topic><camera>{box}<horizontal_fov>1.3962634</horizontal_fov>
<image><width>1280</width><height>720</height></image><clip><near>0.1</near><far>300</far></clip>
</camera></sensor></link></model></sdf>"""


class Side:
    """Satu simulasi (partition) dengan satu kamera."""

    def __init__(self, partition, sensor_type, topic, msg_type):
        opts = NodeOptions()
        opts.partition = partition
        self.node = Node(opts)
        self.partition = partition
        self.msg = None
        self.lock = threading.Lock()
        box = '<box_type>2d</box_type>' if sensor_type == 'boundingbox_camera' else ''
        req = EntityFactory()
        req.sdf = SENSOR.format(type=sensor_type, topic=topic, box=box)
        req.name = 'dataset_cam'
        ok, rep = self.node.request(f'/world/{WORLD}/create', req, EntityFactory, Boolean, 5000)
        if not (ok and rep.data):
            # biasanya karena kamera dari jalankan sebelumnya masih ada: dipakai ulang.
            # (menghapus kamera bounding box membuat Gazebo crash, jadi tidak dihapus)
            print(f'[{partition}] kamera sudah ada atau spawn gagal, dicek lewat set_pose', flush=True)
        self.node.subscribe(msg_type, topic, self.on_msg)

    def on_msg(self, msg):
        with self.lock:
            self.msg = msg

    def set_pose(self, x, y, z, roll, pitch, yaw):
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        q = (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
             cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
        req = (f'name: "dataset_cam", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
               f'orientation: {{w: {q[0]:.6f}, x: {q[1]:.6f}, y: {q[2]:.6f}, z: {q[3]:.6f}}}')
        # lewat perintah gz service: lebih tahan daripada request Python yang dipanggil ratusan kali
        env = dict(os.environ, GZ_PARTITION=self.partition)
        r = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', 'gz.msgs.Pose',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                           env=env, capture_output=True, text=True)
        return 'data: true' in r.stdout

    def take_new(self, timeout=5.0):
        """Buang pesan lama, tunggu pesan baru."""
        with self.lock:
            self.msg = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if self.msg is not None:
                    return self.msg
            time.sleep(0.02)
        return None


def random_pose():
    buoys = [to_world(x, y) for _, x, y in COURSE]
    while True:
        cx, cy = random.uniform(-6, 66), random.uniform(-13, 13)
        wx, wy = to_world(cx, cy)
        if all(math.hypot(wx - bx, wy - by) > 2.5 for bx, by in buoys):
            break
    if random.random() < 0.75:
        yaw = START_YAW + random.gauss(0.0, 0.5)       # kebanyakan menghadap ke lintasan
    else:
        yaw = random.uniform(-math.pi, math.pi)
    # kamera WAM-V: tinggi sekitar 1,3 m dari air, menunduk 0,262 rad
    return (wx, wy, random.uniform(1.15, 1.45), random.gauss(0.0, 0.03),
            0.262 + random.gauss(0.0, 0.04), yaw)


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--out', default=os.path.expanduser('~/dataset_vrx'))
    a.add_argument('--n', type=int, default=800)
    a.add_argument('--seed', type=int, default=1)
    a.add_argument('--min-box', type=float, default=4.0)
    args = a.parse_args()
    random.seed(args.seed)
    os.makedirs(os.path.join(args.out, 'images'), exist_ok=True)
    os.makedirs(os.path.join(args.out, 'labels'), exist_ok=True)

    rgb = Side('rgb', 'camera', '/dataset_cam/image', Image)
    lab = Side('label', 'boundingbox_camera', '/dataset_cam/bbox', AnnotatedAxisAligned2DBox_V)
    time.sleep(2.0)
    if not (rgb.set_pose(*random_pose()) and lab.set_pose(*random_pose())):
        sys.exit('kamera tidak ditemukan. Pastikan dua simulasi (partition rgb dan label) sudah jalan.')

    saved, gagal = 0, 0
    start = len(os.listdir(os.path.join(args.out, 'images')))
    while saved < args.n:
        if gagal >= 20:
            sys.exit('20 kali berturut-turut gagal, cek kedua simulasi')
        pose = random_pose()
        if not (rgb.set_pose(*pose) and lab.set_pose(*pose)):
            gagal += 1
            print('set_pose gagal', flush=True)
            continue
        time.sleep(0.5)
        img, boxes = rgb.take_new(), lab.take_new()
        if img is None or boxes is None:
            gagal += 1
            print('gambar atau label tidak datang:', 'gambar' if img is None else '', 'label' if boxes is None else '', flush=True)
            continue
        gagal = 0
        frame = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
        lines = []
        for b in boxes.annotated_box:
            x0, y0 = max(0.0, b.box.min_corner.x), max(0.0, b.box.min_corner.y)
            x1, y1 = min(W - 1.0, b.box.max_corner.x), min(H - 1.0, b.box.max_corner.y)
            w, h = x1 - x0, y1 - y0
            if not 1 <= b.label <= 4 or w < args.min_box or h < args.min_box:
                continue
            lines.append(f'{b.label - 1} {(x0 + w / 2) / W:.6f} {(y0 + h / 2) / H:.6f} {w / W:.6f} {h / H:.6f}')
        name = f'vrx_{start + saved:06d}'
        cv2.imwrite(os.path.join(args.out, 'images', name + '.jpg'), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        with open(os.path.join(args.out, 'labels', name + '.txt'), 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        saved += 1
        if saved % 50 == 0:
            print(f'{saved}/{args.n} gambar', flush=True)
    print('selesai', saved, 'gambar di', args.out)


if __name__ == '__main__':
    main()
