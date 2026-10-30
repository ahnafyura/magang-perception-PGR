#!/usr/bin/env python3
"""Membuat worlds/perception_regatta.sdf dari world sydney_regatta milik VRX.

Isi world VRX tidak diubah (danau, ombak, pohon, cahaya). Yang diubah hanya:
  1. 6 buoy bawaan yang bergerombol dihapus
  2. lintasan buoy ditambahkan di depan posisi awal WAM-V
  3. tiap buoy diberi plugin Label (dipakai kamera bounding box untuk label dataset)

Butuh VRX sudah ter-source. Jalankan ulang kalau tata letak diubah:
    python3 tools/make_world.py
"""
import math
import os
import re

from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'worlds', 'perception_regatta.sdf')
SRC = os.path.join(get_package_share_directory('vrx_gz'), 'worlds', 'sydney_regatta.sdf')

# posisi dan arah awal WAM-V di VRX (spawn bawaan)
START_X, START_Y, START_YAW = -532.129, 161.982, 1.0

# kelas: (model Fuel VRX, label Gazebo). Kelas YOLO = label - 1
BUOY_TYPES = {
    'red': ('mb_marker_buoy_red', 1),
    'green': ('mb_marker_buoy_green', 2),
    'orange': ('mb_round_buoy_orange', 3),
    'black': ('mb_round_buoy_black', 4),
}

# lintasan dalam koordinat kapal: x = maju dari posisi awal, y = ke kiri (meter)
COURSE = [
    ('red', 15, 4.0), ('green', 15, -4.0),
    ('red', 30, 5.0), ('green', 30, -3.0),
    ('red', 45, 6.0), ('green', 45, -2.0),
    ('red', 60, 4.0), ('green', 60, -4.0),
    ('orange', 22, 0.5), ('black', 37, 1.5),
    ('orange', 52, 3.5), ('black', 52, -1.0),
    ('black', 8, -9.0), ('orange', 26, 11.0), ('black', 40, -12.0), ('orange', 66, 10.0),
]

# blok plugin apung, sama persis dengan yang dipakai VRX untuk model ini
BUOYANCY = {
    'marker': ('25.0', '<pose>0 0 -0.3 0 0 0</pose>',
               '<cylinder><radius>0.325</radius><length>0.1</length></cylinder>'),
    'round': ('75.0', '<pose>0 0 0 0 0 0</pose>', '<sphere><radius>0.21</radius></sphere>'),
}


def to_world(x, y):
    c, s = math.cos(START_YAW), math.sin(START_YAW)
    return START_X + c * x - s * y, START_Y + s * x + c * y


def buoy_block(name, color, x, y):
    model, label = BUOY_TYPES[color]
    drag, pose, geom = BUOYANCY['marker' if 'marker' in model else 'round']
    wx, wy = to_world(x, y)
    return f"""
    <include>
      <name>{name}</name>
      <pose>{wx:.3f} {wy:.3f} 0 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/{model}</uri>
      <plugin name="vrx::PolyhedraBuoyancyDrag" filename="libPolyhedraBuoyancyDrag.so">
        <fluid_density>1000</fluid_density>
        <fluid_level>0.0</fluid_level>
        <linear_drag>{drag}</linear_drag>
        <angular_drag>2.0</angular_drag>
        <buoyancy name="collision_outer">
          <link_name>link</link_name>
          {pose}
          <geometry>{geom}</geometry>
        </buoyancy>
        <wavefield><topic>/vrx/wavefield/parameters</topic></wavefield>
      </plugin>
      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
        <label>{label}</label>
      </plugin>
    </include>"""


def main():
    sdf = open(SRC).read()
    # hapus buoy bawaan (blok <include> yang memuat model mb_...)
    blocks = re.findall(r'\n\s*<include>(?:(?!</include>).)*?/models/mb_[a-z_]+</uri>.*?</include>', sdf, re.S)
    assert len(blocks) == 6, len(blocks)
    for b in blocks:
        sdf = sdf.replace(b, '')
    sdf = sdf.replace('<world name="sydney_regatta">', '<world name="perception_regatta">', 1)

    parts, count = [], {}
    for color, x, y in COURSE:
        count[color] = count.get(color, 0) + 1
        parts.append(buoy_block(f'buoy_{color}_{count[color]}', color, x, y))
    head = ('<!-- DIBUAT OTOMATIS oleh tools/make_world.py dari vrx_gz/worlds/sydney_regatta.sdf.\n'
            '     Jangan diedit manual. -->\n')
    sdf = sdf.replace('</world>', '\n    <!-- lintasan buoy Magang PGR -->' + ''.join(parts) + '\n  </world>', 1)
    sdf = re.sub(r'^(<\?xml[^>]*\?>\n)', r'\1' + head, sdf)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, 'w').write(sdf)
    print('OK', os.path.normpath(OUT), len(COURSE), 'buoy')


if __name__ == '__main__':
    main()
