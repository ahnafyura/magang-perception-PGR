"""Cek cepat fungsi-fungsi TODO tanpa perlu menjalankan simulasi.

    cd ~/perception_ws/src/barunastra_perception
    python3 -m pytest test -v
    python3 -m pytest test -k todo5       # hanya test untuk TODO 5
"""
import math

import numpy as np
import pytest

from barunastra_perception.buoy_ranger import in_box, pick_group, project
from barunastra_perception.cloud_obstacles import cluster, filter_points, sector_min
from barunastra_perception.hsv_detector import HSV_RANGES


# ---------- Hari 1: HSV ----------

def hsv_hit(color, h, s, v):
    return any(all(lo[k] <= x <= hi[k] for k, x in enumerate((h, s, v)))
               for lo, hi in HSV_RANGES[color])


def test_todo1_red_dua_ujung_hue():
    assert hsv_hit('red', 2, 220, 150)      # merah dekat hue 0
    assert hsv_hit('red', 176, 220, 150)    # merah dekat hue 179
    assert not hsv_hit('red', 60, 220, 150)


def test_todo1_green():
    assert hsv_hit('green', 79, 165, 90)       # hijau buoy VRX
    assert not hsv_hit('green', 50, 150, 80)   # hijau pohon
    assert not hsv_hit('green', 2, 220, 150)


def test_todo1_air_dan_langit_tidak_masuk():
    for h, s, v in ((105, 120, 140), (110, 60, 200), (100, 20, 230)):   # air biru, langit pucat
        assert not hsv_hit('red', h, s, v)
        assert not hsv_hit('green', h, s, v)


# ---------- Hari 2: LiDAR ----------

PTS = np.array([
    [10.0, 0.0, -1.0],      # buoy di depan
    [10.0, 0.0, -2.5],      # di bawah permukaan air
    [10.0, 0.0, 9.0],       # pucuk pohon
    [1.0, 0.0, -1.0],       # badan kapal sendiri
    [80.0, 0.0, -1.0],      # terlalu jauh
    [np.nan, 0.0, -1.0],    # rusak
    [0.0, 6.0, -1.0],       # tepat di kiri
    [5.0, -5.0, -1.2],      # kanan depan (-45 derajat)
])


def test_todo5_filter_points():
    out = filter_points(PTS, -1.6, 1.0, 4.0, 60.0)
    assert out.shape == (3, 3)
    assert {tuple(p) for p in out} == {(10.0, 0.0, -1.0), (0.0, 6.0, -1.0), (5.0, -5.0, -1.2)}


def test_todo6_sector_min():
    pts = filter_points(PTS, -1.6, 1.0, 4.0, 60.0)
    assert sector_min(pts, -15, 15) == pytest.approx(10.0)
    assert sector_min(pts, 15, 90) == pytest.approx(6.0)
    assert sector_min(pts, -90, -15) == pytest.approx(math.hypot(5, 5))
    assert math.isinf(sector_min(pts, 100, 170))


def test_cluster_memisah_dua_objek():
    pts = np.array([[10, 0, 0], [10, 0.3, 0], [10.2, 0.1, 0], [10, 5, 0], [10.4, 5.2, 0]], float)
    groups = sorted(len(g) for g in cluster(pts, 1.0))
    assert groups == [2, 3]


# ---------- Hari 2: fusion ----------

FX, FY, CX, CY = 762.7, 762.7, 640.0, 360.0


def test_todo7_project():
    pts = np.array([[10.0, 0.0, 0.0],       # lurus di depan -> pusat gambar
                    [10.0, 10.0, 0.0],      # 45 derajat ke kiri -> u = cx - fx
                    [10.0, 0.0, -1.0],      # lebih rendah -> v lebih besar (ke bawah)
                    [-5.0, 0.0, 0.0]])      # di belakang kamera -> nan
    u, v = project(pts, FX, FY, CX, CY)
    assert (u[0], v[0]) == pytest.approx((CX, CY))
    assert u[1] == pytest.approx(CX - FX)
    assert v[2] == pytest.approx(CY + FY * 0.1)
    assert math.isnan(u[3]) and math.isnan(v[3])


def test_todo8_in_box():
    u = np.array([100.0, 130.0, 160.0, np.nan])
    v = np.array([50.0, 50.0, 50.0, 50.0])
    m = in_box(u, v, 120.0, 50.0, 50.0, 20.0)       # kotak u 95..145, v 40..60
    assert m.tolist() == [True, True, False, False]


def test_pick_group_oklusi():
    depths = np.array([12.1, 12.3, 30.2, 30.5, 30.4])   # buoy dekat menutupi buoy jauh
    assert sorted(pick_group(depths).tolist()) == [0, 1]
    assert sorted(pick_group(depths, expected=29.0).tolist()) == [2, 3, 4]
