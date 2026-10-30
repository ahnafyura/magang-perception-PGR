#!/usr/bin/env python3
"""Membagi dataset berlabel (folder images/ dan labels/, misalnya hasil tools/make_dataset_gt.py)
menjadi train/valid dan membuat data.yaml untuk YOLO. Ekspor Roboflow sudah terbagi, tidak perlu ini.

    python3 tools/split_dataset.py ~/dataset_vrx ~/dataset_vrx_yolo --valid 0.15

Pembagian berurutan per blok (bukan acak per gambar), karena gambar yang diambil
berdekatan waktunya hampir sama. Kalau diacak, gambar kembar bisa masuk train dan
valid sekaligus sehingga nilai validasi terlihat bagus padahal tidak jujur.
"""
import argparse
import os
import shutil

NAMES = ['red', 'green', 'orange', 'black']   # urutan = class id 0..3


def main():
    p = argparse.ArgumentParser()
    p.add_argument('src')
    p.add_argument('dst')
    p.add_argument('--valid', type=float, default=0.15)
    p.add_argument('--block', type=int, default=20, help='ukuran blok gambar berurutan')
    a = p.parse_args()

    images = sorted(f for f in os.listdir(os.path.join(a.src, 'images')) if f.endswith('.jpg'))
    every = max(2, round(1 / a.valid))      # 1 dari tiap `every` blok masuk valid
    count = {'train': 0, 'valid': 0}
    for i, name in enumerate(images):
        split = 'valid' if (i // a.block) % every == every - 1 else 'train'
        label = os.path.splitext(name)[0] + '.txt'
        for sub in ('images', 'labels'):
            os.makedirs(os.path.join(a.dst, split, sub), exist_ok=True)
        shutil.copy(os.path.join(a.src, 'images', name), os.path.join(a.dst, split, 'images', name))
        src_label = os.path.join(a.src, 'labels', label)
        if os.path.exists(src_label):
            shutil.copy(src_label, os.path.join(a.dst, split, 'labels', label))
        count[split] += 1

    with open(os.path.join(a.dst, 'data.yaml'), 'w') as f:
        f.write(f'path: {os.path.abspath(a.dst)}\ntrain: train/images\nval: valid/images\n\n'
                f'nc: {len(NAMES)}\nnames: {NAMES}\n')
    print(count, '->', os.path.join(a.dst, 'data.yaml'))


if __name__ == '__main__':
    main()
