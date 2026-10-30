#!/usr/bin/env python3
"""Melatih YOLOv8n untuk deteksi buoy.

    python3 tools/train_yolo.py ~/dataset_buoy_yolo/data.yaml --epochs 50

Tanpa GPU: tambahkan --device cpu (jauh lebih lambat, kurangi --epochs).
Bisa juga dijalankan di Google Colab (GPU gratis):
    !pip install ultralytics
    !yolo detect train data=data.yaml model=yolov8n.pt epochs=50 imgsz=640
Hasil terbaik: runs/detect/<name>/weights/best.pt
"""
import argparse

from ultralytics import YOLO


def main():
    p = argparse.ArgumentParser()
    p.add_argument('data')
    p.add_argument('--epochs', type=int, default=50)
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--batch', type=int, default=16)
    p.add_argument('--device', default=None, help="'cpu' atau '0' untuk GPU pertama")
    p.add_argument('--name', default='buoy')
    a = p.parse_args()

    # mulai dari bobot yang sudah dilatih di dataset COCO (transfer learning)
    model = YOLO('yolov8n.pt')
    model.train(data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch,
                device=a.device, name=a.name)
    metrics = model.val()
    print('mAP50   :', round(metrics.box.map50, 3))
    print('mAP50-95:', round(metrics.box.map, 3))


if __name__ == '__main__':
    main()
