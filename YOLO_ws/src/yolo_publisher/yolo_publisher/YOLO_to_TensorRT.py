from ultralytics import YOLO

# 모델 로드
model = YOLO('/home/antel/2025IEVE_1of5/2025IEVE/YOLO_ws/weights/YOLO_0216.pt')

# TensorRT 엔진으로 내보내기
model.export(format='engine', device=0)
