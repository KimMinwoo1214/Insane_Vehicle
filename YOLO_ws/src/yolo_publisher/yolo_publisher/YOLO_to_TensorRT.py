from ultralytics import YOLO

# 모델 로드
model = YOLO('/home/parkm04/PycharmProjects/Insane_Vehicle/lane_ws/train9/weights/best.pt')

# TensorRT 엔진으로 내보내기
model.export(format='engine', device=0, half= True)
