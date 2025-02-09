import torch
from ultralytics import YOLO  # YOLOv8을 위한 ultralytics 패키지

# YOLOv8 모델 불러오기 (여기서 ped_1002.pt가 YOLOv8 모델과 호환되는 파일이어야 함)
model = YOLO('/home/mobilion/catkin_ws/src/aMAP/src/models/ped_1002.pt')
model.export(format="onnx", opset=11)

#model.eval()

# # 더미 입력 생성 (모델 입력에 맞는 크기)
# dummy_input = torch.randn(1, 3, 640, 640)

# # ONNX로 변환
# # Export the model
# torch.onnx.export(model,         # model being run
#         dummy_input,       # model input (or a tuple for multiple inputs)
#         "/home/mobilion/catkin_ws/src/aMAP/src/models/ped_1002.onnx",       # where to save the model
#         export_params=True,  # store the trained parameter weights inside the model file
#         opset_version=12,    # the ONNX version to export the model to
#         do_constant_folding=True,  # whether to execute constant folding for optimization
#         input_names = ['modelInput'],   # the model's input names
#         output_names = ['modelOutput'], # the model's output names
#         dynamic_axes={'modelInput' : {0 : 'batch_size'},    # variable length axes
#                             'modelOutput' : {0 : 'batch_size'}})
# print(" ")
# print('Model has been converted to ONNX')