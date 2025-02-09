import onnx
model = onnx.load('/home/mobilion/catkin_ws/src/aMAP/src/models/ped_1002.onnx')
onnx.checker.check_model(model)
print(onnx.helper.printable_graph(model.graph))
