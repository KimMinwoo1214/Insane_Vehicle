import torch
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import torchvision.transforms as transforms
from rdp import rdp  # 점 개수 줄이기
import importlib
import argparse

# **설정 값 직접 입력 (yaml 없이 사용)**
cfg = {
    "dataset": "CULane",  
    "train_width": 800,  
    "train_height": 320,  
    "backbone": "18",  
    "row_anchor": [i for i in range(160, 320, 10)],  
    "col_anchor": [i for i in range(100, 700, 50)],  
    "test_model": "checkpoints/my_trained_model.pth",  
}

def get_model(cfg):
    """ 모델 로드 함수 """
    model_module = importlib.import_module(f"model.model_{cfg['dataset'].lower()}")
    return model_module.get_model(cfg)

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        
        self.lane_publisher = self.create_publisher(String, "/lane_points", 10)
        
        self.get_logger().info("Loading model...")
        self.net = get_model(cfg).cuda()  # GPU 사용
        state_dict = torch.load(cfg["test_model"], map_location="cuda")["model"]
        self.net.load_state_dict(state_dict, strict=False)
        self.net.eval()
        self.get_logger().info("Model loaded successfully.")

        self.img_transforms = transforms.Compose([
            transforms.Resize((cfg["train_height"], cfg["train_width"])),
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            exit()

    def pred2coords(self, pred):
        """ 모델 출력 → 차선 좌표 변환 """
        batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
        max_indices_row = pred['loc_row'].argmax(1).cpu()
        valid_row = pred['exist_row'].argmax(1).cpu()

        coords = []
        row_lane_idx = [1, 2]  

        for i in row_lane_idx:
            tmp = []
            if valid_row[0, :, i].sum() > num_cls_row / 2:
                for k in range(valid_row.shape[1]):
                    if valid_row[0, k, i]:
                        out_tmp = (pred['loc_row'][0, :, k, i].softmax(0) * torch.arange(len(cfg["row_anchor"])).float()).sum().item()
                        tmp.append((int(out_tmp / len(cfg["row_anchor"]) * cfg["train_width"]), int(cfg["row_anchor"][k] * cfg["train_height"])))
                
                # 점 개수 줄이기 (RDP 적용)
                tmp = rdp(tmp, epsilon=5)  
                coords.append(tmp)

        return coords

    def run(self):
        """ 실시간 차선 탐지 및 ROS2 퍼블리싱 """
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: Could not read frame.")
                break

            img = self.img_transforms(frame).unsqueeze(0).cuda()  

            with torch.no_grad():
                pred = self.net(img)

            coords = self.pred2coords(pred)

            # 차선 그리기
            for lane in coords:
                if len(lane) > 1:
                    cv2.polylines(frame, [np.array(lane, np.int32)], isClosed=False, color=(0, 255, 0), thickness=2)

            # ROS2 퍼블리싱
            lane_msg = String()
            lane_msg.data = " ".join([f"{x},{y}" for lane in coords for (x, y) in lane])
            self.lane_publisher.publish(lane_msg)
            self.get_logger().info(f"Published Lane Points: {lane_msg.data}")

            cv2.imshow("Lane Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    detector = LaneDetector()
    detector.run()
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
