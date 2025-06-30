import os
import json
from collections import defaultdict

# 경로 설정
json_path = "your_dataset/annotations/labels.json"
img_root = "your_dataset/images"
out_label_dir = "your_dataset/culane_labels"
os.makedirs(out_label_dir, exist_ok=True)

with open(json_path, 'r') as f:
    coco = json.load(f)

# 이미지 ID -> 파일명 매핑
id_to_filename = {img['id']: img['file_name'] for img in coco['images']}

# 차선 정보 저장용
lane_dict = defaultdict(lambda: [[] for _ in range(4)])

for ann in coco['annotations']:
    image_id = ann['image_id']
    label_id = ann['category_id']  # class index 0~3
    if label_id >= 4:
        continue
    points = list(zip(ann['segmentation'][0][::2], ann['segmentation'][0][1::2]))
    lane_dict[image_id][label_id].extend(points)

# 각 이미지에 대해 .lines.txt 생성
for image_id, lanes in lane_dict.items():
    filename = id_to_filename[image_id]
    txt_path = os.path.join(out_label_dir, filename.replace(".jpg", ".lines.txt"))
    with open(txt_path, 'w') as f:
        for lane in lanes:
            if lane:
                sorted_lane = sorted(lane, key=lambda p: p[1])  # y 기준 정렬
                for x, y in sorted_lane:
                    f.write(f"{int(x)} {int(y)}\n")
            else:
                f.write("-100\n")
