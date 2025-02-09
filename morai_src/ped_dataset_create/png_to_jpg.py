from PIL import Image
import os

# PNG 이미지가 있는 폴더 경로 설정
source_folder = os.path.expanduser("/home/mobilion/Desktop/ped_data")  # 본인의 경로로 수정하세요
destination_folder = os.path.expanduser("/home/mobilion/Desktop/ped_data_jpg")  # 변환된 JPG 이미지를 저장할 폴더

# 저장할 폴더가 없다면 생성
if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)

# PNG 파일들을 하나씩 변환
for filename in os.listdir(source_folder):
    if filename.endswith(".png"):  # PNG 파일만 선택
        png_path = os.path.join(source_folder, filename)
        jpg_path = os.path.join(destination_folder, os.path.splitext(filename)[0] + ".jpg")  # 확장자를 JPG로 변경

        # PNG 이미지를 열고 JPG로 변환하여 저장
        with Image.open(png_path) as img:
            rgb_img = img.convert("RGB")  # PNG의 투명도를 제거하고 RGB 모드로 변환
            rgb_img.save(jpg_path, "JPEG")  # JPG 형식으로 저장

        print(f"{filename}이(가) JPG로 변환되어 {destination_folder}에 저장되었습니다.")

print("PNG에서 JPG로의 변환이 완료되었습니다.")
