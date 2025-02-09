import os
import shutil

# 기존 사진들이 담긴 폴더 경로
source_folder = os.path.expanduser("/home/mobilion/Downloads/MoraiLauncher_Lin_mando/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/SensorData/LIDAR_4")  # 본인의 경로로 수정하세요

# 조건에 맞는 사진을 저장할 새 폴더 경로
destination_folder = os.path.expanduser("/home/mobilion/Desktop/Chanwoo/annotation")  # 새 폴더 경로로 수정 가능

# 새 폴더가 없다면 생성
if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)

# 파일을 분류하여 새 폴더로 복사
for txt_file in os.listdir(source_folder):
    if txt_file.endswith('Instance.txt') and (txt_file.startswith("24-09-28")): # '24-09-24'로 시작하는 파일만 선택
        source_path = os.path.join(source_folder, txt_file)
        destination_path = os.path.join(destination_folder, txt_file)
        
        # 파일 복사
        shutil.copy2(source_path, destination_path)
        print(f"{txt_file} 파일이 {destination_folder}에 저장되었습니다.")

print("분류 작업이 완료되었습니다.")
