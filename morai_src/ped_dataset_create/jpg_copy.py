import os
import shutil

# 원본 폴더와 복사할 대상 폴더의 경로를 설정합니다.
source_folder = '/home/mobilion/Downloads/MoraiLauncher_Lin_mando/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/SensorData/CAMERA_3'  # 원본 폴더 경로
destination_folder = '/home/mobilion/Desktop/dataset/yoloformat/pp1'  # 대상 폴더 경로

# 대상 폴더가 존재하지 않으면 생성합니다.
os.makedirs(destination_folder, exist_ok=True)

# 원본 폴더에서 파일을 검색하고, .png 파일만 복사합니다.
for filename in os.listdir(source_folder):
    # 파일이 .png로 끝나는지 확인합니다.
    if filename.startswith('24-10-02') and filename.endswith('.png'):
        source_path = os.path.join(source_folder, filename)
        destination_path = os.path.join(destination_folder, filename)
        
        # 파일을 복사합니다.
        shutil.copy2(source_path, destination_path)
        print(f'Copied: {filename} to {destination_folder}')

print('복사가 완료되었습니다.')
