import os

# 대상 폴더 경로 설정
folder_path = "/home/mobilion/Desktop/dataset/yoloformat/pp1"  # 작업할 폴더 경로를 여기에 입력하세요.

# 폴더 내의 모든 .txt 파일을 검색하고 "_BBox2D"를 제거
for filename in os.listdir(folder_path):
    if filename.endswith(".txt") and "_BBox2D" in filename:
        # 기존 파일 경로
        old_path = os.path.join(folder_path, filename)
        
        # 새로운 파일명에서 "_BBox2D" 제거
        new_filename = filename.replace("_BBox2D", "")
        new_path = os.path.join(folder_path, new_filename)
        
        # 파일 이름 변경
        os.rename(old_path, new_path)
        print(f"Renamed: {old_path} -> {new_path}")

# 폴더 내의 모든 .png 파일을 검색하고 "_Intensity"를 제거
for filename in os.listdir(folder_path):
    if filename.endswith(".png") and "_Intensity" in filename:
        # 기존 파일 경로
        old_path = os.path.join(folder_path, filename)
        
        # 새로운 파일명에서 "_Intensity" 제거
        new_filename = filename.replace("_Intensity", "")
        new_path = os.path.join(folder_path, new_filename)
        
        # 파일 이름 변경
        os.rename(old_path, new_path)
        print(f"Renamed: {old_path} -> {new_path}")

print("All matching files have been renamed successfully.")
