import os

# 분류된 텍스트 파일들이 있는 폴더 경로
txt_folder = os.path.expanduser("/home/mobilion/Desktop/ped_annotation")  # 분류된 txt 파일들이 있는 폴더

# txt 파일들을 하나씩 처리
for txt_file in os.listdir(txt_folder):
    if txt_file.endswith(".txt"):
        txt_path = os.path.join(txt_folder, txt_file)

        # 조건에 맞는 줄만 남기기
        with open(txt_path, 'r') as file:
            lines = file.readlines()

        # 'Pedestrian' 또는 'Obstacle'로 시작하는 줄만 필터링
        filtered_lines = [line for line in lines if line.startswith("Pedestrian") or line.startswith("Obstacle")]

        # 필터링된 줄을 다시 파일에 저장
        with open(txt_path, 'w') as file:
            file.writelines(filtered_lines)

        print(f"{txt_file} 파일이 필터링되었습니다.")

print("모든 파일의 필터링 작업이 완료되었습니다.")
