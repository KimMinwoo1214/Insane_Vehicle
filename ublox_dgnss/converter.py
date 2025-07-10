import csv
import utm

# === 사용자 입력 ===
txt_file = "RDDF.txt"   # txt 파일명
csv_file = "lastdance.csv"  # 저장할 csv 파일명

rows = []

with open(txt_file, "r") as file:
    for line in file:
        line = line.strip()
        if line.startswith("data:"):
            parts = line.split(":")[1].strip().split(",")
            lat = float(parts[1])
            lon = float(parts[0])
            utm_result = utm.from_latlon(lat, lon)
            utm_x = utm_result[0]  # Easting
            utm_y = utm_result[1]  # Northing
            rows.append([lon, lat, utm_x, utm_y])

with open(csv_file, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Long", "Lat", "UTM_X(East)", "UTM_Y(North)"])
    writer.writerows(rows)

print(f"✅ 변환 완료: {csv_file}")
