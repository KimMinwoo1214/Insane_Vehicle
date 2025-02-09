import json

def convert_txt_to_json(txt_file, json_file):
    waypoints = []
    
    # Open the text file and read waypoint lines
    with open(txt_file, 'r') as file:
        print("start")
        for line in file:
            coords = list(map(float, line.split()))
            waypoints.append(coords)
    # Create a dictionary structure matching your format (without the removed fields)
    json_data = {
        "idx": 0,
        "points": waypoints,
        "max_speed": 20
    }
    print("complete")

    # Save the result to a JSON file
    with open(json_file, 'w') as file:
        json.dump(json_data, file, indent=4)

# Example usage
txt_file = '/home/mobilion/catkin_ws/src/aMAP/src/link/mando_path.txt'  # Replace with your text file path
json_file = '/home/mobilion/catkin_ws/src/aMAP/src/link/mando_path.txt'
convert_txt_to_json(txt_file, json_file)