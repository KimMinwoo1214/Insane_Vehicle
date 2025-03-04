import os
import json
import numpy as np
import pandas as pd

results_dir = "/home/f1tenth/kjy_ws/src/CalibJ/CalibJ/results"
csv_name = "reprojection_errors_"
json_name = "calibration_extrinsic_"

def calculate_average_reprojection_error(results_dir, csv_name, json_name):
    """
    Calculate average reprojection error per file, overall average, and deviation of extrinsic matrices.

    Args:
        results_dir (str): Directory containing CSV and JSON files.
        csv_name (str): Base name of reprojection error CSV files.
        json_name (str): Base name of calibration extrinsic JSON files.

    Returns:
        None
    """
    # Find all CSV files
    csv_files = sorted([f for f in os.listdir(results_dir) if f.startswith(csv_name) and f.endswith('.csv')])

    # Store per-file averages and accumulate all reprojection errors
    file_averages = {}
    all_errors = []

    for csv_file in csv_files:
        file_path = os.path.join(results_dir, csv_file)
        data = pd.read_csv(file_path)

        # Calculate average for the current file
        file_avg = data['Reprojection Error'].mean()
        file_averages[csv_file] = file_avg

        # Collect all errors
        all_errors.extend(data['Reprojection Error'].values)

    # Calculate overall average
    overall_average = np.mean(all_errors)

    # Find all JSON files
    json_files = sorted([f for f in os.listdir(results_dir) if f.startswith(json_name) and f.endswith('.json')])

    # Load rvec and tvec from all JSON files
    rvecs = []
    tvecs = []

    for json_file in json_files:
        file_path = os.path.join(results_dir, json_file)
        with open(file_path, 'r') as f:
            extrinsic_data = json.load(f)
            rvecs.append(np.array(extrinsic_data['rvec']))
            tvecs.append(np.array(extrinsic_data['tvec']))

    # Calculate deviation and percentage deviation for rvec and tvec
    rvecs = np.array(rvecs)
    tvecs = np.array(tvecs)

    rvec_std = np.std(rvecs, axis=0)
    tvec_std = np.std(tvecs, axis=0)


    # Create result dictionary
    result = {
        "file_averages": file_averages,
        "overall_average_error": overall_average,
        "rvec_deviation": rvec_std.tolist(),
        "tvec_deviation": tvec_std.tolist(),
    }

    # Save to JSON
    output_path = os.path.join(results_dir, "calibration_evaluation_results.json")
    with open(output_path, 'w') as f:
        json.dump(result, f, indent=4)

    print(f"Calibration evaluation results saved to {output_path}")

# Run the function
calculate_average_reprojection_error(results_dir, csv_name, json_name)
