from sklearn.metrics import silhouette_score, davies_bouldin_score, calinski_harabasz_score
import csv
import os
from datetime import datetime

def evaluate_clustering(cluster_points, labels, c_type="dbscan"):
    """
    클러스터링 평가 함수: 다양한 지표를 계산하여 출력합니다.

    Args:
        cluster_points (np.ndarray): 클러스터링된 데이터 포인트 (NxD).
        labels (np.ndarray): 클러스터 레이블.

    Returns:
        dict: 계산된 지표들을 포함한 딕셔너리.
    """
    # 결과 저장용 딕셔너리
    scores = {"c_type": c_type}

    # 1. Silhouette Score (값이 클수록 좋음)
    if len(set(labels)) > 1:  # 최소 2개의 클러스터가 있어야 계산 가능
        silhouette = silhouette_score(cluster_points, labels)
        scores['silhouette_score'] = silhouette
        print(f"Silhouette Score: {silhouette}")
    else:
        scores['silhouette_score'] = None
        print("Silhouette Score: Cannot be computed (less than 2 clusters)")

    # 2. Davies-Bouldin Index (값이 작을수록 좋음)
    try:
        db_index = davies_bouldin_score(cluster_points, labels)
        scores['davies_bouldin_index'] = db_index
        print(f"Davies-Bouldin Index: {db_index}")
    except ValueError as e:
        scores['davies_bouldin_index'] = None
        print(f"Davies-Bouldin Index: Cannot be computed ({e})")

    # 3. Calinski-Harabasz Index (값이 클수록 좋음)
    try:
        ch_index = calinski_harabasz_score(cluster_points, labels)
        scores['calinski_harabasz_index'] = ch_index
        print(f"Calinski-Harabasz Index: {ch_index}")
    except ValueError as e:
        scores['calinski_harabasz_index'] = None
        print(f"Calinski-Harabasz Index: Cannot be computed ({e})")

    # 4. Noise Ratio (DBSCAN에서 -1 라벨이 노이즈를 나타냄)
    noise_ratio = sum(labels == -1) / len(labels)
    scores['noise_ratio'] = noise_ratio
    print(f"Noise Ratio: {noise_ratio:.2f}")

    # 5. Number of Clusters (유효 클러스터의 개수)
    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    scores['num_clusters'] = num_clusters
    print(f"Number of Clusters: {num_clusters}")

    return scores


def record_evaluation_result(score, last_record_time, evaluation_results, saved):
    """평가 결과를 기록하고, 조건에 따라 CSV로 저장"""
    now = datetime.now()
    if (now - last_record_time).total_seconds() >= 5:
        evaluation_results.append(score)
        last_record_time = now

        if len(evaluation_results) >= 10 and not saved:
            save_evaluation_results(score["c_type"], evaluation_results, saved)
            savaed = True


def save_evaluation_results(c_type, evaluation_results, saved):
    """평가 결과를 CSV 파일로 저장"""
    file_name = f"{c_type}_eval.csv"
    file_path = os.path.join(os.getcwd(), file_name)

    # CSV 파일로 저장
    keys = evaluation_results[0].keys()
    with open(file_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(evaluation_results)

    print(f"Evaluation results saved to {file_path}")