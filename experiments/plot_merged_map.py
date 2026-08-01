#!/usr/bin/env python3

import sys
import json
from pathlib import Path
import yaml
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image

if len(sys.argv) != 2:
        print(f"Uso: python {sys.argv[0]} <diretorio_dos_mapas>")
        sys.exit(1)
    
RESULTS_DIR = Path(sys.argv[1])

if not RESULTS_DIR.exists():
    print(f"Diretório não encontrado: {RESULTS_DIR}")
    sys.exit(1)

def load_map(yaml_file):
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    pgm_path = yaml_file.parent / data["image"]

    image = np.array(Image.open(pgm_path))
    resolution = data["resolution"]
    origin = data["origin"]

    return image, resolution, origin

def load_trajectories(gt_file, odom_file, slam_file):
    gt_data = pd.read_csv(gt_file)
    odom_data = pd.read_csv(odom_file)
    slam_data = pd.read_csv(slam_file)

    return gt_data, odom_data, slam_data

def load_alignment_json(alignment_file):
    with open(alignment_file, "r") as f:
        alignment_data = json.load(f)
    return alignment_data

def compute_extent(image, resolution, origin):
    height, width = image.shape

    xmin = origin[0]
    xmax = origin[0] + width * resolution
    ymin = origin[1]
    ymax = origin[1] + height * resolution

    return [xmin, xmax, ymin, ymax]

def transfmatrix2pose(T):
    tx = T[0][2]
    ty = T[1][2]
    theta = math.atan2(T[1][0], T[0][0])
    return tx, ty, theta

def pose2transfmatrix(tx, ty, teta):
    c = math.cos(teta)
    s = math.sin(teta)
    return [
        [c, -s, tx],
        [s, c, ty],
        [0, 0, 1],
    ]

def inverse_transform(T):
    c = T[0][0]
    s = T[1][0]
    tx = T[0][2]
    ty = T[1][2]
    return [
        [c, s, -(c*tx+s*ty)],
        [-s, c, -(-s*tx+c*ty)],
        [0, 0, 1],
    ]

def matrix_multiplication(A,B):
    result = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                result[i][j] += A[i][k]*B[k][j]

    return result

def apply_transformation(trajectory, T):
    transformed = []
    for x, y in zip(trajectory["x"], trajectory["y"]):
        p = np.array([x, y, 1.0])
        p_new = T @ p
        transformed.append((p_new[0], p_new[1]))
    return pd.DataFrame(transformed, columns=['x', 'y'])

def compute_trajectory_transformation(robot_name, robot_reference, gt_data, odom_data, slam_data, alignment_data):
    current = alignment_data[robot_reference][robot_reference]
    relative = alignment_data[robot_reference][robot_name]
    other = alignment_data[robot_name][robot_name]
    Tc = pose2transfmatrix(current["translation"]["x"], current["translation"]["y"], current["rotation"])
    Tr = pose2transfmatrix(relative["translation"]["x"],relative["translation"]["y"],relative["rotation"])
    To = pose2transfmatrix(other["translation"]["x"],other["translation"]["y"],other["rotation"])
    T = matrix_multiplication(matrix_multiplication(Tc, Tr),inverse_transform(To))

    gt_data = apply_transformation(gt_data, T)
    odom_data = apply_transformation(odom_data, T)
    slam_data = apply_transformation(slam_data, T)

    return gt_data, odom_data, slam_data 

def plot_map_with_trajectories(map_yaml_file, gt_traject_files, odom_traject_files, slam_traject_files, alignment_data_file):
    image, resolution, origin = load_map(map_yaml_file)
    extent = compute_extent(image, resolution, origin)
    map = np.flipud(image)
    plt.figure(figsize=(8, 8))
    plt.imshow(map, cmap="gray", origin="lower", extent=extent,) # merged map

    robot_reference = gt_traject_files[0].stem.replace("_gt", "")
    align_data = load_alignment_json(alignment_data_file)
    for gt_file, odom_file, slam_file in zip(gt_traject_files, odom_traject_files, slam_traject_files):
            robot_name = gt_file.stem.replace("_gt", "")
            print(f"Processando trajetória {robot_name}...")

            gt, odom, slam = load_trajectories(gt_file, odom_file, slam_file)
            if robot_name != robot_reference:
                print(f"Transformando trajetória de {robot_name} para referência de {robot_reference}")
                gt, odom, slam = compute_trajectory_transformation(
                    robot_name, robot_reference,
                    gt, odom, slam, align_data,
                )

            plt.plot( #ground truth trajectory
                gt["x"], gt["y"], linewidth=0.5, marker="o",
                markersize=1, markevery=50, label=f"{robot_name} ground truth"
            )
            plt.plot( #odom trajectory
                odom["x"], odom["y"], linewidth=0.5, marker="o",
                markersize=1, markevery=50, label=f"{robot_name} odometry"
            )
            plt.plot( #slam trajectory
                slam["x"], slam["y"], linewidth=0.5, marker="o",
                markersize=1, markevery=50, label=f"{robot_name} SLAM"
            )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Merged Map com Trajetórias")
    plt.grid(True, alpha=0.3)
    plt.axis("auto")
    plt.legend()

    map_with_traj_output = RESULTS_DIR / f"merged_map_plot_with_traject.png"
    plt.savefig(map_with_traj_output, dpi=300)
    plt.close()
    print(f"Merged Map com trajetórias salvo em {map_with_traj_output}")

def plot_map(map_yaml_file, output_file, title):
    image, resolution, origin = load_map(map_yaml_file)
    extent = compute_extent(image, resolution, origin)
    image = np.flipud(image)

    plt.figure(figsize=(8, 8))
    plt.imshow(image, cmap="gray", origin="lower", extent=extent,) #mapa
    plt.scatter(0, 0, color="blue", s=10, label="Origem [0,0]", zorder=5,) #origem

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.axis("auto")
    plt.legend()

    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Merged Map salvo em {output_file}")

def main():
    yaml_file = RESULTS_DIR / "merged_map_save.yaml"
    if not yaml_file.is_file():
        print("Merged Map não encontrado.")
        return

    map_output = RESULTS_DIR / "merged_map_plot.png"
    plot_map(map_yaml_file=yaml_file, output_file=map_output, title=f"Merged Map",)

    gt_traject_files = sorted(RESULTS_DIR.glob("*_gt.csv"))
    odom_traject_files = sorted(RESULTS_DIR.glob("*_odom.csv"))
    slam_traject_files = sorted(RESULTS_DIR.glob("*_slam.csv"))
    alignment_data_file = RESULTS_DIR / "alignment_data.json"
    plot_map_with_trajectories(yaml_file, gt_traject_files, odom_traject_files, slam_traject_files, alignment_data_file)

if __name__ == "__main__":
    main()