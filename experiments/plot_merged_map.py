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

def load_json(file):
    with open(file, "r") as f:
        data = json.load(f)
    return data

def compute_extent(image, resolution, origin):
    height, width = image.shape

    xmin = origin[0]
    xmax = origin[0] + width * resolution
    ymin = origin[1]
    ymax = origin[1] + height * resolution

    return [xmin, xmax, ymin, ymax]

def apply_transformation(trajectory, T):
    transformed = []
    for x, y in zip(trajectory["x"], trajectory["y"]):
        p = np.array([x, y, 1.0])
        p_new = T @ p
        transformed.append((p_new[0], p_new[1]))
    return pd.DataFrame(transformed, columns=['x', 'y'])

def compute_trajectory_transformation(robot_name, gt_data, odom_data, slam_data, transformation_data):
    T = transformation_data["transformation"][robot_name]
    gt_data = apply_transformation(gt_data, T)
    odom_data = apply_transformation(odom_data, T)
    slam_data = apply_transformation(slam_data, T)

    return gt_data, odom_data, slam_data 

def plot_map_with_trajectories(map_yaml_file, gt_traject_files, odom_traject_files, slam_traject_files, transformation_file):
    image, resolution, origin = load_map(map_yaml_file)
    extent = compute_extent(image, resolution, origin)
    map = np.flipud(image)
    plt.figure(figsize=(8, 8))
    plt.imshow(map, cmap="gray", origin="lower", extent=extent,) # merged map

    transform_data = load_json(transformation_file)
    transformed_robots = set(transform_data["transformation"].keys())
    for gt_file, odom_file, slam_file in zip(gt_traject_files, odom_traject_files, slam_traject_files):
            robot_name = gt_file.stem.replace("_gt", "")
            print(f"Processando trajetória {robot_name}...")

            gt, odom, slam = load_trajectories(gt_file, odom_file, slam_file)
            if robot_name in transformed_robots:
                print(f"Transformando trajetória de {robot_name}")
                gt, odom, slam = compute_trajectory_transformation(
                    robot_name, gt, odom, slam, transform_data,
                )

            plt.plot( #ground truth trajectory
                gt["x"], gt["y"], linewidth=0.5, marker="o",
                markersize=2, markevery=100, label=f"{robot_name} ground truth"
            )
            plt.plot( #odom trajectory
                odom["x"], odom["y"], linewidth=0.5, marker="x",
                markersize=2, markevery=100, label=f"{robot_name} odometry"
            )
            plt.plot( #slam trajectory
                slam["x"], slam["y"], linewidth=0.5, marker="s",
                markersize=2, markevery=100, label=f"{robot_name} SLAM"
            )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Merged Map com Trajetórias")
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
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
    plt.axis("equal")
    plt.legend()

    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"{title} salvo em {output_file}")

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
    transformation_data_file = RESULTS_DIR / "transformation_data.json"
    plot_map_with_trajectories(yaml_file, gt_traject_files, odom_traject_files, slam_traject_files, transformation_data_file)

if __name__ == "__main__":
    main()