#!/usr/bin/env python3

import sys
import os
from pathlib import Path

import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image

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

def compute_extent(image, resolution, origin):
    height, width = image.shape

    xmin = origin[0]
    xmax = origin[0] + width * resolution
    ymin = origin[1]
    ymax = origin[1] + height * resolution

    return [xmin, xmax, ymin, ymax]

def plot_map(image, resolution, origin, output_file, title):
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

def plot_map_with_trajectories(image, resolution, origin, gt, odom, slam, output_file, title):
    extent = compute_extent(image, resolution, origin)
    map = np.flipud(image)

    plt.figure(figsize=(8, 8))
    plt.imshow(map, cmap="gray", origin="lower", extent=extent,) #mapa
    plt.plot( #ground truth trajectory
        gt["x"], gt["y"], linewidth=0.5, marker="o",
        markersize=1, markevery=50, label="Ground Truth"
    )
    plt.plot( #odom trajectory
        odom["x"], odom["y"], linewidth=0.5, marker="o",
        markersize=1, markevery=50, label="Odometry"
    )
    plt.plot( #slam trajectory
        slam["x"], slam["y"], linewidth=0.5, marker="o",
        markersize=1, markevery=50, label="SLAM"
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.axis("auto")
    plt.legend()

    plt.savefig(output_file, dpi=300)
    plt.close()

def main():
    if len(sys.argv) != 2:
        print(f"Uso: python {sys.argv[0]} <diretorio_dos_mapas>")
        sys.exit(1)
    
    RESULTS_DIR = Path(sys.argv[1])

    if not RESULTS_DIR.exists():
        print(f"Diretório não encontrado: {RESULTS_DIR}")
        sys.exit(1)

    yaml_files = sorted(RESULTS_DIR.glob("*_map_save.yaml"))
    gt_traject_files = sorted(RESULTS_DIR.glob("*_gt.csv"))
    odom_traject_files = sorted(RESULTS_DIR.glob("*_odom.csv"))
    slam_traject_files = sorted(RESULTS_DIR.glob("*_slam.csv"))

    if len(yaml_files) == 0:
        print("Nenhum mapa encontrado.")
        return

    for yaml_file, gt_file, odom_file, slam_file in zip(yaml_files, gt_traject_files, odom_traject_files, slam_traject_files):
        robot_name = yaml_file.stem.replace("_map_save", "")
        print(f"Processando {robot_name}...")
        
        image, resolution, origin = load_map(yaml_file)
        gt, odom, slam = load_trajectories(gt_file, odom_file, slam_file)

        map_output = RESULTS_DIR / f"{robot_name}_map_plot.png"
        map_with_traj_output = RESULTS_DIR / f"{robot_name}_map_plot_with_traject.png"

        plot_map(
            image=image, resolution=resolution, origin=origin,
            output_file=map_output, title=f"Mapa Local: {robot_name}",
        )
        print(f"Mapa salvo em {map_output}")

        plot_map_with_trajectories(
            image=image, resolution=resolution, origin=origin, gt=gt, odom=odom, slam=slam,
            output_file=map_with_traj_output, title=f"Mapa Local com Trajetórias: {robot_name}",
        )
        print(f"Mapa com trajetórias salvo em {map_with_traj_output}")

if __name__ == "__main__":
    main()