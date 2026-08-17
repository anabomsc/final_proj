#!/usr/bin/env python3

import sys
import os
from pathlib import Path
import yaml
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
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

def pose2matrix(tx,ty,theta):
    c = math.cos(theta)
    s = math.sin(theta)
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

def compute_coords(image, origin, resolution):
    height, width = image.shape
    theta = origin[2]
    origin_T = pose2matrix(origin[0], origin[1], theta)

    x_data_idx = np.arange(width)
    y_data_idx = np.arange(height)[::-1]
    x_data, y_data = np.meshgrid(x_data_idx, y_data_idx)
    tx_map = x_data*resolution
    ty_map = y_data*resolution

    x_coords = tx_map*origin_T[0][0] + ty_map*origin_T[0][1] + origin_T[0][2]
    y_coords = tx_map*origin_T[1][0] + ty_map*origin_T[1][1] + origin_T[1][2]

    return x_coords, y_coords

def plot_map(map_yaml_file, transparency):
    image, resolution, origin = load_map(map_yaml_file)
    x_coords, y_coords = compute_coords(image, origin, resolution)
    plt.pcolormesh(x_coords, y_coords, image, cmap='gray', shading='auto', alpha=transparency)

def main():
    yaml_files = sorted(RESULTS_DIR.glob("*_map_aligned_save.yaml"))
    if len(yaml_files) == 0:
        print("Nenhum mapa alinhado encontrado.")
        return

    transparency = 1.0
    delta = transparency/len(yaml_files)

    plt.figure(figsize=(8, 8))
    for yaml_file in yaml_files:
        robot_name = yaml_file.stem.replace("_map_aligned_save", "")
        print(f"Processando {robot_name}...")

        plot_map(yaml_file, transparency)
        transparency = transparency - delta

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Mapas Locais Alinhados")
    plt.grid(True, alpha=0.3)
    plt.axis("equal")

    map_output = RESULTS_DIR / "aligned_maps_plot.png"
    plt.savefig(map_output, dpi=300)
    plt.close()
    print(f"Aligned Maps salvo em {map_output}")

    for yaml_file in yaml_files:
        robot_name = yaml_file.stem.replace("_map_aligned_save", "")
        output_png = os.path.join(RESULTS_DIR, f"{robot_name}_map_aligned.png")
        plt.figure(figsize=(8, 8))
        plot_map(yaml_file, transparency=1.0)
        plt.scatter(0, 0, color="blue", s=10, label="Origem [0,0]", zorder=5,) #origem
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title(f"Map Aligned {robot_name}")
        plt.grid(True, alpha=0.3)
        plt.axis("equal")
        plt.legend()
        plt.savefig(output_png, dpi=300)
        plt.close()
        print(f"Map Aligned {robot_name} salvo em {output_png}")

if __name__ == "__main__":
    main()