#!/usr/bin/env python3

import sys
from pathlib import Path
import yaml
import numpy as np
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

def compute_extent(image, resolution, origin):
    height, width = image.shape

    xmin = origin[0]
    xmax = origin[0] + width * resolution
    ymin = origin[1]
    ymax = origin[1] + height * resolution

    return [xmin, xmax, ymin, ymax]

def plot_map(map_yaml_file, transparency):
    image, resolution, origin = load_map(map_yaml_file)
    extent = compute_extent(image, resolution, origin)
    image = np.flipud(image)

    plt.imshow(image, cmap="gray", origin="lower", extent=extent, alpha=transparency) #mapa

def main():
    yaml_files = sorted(RESULTS_DIR.glob("*_map_aligned_save.yaml"))
    if len(yaml_files) == 0:
        print("Nenhum mapa alinhado encontrado.")
        return

    transparency = 1.0
    delta = transparency/len(yaml_files)

    plt.figure(figsize=(8, 8))
    for yaml_file in yaml_files:
        robot_name = yaml_file.stem.replace("_map_save", "")
        print(f"Processando {robot_name}...")

        plot_map(yaml_file, transparency)
        transparency = transparency - delta

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Mapas Locais Alinhados")
    plt.grid(True, alpha=0.3)
    plt.axis("auto")

    map_output = RESULTS_DIR / "aligned_maps_plot.png"
    plt.savefig(map_output, dpi=300)
    plt.close()
    print(f"Aligned Maps salvo em {map_output}")

if __name__ == "__main__":
    main()