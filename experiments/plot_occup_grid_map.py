#!/usr/bin/env python3

import sys
from pathlib import Path

import yaml
import numpy as np
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
    plt.scatter(0, 0, color="blue", s=20, label="Origem [0,0]", zorder=5,) #origem

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

    if len(yaml_files) == 0:
        print("Nenhum mapa encontrado.")
        return
    print(f"{len(yaml_files)} mapas encontrados.\n")

    for yaml_file in yaml_files:
        robot_name = yaml_file.stem.replace("_map_save", "")
        image, resolution, origin = load_map(yaml_file)
        output = RESULTS_DIR / f"{robot_name}_map_plot.png"
        print(f"Processando {robot_name}")

        plot_map(
            image=image, resolution=resolution, origin=origin,
            output_file=output, title=f"Mapa Local: {robot_name}",
        )
        print(f"Salvo em {output}\n")

if __name__ == "__main__":
    main()