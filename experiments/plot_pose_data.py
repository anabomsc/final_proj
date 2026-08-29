import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
from pathlib import Path

if len(sys.argv) != 2:
    print(f"Uso: python {sys.argv[0]} <diretorio_dos_trajetorias>")
    sys.exit(1)

RESULTS_DIR = Path(sys.argv[1])

if not RESULTS_DIR.exists():
    print(f"Diretório não encontrado: {RESULTS_DIR}")
    sys.exit(1)

def load_trajectories(gt_file, odom_file, slam_file):
    gt_data = pd.read_csv(gt_file)
    odom_data = pd.read_csv(odom_file)
    slam_data = pd.read_csv(slam_file)

    return gt_data, odom_data, slam_data

def plot_trajectories(gt, odom, slam, output_file, title):
    plt.figure(figsize=(8, 8))
    plt.plot( #ground truth trajectory
        gt["x"], gt["y"], linewidth=1, marker="o",
        markersize=2, markevery=100, label="Ground Truth"
    )
    plt.plot( #odom trajectory
        odom["x"], odom["y"], linewidth=1, marker="x",
        markersize=2, markevery=100, label="Odometry"
    )
    plt.plot( #slam trajectory
        slam["x"], slam["y"], linewidth=1, marker="s",
        markersize=2, markevery=100, label="SLAM"
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(title)
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Figura salva em {output_file}")

def main():
    gt_traject_files = sorted(RESULTS_DIR.glob("*_gt.csv"))
    odom_traject_files = sorted(RESULTS_DIR.glob("*_odom.csv"))
    slam_traject_files = sorted(RESULTS_DIR.glob("*_slam.csv"))

    traject_set_complete = (len(gt_traject_files) == len(odom_traject_files) == len(slam_traject_files))
    traject_exist = (len(gt_traject_files) != 0) and (len(odom_traject_files) != 0) and (len(slam_traject_files) != 0)
    trajectories_ok = traject_set_complete and traject_exist
    if not trajectories_ok:
        print("Erro ao procurar trajetórias [gt, odom, slam].")
        return

    for gt_file, odom_file, slam_file in zip(gt_traject_files, odom_traject_files, slam_traject_files):
        robot_name = odom_file.stem.replace("_odom", "")
        print(f"Processando {robot_name}...")

        gt, odom, slam = load_trajectories(gt_file, odom_file, slam_file)
        output_png = os.path.join(RESULTS_DIR, f"trajectory_{robot_name}.png")

        plot_trajectories(
            gt=gt, odom=odom, slam=slam, output_file=output_png, title=f"Trajetória - {robot_name}",
        )

if __name__ == "__main__":
    main()