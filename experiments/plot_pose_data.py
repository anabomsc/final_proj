import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

RESULTS_DIR = sys.argv[1]


ROBOTS = {
    "artbot1": {
        "ground_truth": "artbot1_gt.csv",
        "odom": "artbot1_odom.csv",
        "slam": "artbot1_slam.csv"
    },
    "artbot2": {
        "ground_truth": "artbot2_gt.csv",
        "odom": "artbot2_odom.csv",
        "slam": "artbot2_slam.csv"
    }
}

for robot_name, files in ROBOTS.items():

    gt = pd.read_csv(
        os.path.join(
            RESULTS_DIR,
            files["ground_truth"]
        )
    )

    odom = pd.read_csv(
        os.path.join(
            RESULTS_DIR,
            files["odom"]
        )
    )

    slam = pd.read_csv(
        os.path.join(
            RESULTS_DIR,
            files["slam"]
        )
    )

    plt.figure(figsize=(8, 8))

    plt.plot(
        gt["x"],
        gt["y"],
        linewidth=1,
        marker="o",
        markersize=0.5,
        label="Ground Truth"
    )

    plt.plot(
        odom["x"],
        odom["y"],
        linewidth=1,
        label="Odometry"
    )

    plt.plot(
        slam["x"],
        slam["y"],
        linewidth=1,
        label="SLAM"
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

    plt.title(
        f"Trajetória - {robot_name}"
    )

    plt.grid(True)
    plt.axis("equal")
    plt.legend()

    plt.tight_layout()

    output_png = os.path.join(
        RESULTS_DIR,
        f"trajectory_{robot_name}.png"
    )

    plt.savefig(
        output_png,
        dpi=300
    )

    plt.close()

    print(
        f"Figura salva em {output_png}"
    )