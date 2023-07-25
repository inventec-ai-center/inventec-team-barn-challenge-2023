#!/usr/bin/python3
import os
import argparse
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt


class BARNPlot:
    def __init__(self, file_name) -> None:
        plt.rcParams.update({"font.size": 16})
        self._file_name = file_name

    def run(self) -> None:
        """TODO:"""
        txt_file = os.path.join(self._file_name)

        try:
            df = pd.read_csv(
                txt_file,
                delim_whitespace=True,
                names=[
                    "world_idx",
                    "success",
                    "collided",
                    "timeout",
                    "time",
                    "nav_metic",
                ],
            )
        except:
            print(f"File `{self._file_name}` not found...")
            return

        world_idx = df["world_idx"].unique()
        result_rate = []

        for idx in world_idx:
            df_world = df.loc[(df["world_idx"] == idx)]
            success = df_world["success"].sum()
            collided = df_world["collided"].sum()
            timeout = df_world["timeout"].sum()
            # fail = df_world["success"].size - success

            result_rate.append([idx, success, collided, timeout])

        np_result_rate = np.array(result_rate)

        total_sucess = df["success"].mean()
        total_collided = df["collided"].mean()
        total_timeout = df["timeout"].mean()

        bar_width = 4
        fig, ax = plt.subplots(figsize=(20, 8))
        fig.tight_layout()

        ax.bar(
            np_result_rate[:, 0],
            np_result_rate[:, 1],
            width=bar_width,
            edgecolor="gray",
            label=f"Success: {total_sucess:.3f}",
            align="center",
        )

        ax.bar(
            np_result_rate[:, 0],
            np_result_rate[:, 2],
            bottom=np_result_rate[:, 1],
            width=bar_width,
            edgecolor="gray",
            label=f"Collided: {total_collided:.3f}",
            align="center",
        )

        ax.bar(
            np_result_rate[:, 0],
            np_result_rate[:, 3],
            bottom=np_result_rate[:, 1] + np_result_rate[:, 2],
            width=bar_width,
            edgecolor="gray",
            label=f"Timeout: {total_timeout:.3f}",
            align="center",
        )

        ax.set_xticks(
            [r for r in world_idx],
        )
        ax.set_xticklabels([str(r) for r in world_idx], rotation=90)

        # ax.set_ylim([0, 20])
        ax.set_xlabel("World Index")
        ax.set_ylabel("Num of Trial")
        ax.set_title(self._file_name)
        ax.legend()

        fig_name = os.path.join(os.getcwd(), self._file_name)
        plt.savefig(f"{fig_name}.png")
        # plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BARN plot results")
    parser.add_argument("--out_path", type=str, default="out.txt")
    args = parser.parse_args()

    plot = BARNPlot(file_name=args.out_path)
    plot.run()
