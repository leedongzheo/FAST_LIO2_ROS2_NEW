import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def clean(df):
    df = df[["t", "x", "y", "z"]].copy()
    df = df.replace([np.inf, -np.inf], np.nan).dropna()
    return df.reset_index(drop=True)


def path_length_xy(df):
    xy = df[["x", "y"]].to_numpy()
    return np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()


def find_point_after_distance(df, d=30.0):
    xy = df[["x", "y"]].to_numpy()
    p0 = xy[0]
    dist = np.linalg.norm(xy - p0, axis=1)
    idx = np.where(dist > d)[0]
    if len(idx) == 0:
        return min(len(df) - 1, 20)
    return int(idx[0])


def align_start_heading(src, ref):
    src_xy = src[["x", "y"]].to_numpy()
    ref_xy = ref[["x", "y"]].to_numpy()

    src0 = src_xy[0]
    ref0 = ref_xy[0]

    i_src = find_point_after_distance(src, 30.0)
    i_ref = find_point_after_distance(ref, 30.0)

    v_src = src_xy[i_src] - src0
    v_ref = ref_xy[i_ref] - ref0

    ang_src = np.arctan2(v_src[1], v_src[0])
    ang_ref = np.arctan2(v_ref[1], v_ref[0])
    theta = ang_ref - ang_src

    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    aligned = (src_xy - src0) @ R.T + ref0
    return aligned


gt = clean(pd.read_csv("groundtruth.csv"))
flio = clean(pd.read_csv("fastlio2.csv"))

print("GT rows:", len(gt), "XY length:", path_length_xy(gt))
print("F-LIO2 rows:", len(flio), "XY length:", path_length_xy(flio))
print("GT x range:", gt["x"].min(), gt["x"].max())
print("GT y range:", gt["y"].min(), gt["y"].max())
print("F-LIO2 x range:", flio["x"].min(), flio["x"].max())
print("F-LIO2 y range:", flio["y"].min(), flio["y"].max())

flio_aligned = align_start_heading(flio, gt)

plt.figure(figsize=(10, 3.2))

plt.plot(gt["x"], gt["y"], "k--", linewidth=2, label="Ground truth")
plt.plot(flio_aligned[:, 0], flio_aligned[:, 1], linewidth=2, label="Fastlio2 (Ouster) aligned")

plt.scatter(gt["x"].iloc[0], gt["y"].iloc[0], c="k", s=35)
plt.text(gt["x"].iloc[0], gt["y"].iloc[0] + 20, "Start", color="red", fontsize=13)

plt.scatter(gt["x"].iloc[-1], gt["y"].iloc[-1], c="k", s=35)
plt.text(gt["x"].iloc[-1], gt["y"].iloc[-1] + 20, "End", color="red", fontsize=13)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.legend()
plt.tight_layout()
plt.savefig("fastlio2_ouster.png", dpi=300)
plt.show()
