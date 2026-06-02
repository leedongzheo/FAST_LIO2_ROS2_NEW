#!/usr/bin/env python3
import sys
import csv
import math


def is_float(x):
    try:
        float(x)
        return True
    except Exception:
        return False


def rotmat_to_quat(R):
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]

    tr = r00 + r11 + r22

    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (r21 - r12) / S
        qy = (r02 - r20) / S
        qz = (r10 - r01) / S
    elif (r00 > r11) and (r00 > r22):
        S = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / S
        qx = 0.25 * S
        qy = (r01 + r10) / S
        qz = (r02 + r20) / S
    elif r11 > r22:
        S = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / S
        qx = (r01 + r10) / S
        qy = 0.25 * S
        qz = (r12 + r21) / S
    else:
        S = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / S
        qx = (r02 + r20) / S
        qy = (r12 + r21) / S
        qz = 0.25 * S

    return qx, qy, qz, qw


def parse_numeric_lines(path):
    rows = []

    with open(path, "r", errors="ignore") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            if line.startswith("#"):
                continue

            line = line.replace(",", " ")
            parts = line.split()

            if len(parts) == 0:
                continue

            if not all(is_float(p) for p in parts):
                continue

            rows.append([float(p) for p in parts])

    return rows


def convert(in_path, out_path):
    rows = parse_numeric_lines(in_path)

    if not rows:
        raise RuntimeError("Không đọc được dòng số nào trong Groundtruth.txt")

    ncol = len(rows[0])
    print("Detected columns:", ncol)
    print("First numeric row:", rows[0])

    out_rows = []

    for i, r in enumerate(rows):
        n = len(r)

        # Format TUM phổ biến:
        # t x y z qx qy qz qw
        if n >= 8 and n < 12:
            t = r[0]
            x, y, z = r[1], r[2], r[3]
            qx, qy, qz, qw = r[4], r[5], r[6], r[7]

        # Format ma trận pose 3x4, không có timestamp:
        # r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz
        elif n == 12:
            t = float(i)
            R = [
                [r[0], r[1], r[2]],
                [r[4], r[5], r[6]],
                [r[8], r[9], r[10]],
            ]
            x, y, z = r[3], r[7], r[11]
            qx, qy, qz, qw = rotmat_to_quat(R)

        # Format timestamp + ma trận pose 3x4:
        # t r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz
        elif n >= 13:
            t = r[0]
            a = r[1:13]
            R = [
                [a[0], a[1], a[2]],
                [a[4], a[5], a[6]],
                [a[8], a[9], a[10]],
            ]
            x, y, z = a[3], a[7], a[11]
            qx, qy, qz, qw = rotmat_to_quat(R)

        # Format tối giản:
        # t x y z
        elif n >= 4:
            t = r[0]
            x, y, z = r[1], r[2], r[3]
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        else:
            continue

        out_rows.append([t, x, y, z, qx, qy, qz, qw])

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(out_rows)

    print("Saved:", out_path)
    print("Rows exported:", len(out_rows))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 convert_groundtruth_to_csv.py Groundtruth.txt groundtruth.csv")
        sys.exit(1)

    convert(sys.argv[1], sys.argv[2])
