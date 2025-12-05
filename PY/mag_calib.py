#!/usr/bin/env python3
"""
mag_calib.py
Ellipsoid fit for magnetometer hard-iron and (diagonal) soft-iron correction.
Input: CSV file with columns mx,my,mz (in sensor units)
Output: JSON with mag_offset (3) and mag_scale (3) suitable for tracker calibration.

Usage:
  python mag_calib.py --in mag_samples.csv --out mag_calib.json
If you don't have CSV, you can pipe whitespace-separated triples.
"""
import argparse, numpy as np, json, sys

def ellipsoid_fit(X):
    # X: N x 3 (mx,my,mz)
    # Fit ellipsoid: x.T * A * x + b.T * x + c = 0
    # Solve linear system for symmetric A (6 unique), b (3), c (1)
    D = np.column_stack([X[:,0]*X[:,0], X[:,1]*X[:,1], X[:,2]*X[:,2],
                         2*X[:,0]*X[:,1], 2*X[:,0]*X[:,2], 2*X[:,1]*X[:,2],
                         2*X[:,0], 2*X[:,1], 2*X[:,2], np.ones(X.shape[0])])
    # Solve D * v = 0 with constraint ||v||=1 -> use SVD smallest singular vector
    _,_,Vt = np.linalg.svd(D, full_matrices=False)
    v = Vt.T[:,-1]
    # build matrices
    A = np.array([[v[0], v[3], v[4]],
                  [v[3], v[1], v[5]],
                  [v[4], v[5], v[2]]])
    b = np.array([v[6], v[7], v[8]])
    c = v[9]
    # center:
    center = -0.5 * np.linalg.inv(A).dot(b)
    # translate to center
    translated = X - center
    # compute scale (radii) from eigen decomposition
    R = translated.dot(np.linalg.inv(A)).dot(translated.T)
    # compute scale factors via formula
    val = (center.T.dot(A).dot(center) - c)
    eigvals, eigvecs = np.linalg.eig(A / val)
    radii = np.sqrt(1.0 / eigvals)
    return center, radii, A, b, c

def calibrate_mag(data):
    X = np.array(data, dtype=float)
    if X.shape[0] < 20:
        raise ValueError("need >=20 samples for ellipsoid fit")
    center, radii, A, b, c = ellipsoid_fit(X)
    # produce offsets (hard iron) = center, soft-iron scale diag = mean(radii)/radii
    scales = float(np.mean(radii)) / radii
    return {"mag_offset": [float(center[0]), float(center[1]), float(center[2])],
            "mag_scale": [float(scales[0]), float(scales[1]), float(scales[2])],
            "radii": [float(radii[0]), float(radii[1]), float(radii[2])],
            "A": A.tolist()}

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="input", required=True)
    p.add_argument("--out", dest="out", default="mag_calib.json")
    args = p.parse_args()
    data = []
    # support simple CSV with mx,my,mz or whitespace triples
    with open(args.input, "r") as f:
        for line in f:
            line=line.strip()
            if not line: continue
            parts = [t for t in line.replace(",", " ").split() if t!='']
            if len(parts) < 3: continue
            try:
                mx, my, mz = float(parts[0]), float(parts[1]), float(parts[2])
                data.append([mx,my,mz])
            except:
                continue
    if not data:
        print("no samples read")
        sys.exit(1)
    out = calibrate_mag(data)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print("Saved", args.out)
    print("mag_offset:", out["mag_offset"])
    print("mag_scale:", out["mag_scale"])

if __name__ == "__main__":
    main()
