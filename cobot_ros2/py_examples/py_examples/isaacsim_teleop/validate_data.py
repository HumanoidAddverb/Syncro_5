import os
import cv2
import numpy as np
import pandas as pd
from pathlib import Path

# ---- paths ----
root = Path("/vla_dataset/syncro5/teleop_session_1778229489/")   # change this
parquet_path = root / "data" / "chunk-000" /"file-000.parquet"

ego_dir = root / "images"/"observation.images.ego/episode_001"
ext_dir = root / "images"/"observation.images.external/episode_001"

# ---- load parquet ----
df = pd.read_parquet(parquet_path)
df.to_csv("csv_for_teleop.csv", index=False)


print(f"Total rows in parquet: {len(df)}")

# =========================================================
# 1. IMAGE VALIDATION
# =========================================================

def validate_images(folder):
    files = sorted(os.listdir(folder))
    missing = 0
    corrupt = 0

    for f in files:
        path = folder / f

        if not path.exists():
            missing += 1
            continue

        img = cv2.imread(str(path))
        if img is None:
            corrupt += 1

    return len(files), missing, corrupt


ego_total, ego_missing, ego_corrupt = validate_images(ego_dir)
ext_total, ext_missing, ext_corrupt = validate_images(ext_dir)

print("\n--- IMAGE CHECK ---")
print(f"EGO     → total: {ego_total}, missing: {ego_missing}, corrupt: {ego_corrupt}")
print(f"EXTERNAL→ total: {ext_total}, missing: {ext_missing}, corrupt: {ext_corrupt}")

# =========================================================
# 2. COUNT CONSISTENCY
# =========================================================

print("\n--- COUNT CHECK ---")
print(f"Parquet rows: {len(df)}")
print(f"EGO images : {ego_total}")
print(f"EXT images : {ext_total}")

if len(df) != ego_total or len(df) != ext_total:
    print("❌ MISMATCH: row count != image count")
else:
    print("✅ Counts match")


print(df.columns)
# =========================================================
# 3. PARQUET DATA VALIDATION
# =========================================================

def has_nan(arr):
    try:
        return np.isnan(arr).any()
    except:
        return True

bad_obs = 0
bad_action = 0
bad_ts = 0

for _, row in df.iterrows():

    if has_nan(row["state.joint_positions"]):
        bad_obs += 1

    if has_nan(row["action.robot"]):
        bad_action += 1

    if pd.isnull(row["timestamp"]):
        bad_ts += 1

print("\n--- PARQUET CHECK ---")
print(f"Bad observation rows: {bad_obs}")
print(f"Bad action rows     : {bad_action}")
print(f"Bad timestamps      : {bad_ts}")

# =========================================================
# 4. (OPTIONAL) TIMESTAMP SANITY
# =========================================================

ts = df["timestamp"].values
dt = np.diff(ts)

print("\n--- TIMESTAMP CHECK ---")
print(f"Min dt: {dt.min():.4f}")
print(f"Max dt: {dt.max():.4f}")
print(f"Mean dt: {dt.mean():.4f}")