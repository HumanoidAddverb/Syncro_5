import pandas as pd
from pathlib import Path


# ---- paths ----
root = Path("/vla_dataset/syncro5/teleop_session_1777970218/")   # change this
parquet_path = root / "data" / "chunk-000" /"file-000.parquet"

# Read parquet file
df = pd.read_parquet(parquet_path)

# Save as CSV
df.to_csv("teleop_fixed_csv.csv", index=False)