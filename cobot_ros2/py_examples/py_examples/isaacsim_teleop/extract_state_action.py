import pandas as pd
from pathlib import Path
import ast

# ── Input / Output Paths ─────────────────────────────────────────────
root = Path("/vla_dataset/syncro5/teleop_session_1778229489/")   # change this
parquet_path = root / "data" / "chunk-000" /"file-000.parquet"

CSV_FILE     = "joint_positions_actions.csv"

# ── Load Parquet ─────────────────────────────────────────────────────
df = pd.read_parquet(parquet_path)

# ── Extract Required Columns ─────────────────────────────────────────
joint_positions = df["state.joint_positions"]
actions         = df["action.robot"]

# ── Expand list columns into separate columns ────────────────────────
joint_df = pd.DataFrame(
    joint_positions.tolist(),
    columns=[f"joint_position_{i}" for i in range(len(joint_positions.iloc[0]))]
)

action_df = pd.DataFrame(
    actions.tolist(),
    columns=[f"action_{i}" for i in range(len(actions.iloc[0]))]
)

filtered_df = pd.DataFrame({
    "joint1_action": actions.apply(lambda x: x[0]),
    "joint2_action": actions.apply(lambda x: x[1]),
    "joint3_action": actions.apply(lambda x: x[2]),
    "gripper_action": actions.apply(lambda x: x[-1]),
})

# ── Combine into Final CSV ───────────────────────────────────────────
# out_df = pd.concat([joint_df, action_df], axis=1)

# Optional: keep timestamp/frame number
# if "timestamp" in df.columns:
#     filtered_df.insert(0, "timestamp", df["timestamp"])

# if "frame_number" in df.columns:
#     out_df.insert(0, "frame_number", df["frame_number"])

# ── Save CSV ─────────────────────────────────────────────────────────
filtered_df.to_csv(CSV_FILE, index=False)

print(f"Saved CSV to: {CSV_FILE}")