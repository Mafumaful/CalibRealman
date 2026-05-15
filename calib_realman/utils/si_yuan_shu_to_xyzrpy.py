import numpy as np
from scipy.spatial.transform import Rotation

# ---------- 输入 ----------
quat = {
    "w": 0.7190511940325509,
    "x": -0.0022274799563179524,
    "y": 0.010556745501967548,
    "z": 0.694873350919298
}

pos = {
    "x": 0.08660505762758673,
    "y": -0.01998700594906892,
    "z": 0.06094086274144063
}

# ---------- 转换 ----------
# quaternion (w, x, y, z)
q = [quat["w"], quat["x"], quat["y"], quat["z"]]

# 转成 scipy Rotation
r = Rotation.from_quat(q)   # 注意：scipy 顺序是 [x, y, z, w]

# XYZ Euler（单位：弧度）
rpy_rad = r.as_euler('xyz', degrees=False)

# XYZ Euler（单位：度）
rpy_deg = np.degrees(rpy_rad)

# ---------- 输出 ----------
print("Position (m):")
print(f"  x = {pos['x']:.6f}")
print(f"  y = {pos['y']:.6f}")
print(f"  z = {pos['z']:.6f}")

print("\nRPY (XYZ, deg):")
print(f"  roll  = {rpy_deg[0]:.4f}")
print(f"  pitch = {rpy_deg[1]:.4f}")
print(f"  yaw   = {rpy_deg[2]:.4f}")
print([pos['x'], pos['y'], pos['z'], rpy_deg[0], rpy_deg[1], rpy_deg[2]])