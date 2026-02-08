# test_piper_chdir_fix.py

import os
import time
from omni.isaac.kit import SimulationApp

# 1. 启动
simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RayTracedLighting", 
})

import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdGeom

# 2. 创建世界
world = World(stage_units_in_meters=1.0)

# 3. 路径配置
USD_FULL_PATH = "/export/ra/liyuxuan/PiPER-X/piper_isaac_sim/USD/piper_x_v1.usd"
USD_DIR = os.path.dirname(USD_FULL_PATH) # 拿到文件夹路径
USD_FILE = os.path.basename(USD_FULL_PATH) # 拿到文件名

ROBOT_PATH = "/World/PiPER_X"

print(f"Target USD Directory: {USD_DIR}")

# --- [关键修复] 切换工作目录 ---
# 这一步让 USD 内部的 "./meshes/..." 相对路径能正确找到文件
original_dir = os.getcwd()
os.chdir(USD_DIR)
print(f"Changed working directory to: {os.getcwd()}")


# 4. 加载 USD
add_reference_to_stage(usd_path=USD_FILE, prim_path=ROBOT_PATH)

# 切回原来的目录 (为了保存图片方便)
os.chdir(original_dir)
print(f"Restored working directory to: {original_dir}")

# 5. 强制加载 Payload
world.stage.Load()

mesh_count = 0
for prim in world.stage.Traverse():
    if str(prim.GetPath()).startswith(ROBOT_PATH) and prim.GetTypeName() == "Mesh":
        mesh_count += 1

if mesh_count == 0:
    print("\n[STILL FAILED] Changing directory didn't fix it.")
    print("The USD likely references files that don't exist even in that folder.")
else:
    print(f"\n[SUCCESS] Found {mesh_count} meshes! Taking photo now...")

    # --- 拍照流程 ---
    rep.create.light(light_type="Dome", intensity=150.0, rotation=(270, 0, 0))
    rep.create.light(light_type="Distant", intensity=300.0, rotation=(315, 0, 0))

    # 自动计算相机位置
    imageable = UsdGeom.Imageable(world.stage.GetPrimAtPath(ROBOT_PATH))
    bound = imageable.ComputeWorldBound(0, UsdGeom.Tokens.default_)
    box = bound.GetRange()
    center = (box.GetMin() + box.GetMax()) / 2.0
    
    # 如果包围盒还是几乎为0 (意味着虽然有Mesh但没加载几何数据)，手动给一个位置
    if box.GetSize()[2] < 0.05:
        print("[WARNING] Bounding box is tiny. Using hardcoded camera position.")
        cam_target = (0, 0, 0.2)
        cam_pos = (1.5, 1.5, 1.0)
    else:
        cam_target = center
        cam_pos = (center[0] + 1.5, center[1] + 1.5, center[2] + 1.0)

    camera = rep.create.camera(position=cam_pos, look_at=cam_target)
    render_product = rep.create.render_product(camera, (1024, 768))

    output_dir = "/export/ra/liyuxuan/PiPER-X/img_test/"
    print(f"Saving to: {output_dir}")
    
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=output_dir, rgb=True)
    writer.attach([render_product])

    world.reset()
    print("Warming up renderer...")
    for i in range(30):
        world.step(render=True)
    
    print("Capturing...")
    for i in range(5):
        rep.orchestrator.step()
        world.step(render=True)
        time.sleep(1)

simulation_app.close()