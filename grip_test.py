# piper_pick_cube_final.py

import numpy as np
import time
import os
import glob
import shutil
from omni.isaac.kit import SimulationApp

# 1. 启动仿真
simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.materials import PhysicsMaterial
import omni.replicator.core as rep

# ==========================================
# 虚拟 Piper 接口
# ==========================================
class VirtualPiperInterface:
    def __init__(self, robot_prim_path, world, output_dir):
        self.robot = Robot(prim_path=robot_prim_path, name="piper_virtual")
        self.world = world
        self.world.scene.add(self.robot)
        self.output_dir = output_dir
        self.current_joint_phys = np.zeros(8)
        self.FACTOR_JOINT = 57295.7795
        self.FACTOR_GRIPPER = 1000000.0 
        self._clean_output_dir()

    def _clean_output_dir(self):
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.makedirs(self.output_dir)
        # 预先创建 rgb 文件夹，防止 Replicator 找不到
        os.makedirs(os.path.join(self.output_dir, "rgb"), exist_ok=True)
        print(f"[Sim] 文件夹已清空: {self.output_dir}")

    def EnablePiper(self):
        if not self.world.is_playing():
            self.world.reset()
        return True

    def JointCtrl(self, *joints):
        rads = [j / self.FACTOR_JOINT for j in joints]
        for i in range(6):
            self.current_joint_phys[i] = rads[i]
        self._sync()

    def GripperCtrl(self, stroke_int):
        # 限制范围，防止穿模
        width = np.clip(abs(stroke_int) / self.FACTOR_GRIPPER, 0, 0.08)
        self.current_joint_phys[6] = width
        self.current_joint_phys[7] = width
        self._sync()

    def _sync(self):
        self.robot.set_joint_positions(self.current_joint_phys)

    def Capture(self, custom_name):
        """
        [增强版] 自动寻找并重命名图片
        """
        print(f"[Sim] 📸 正在拍照 -> {custom_name}.png")
        
        # 1. 触发 Replicator 写入
        rep.orchestrator.step() 
        self.world.step(render=True) 
        time.sleep(1.0) # 等待稍微久一点，确保写入完成
        
        # 2. 查找：同时查找根目录和 rgb 子目录
        search_paths = [
            os.path.join(self.output_dir, "rgb_*.png"),       # 根目录
            os.path.join(self.output_dir, "rgb", "rgb_*.png") # rgb子目录
        ]
        
        files = []
        for p in search_paths:
            files.extend(glob.glob(p))
            
        if not files:
            print("[Error] 未找到生成的图片，无法重命名！")
            return

        # 找到最新的那张
        latest_file = max(files, key=os.path.getmtime)
        
        # 移动并改名
        new_path = os.path.join(self.output_dir, f"{custom_name}.png")
        
        try:
            shutil.move(latest_file, new_path)
            print(f"[Sim] ✅ 已保存为: {os.path.basename(new_path)}")
        except Exception as e:
            print(f"[Error] 改名失败: {e}")

# ==========================================
# 场景搭建
# ==========================================
def setup_scene(world):
    # 高摩擦力是抓取成功的关键
    friction_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/HighFriction",
        static_friction=10.0,
        dynamic_friction=10.0
    )

    world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.25, 0.0, 0.1]), 
            scale=np.array([0.3, 0.4, 0.2]), 
            color=np.array([0.7, 0.7, 0.7]), 
        )
    )

    world.scene.add(
        DynamicCuboid(
            prim_path="/World/TargetCube",
            name="target_cube",
            position=np.array([0.25, 0.0, 0.22]), # 在桌子中心上方
            scale=np.array([0.03, 0.03, 0.03]), # 3cm 方块
            color=np.array([1.0, 0.0, 0.0]), 
            mass=0.1, 
            physics_material=friction_material
        )
    )

# ==========================================
# 主程序
# ==========================================
world = World(stage_units_in_meters=1.0)
USD_PATH = "/export/ra/liyuxuan/PiPER-X/piper_isaac_sim/USD/piper_x_v1.usd"
ROBOT_PATH = "/World/PiPER_X"

# 加载机器人
original_dir = os.getcwd()
os.chdir(os.path.dirname(USD_PATH))
add_reference_to_stage(usd_path=os.path.basename(USD_PATH), prim_path=ROBOT_PATH)
os.chdir(original_dir)
world.stage.Load()

setup_scene(world)

# --- 灯光设置 ---
rep.create.light(light_type="Dome", intensity=500.0, rotation=(270, 0, 0))
rep.create.light(light_type="Distant", intensity=800.0, rotation=(315, 0, 45))
rep.create.light(light_type="Distant", intensity=500.0, rotation=(315, 0, 225))

# --- 摄像头设置 ---
cam = rep.create.camera(position=(0.6, 0.6, 1.3), look_at=(0.25, 0, 0.2))

OUTPUT_DIR = "/export/ra/liyuxuan/PiPER-X/img_pick_fixed/"
render_product = rep.create.render_product(cam, (1024, 768))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUTPUT_DIR, rgb=True)
writer.attach([render_product])

# 禁止自动拍照
rep.orchestrator.set_capture_on_play(False)

piper = VirtualPiperInterface(ROBOT_PATH, world, OUTPUT_DIR)

if __name__ == "__main__":
    piper.EnablePiper()
    factor = 57295.7795
    
    # === 关键动作序列 ===
    # 针对方块位置 (0.25, 0, 0.22) 计算的角度
    
    # 1. 接近 (高处)
    pose_approach = [0.0, 0.2, 0.0, 0.0, -0.2, 0.0] 
    
    # 2. 下探 (准备抓取)
    # 这里的角度让夹爪垂直向下，并到达方块高度
    pose_grasp = [0.0, 0.45, -0.4, 0.0, -0.6, 0.0]
    
    # 3. 提起 (抓到后抬起)
    pose_lift = [0.0, 0.2, -0.4, 0.0, -0.2, 0.0]

    print("Pre-warming renderer...")
    world.reset()
    for i in range(60): world.step(render=True)
        
    # --- Step 1: 移动到上方，张开夹爪 ---
    print("Action: Approach")
    joints = [round(p * factor) for p in pose_approach]
    piper.JointCtrl(*joints)
    piper.GripperCtrl(50000) # 张开
    
    # 等待动作稳定
    for i in range(60): world.step(render=True)
    piper.Capture("01_Approach")

    # --- Step 2: 下探到位 ---
    print("Action: Go Down")
    joints = [round(p * factor) for p in pose_grasp]
    piper.JointCtrl(*joints)
    piper.GripperCtrl(50000) # 保持张开
    
    for i in range(60): world.step(render=True)
    piper.Capture("02_Ready_To_Grasp")

    # --- Step 3: 闭合夹爪 (抓取) ---
    print("Action: Close Gripper")
    joints = [round(p * factor) for p in pose_grasp] # 保持位置不动
    piper.JointCtrl(*joints)
    
    # 关键：方块是 3cm (30000微米)，我们要闭合到比它小一点点 (比如20000)
    # 这样物理引擎才会产生“挤压”力
    piper.GripperCtrl(20000) 
    
    for i in range(60): world.step(render=True)
    piper.Capture("03_Closed")

    # --- Step 4: 提起方块 ---
    print("Action: Lift")
    joints = [round(p * factor) for p in pose_lift]
    piper.JointCtrl(*joints)
    piper.GripperCtrl(20000) # 保持紧闭
    
    for i in range(60): world.step(render=True)
    piper.Capture("04_Lifted")
    
    print("Mission Complete.")
    simulation_app.close()