# test_piper_sdk_renamer.py

import time
import numpy as np
import os
import glob
import shutil
from omni.isaac.kit import SimulationApp

# 1. 启动 Isaac Sim
simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.replicator.core as rep

# ==========================================
# [虚拟 SDK] 包含自动改名功能的增强版
# ==========================================
class VirtualPiperInterface:
    def __init__(self, robot_prim_path, world, output_dir):
        self.robot = Robot(prim_path=robot_prim_path, name="piper_virtual")
        self.world = world
        self.output_dir = output_dir
        self.world.scene.add(self.robot)
        
        # 内部物理状态
        self.current_joint_phys = np.zeros(8)
        self.FACTOR_JOINT = 57295.7795
        self.FACTOR_GRIPPER = 1000000.0 
        
        # [关键] 记录当前 Replicator 计数，用于找文件
        self.rep_counter = 0

        # 初始化时清空文件夹，防止改名出错
        self._clean_output_dir()

    def _clean_output_dir(self):
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.makedirs(self.output_dir)
        print(f"[Sim] 文件夹已清空: {self.output_dir}")

    def ConnectPort(self):
        print("[Sim] 端口连接")

    def EnablePiper(self):
        if not self.world.is_playing():
            self.world.reset()
        return True

    def MotionCtrl_2(self, *args):
        pass

    def JointCtrl(self, j1, j2, j3, j4, j5, j6):
        # 整数 -> 弧度
        rads = [j / self.FACTOR_JOINT for j in [j1, j2, j3, j4, j5, j6]]
        for i in range(6):
            self.current_joint_phys[i] = rads[i]
        self._sync_sim()

    def GripperCtrl(self, stroke_int, force, speed, mode):
        # 微米 -> 米
        width = np.clip(abs(stroke_int) / self.FACTOR_GRIPPER, 0, 0.1)
        self.current_joint_phys[6] = width
        self.current_joint_phys[7] = width
        self._sync_sim()

    def _sync_sim(self):
        """只更新物理，绝对不触发拍照！"""
        self.robot.set_joint_positions(self.current_joint_phys)
        self.world.step(render=True) # 这里 render=True 只是为了物理计算光照，不会保存图片

    def SimWait(self, seconds):
        """代替 time.sleep，保持物理刷新"""
        steps = int(seconds * 60)
        for _ in range(steps):
            self.world.step(render=True)

    def Capture(self, custom_name):
        """
        [核心功能] 拍照并立即改名
        custom_name: 例如 "01_zero_pose" (不需要加 .png)
        """
        print(f"[Sim] 📸 正在拍照 -> {custom_name}.png")
        
        # 1. 触发 Replicator 写入一张图片
        rep.orchestrator.step() 
        self.world.step(render=True) 
        time.sleep(0.5) 
        
        
        # 查找所有 png
        files = glob.glob(os.path.join(self.output_dir, "rgb_*.png"))


        # 找到最新的那张 (按修改时间排序)
        latest_file = max(files, key=os.path.getmtime)
        
        # 目标文件名
        new_path = os.path.join(self.output_dir, f"{custom_name}.png")
        
        try:
            # 移动并改名 (从 rgb 子文件夹 移到 外面，并改名)
            shutil.move(latest_file, new_path)
            print(f"[Sim] ✅ 已保存为: {os.path.basename(new_path)}")
        except Exception as e:
            print(f"[Error] 改名失败: {e}")


# ==========================================
# 主程序
# ==========================================

world = World(stage_units_in_meters=1.0)
USD_PATH = "/export/ra/liyuxuan/PiPER-X/piper_isaac_sim/USD/piper_x_v1.usd"
ROBOT_PATH = "/World/PiPER_X"
OUTPUT_DIR = "/export/ra/liyuxuan/PiPER-X/img_renamed/" # 改个新目录

# 路径修复
original_dir = os.getcwd()
os.chdir(os.path.dirname(USD_PATH))
add_reference_to_stage(usd_path=os.path.basename(USD_PATH), prim_path=ROBOT_PATH)
os.chdir(original_dir)
world.stage.Load()

# 灯光与相机
rep.create.light(light_type="Dome", intensity=150.0, rotation=(270,0,0))
rep.create.light(light_type="Distant", intensity=300.0, rotation=(315,0,0))
cam = rep.create.camera(position=(1.2, 1.2, 0.8), look_at=(0,0,0.2))
render_product = rep.create.render_product(cam, (1024, 768))

# 初始化 Replicator Writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUTPUT_DIR, rgb=True)
writer.attach([render_product])

# [关键修复] 禁止自动在 physics step 时拍照，只有调用 orchestrator.step() 时才拍
rep.orchestrator.set_capture_on_play(False)

# 初始化虚拟 SDK
piper = VirtualPiperInterface(ROBOT_PATH, world, OUTPUT_DIR)

if __name__ == "__main__":
    piper.ConnectPort()
    piper.EnablePiper()
    
    print("--- 开始动作序列 ---")
    
    # 状态变量
    position = [0]*7
    count = 0
    factor = 57295.7795
    
    # 我们跑 800 帧，确保覆盖所有动作
    for _ in range(800):
        count += 1
        
        # --- 1. 下达动作指令 ---
        if count == 10:
            position = [0]*7 # 归零
            
        elif count == 300:
            # 张开夹爪 (0.05m -> 50000微米)
            position = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5, 0.05]
            
        elif count == 600:
            position = [0]*7 # 回归零

        # --- 2. 驱动机器人 (只动，不拍) ---
        # 计算关节整数值
        joints_int = [round(p * factor) for p in position[:6]]
        gripper_int = round(position[6] * 1000 * 1000)
        
        piper.JointCtrl(*joints_int)
        piper.GripperCtrl(gripper_int, 1000, 0x01, 0)
        
        # --- 3. 延时拍照 (动完了再拍) ---
        # [关键] 只有在这里才会触发 orchestrator.step()
        # 确保动作稳定后再拍 (动作下达后 +50帧)
        
        if count == 60: # 对应 10帧的动作
            piper.Capture("01_Start_Zero")
            
        elif count == 350: # 对应 300帧的动作
            piper.Capture("02_Open_Pose")
            
        elif count == 650: # 对应 600帧的动作
            piper.Capture("03_End_Zero")
            
    
    print("脚本结束。")
    simulation_app.close()