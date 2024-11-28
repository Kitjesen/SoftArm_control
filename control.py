from motor_control import ZDTX42V2, SysParams
import time

class MotorController:
    def __init__(self, port='COM7', baudrate=115200):
        self.motor = ZDTX42V2(port=port, baudrate=baudrate)
        self.addr = 3  # 默认电机地址为1
        
    def initialize(self):
        """初始化电机"""
        try:
            # 连接电机
            self.motor.connect()
            
            # 读取电机版本信息
            self.motor.read_sys_params(self.addr, SysParams.VERSION)
            time.sleep(0.1)  # 等待响应
            
            # 设置为闭环控制模式
            self.motor.modify_ctrl_mode(self.addr, save=True, ctrl_mode=2)
            time.sleep(0.1)  # 等待模式切换完成
            
            # 使能电机
            self.motor.enable_motor(self.addr, enable=True)
            time.sleep(0.1)  # 等待使能完成
            
            print("电机初始化完成 ✓")
            return True
            
        except Exception as e:
            print(f"初始化失败: {str(e)} ❌")
            return False

    def close(self):
        """关闭电机"""
        try:
            # 停止电机
            self.motor.stop_motor(self.addr)
            # 关闭使能
            self.motor.enable_motor(self.addr, enable=False)
            # 断开连接
            self.motor.disconnect()
            print("电机已安全关闭 ✓")
        except Exception as e:
            print(f"关闭失败: {str(e)} ❌")

    def run_velocity_mode(self, velocity: float, direction: int = 0, duration: float = 2.0):
        """
        运行速度模式
        
        Args:
            velocity: 目标速度(RPM)，范围0.0-4000.0
            direction: 0为顺时针，1为逆时针
            duration: 运行时间(秒)
        """
        try:
            print(f"开始速度模式控制: {velocity} RPM, {'逆时针' if direction else '顺时针'}")
            
            # 设置速度
            self.motor.set_velocity(
                addr=self.addr,
                direction=direction,
                ramp=1000,  # 加速度1000RPM/s
                velocity=velocity
            )
            
            # 运行指定时间
            time.sleep(duration)
            
            # 停止电机
            self.motor.stop_motor(self.addr)
            print("速度模式运行完成 ✓")
            
        except Exception as e:
            print(f"速度模式运行失败: {str(e)} ❌")
            self.motor.stop_motor(self.addr)  # 确保电机停止

    def run_position_mode(self, position: float, velocity: float = 1000.0, 
                         direction: int = 0, is_absolute: bool = True):
        """
        运行位置模式
        
        Args:
            position: 目标位置(度)
            velocity: 运行速度(RPM)，默认1000RPM
            direction: 0为顺时针，1为逆时针
            is_absolute: True为绝对位置，False为相对位置
        """
        try:
            print(f"开始位置模式控制: {position}°, {velocity} RPM")
            
            # 使用梯形轨迹位置模式
            self.motor.set_position_traj(
                addr=self.addr,
                direction=direction,
                acc=1000,  # 加速度1000RPM/s
                dec=1000,  # 减速度1000RPM/s
                velocity=velocity,
                position=position,
                rel_abs_flag=0 if is_absolute else 1
            )
            
            # 等待运动完成（这里可以添加位置到达检测）
            time.sleep(5)
            print("位置模式运行完成 ✓")
            
        except Exception as e:
            print(f"位置模式运行失败: {str(e)} ❌")
            self.motor.stop_motor(self.addr)

    def run_torque_mode(self, torque: int, direction: int = 0, duration: float = 2.0):
        """
        运行力矩模式
        
        Args:
            torque: 力矩大小(mA)，范围0-4000
            direction: 0为正向，1为反向
            duration: 运行时间(秒)
        """
        try:
            print(f"开始力矩模式控制: {torque} mA")
            
            # 设置力矩
            self.motor.set_torque(
                addr=self.addr,
                sign=direction,
                t_ramp=1000,  # 力矩斜率1000mA/s
                torque=torque
            )
            
            # 运行指定时间
            time.sleep(duration)
            
            # 停止电机
            self.motor.stop_motor(self.addr)
            print("力矩模式运行完成 ✓")
            
        except Exception as e:
            print(f"力矩模式运行失败: {str(e)} ❌")
            self.motor.stop_motor(self.addr)

def main():
    # 创建控制器实例
    controller = MotorController(port='COM7')
    
    try:
        # 初始化，如果失败则直接返回
        if not controller.initialize():
            return
        
        # 等待系统稳定
        time.sleep(1)
        
        # 运行示例: 速度模式测试
        print("开始速度模式测试...")
        controller.run_velocity_mode(velocity=1000.0, direction=1, duration=1.0)
        time.sleep(1)  # 等待电机完全停止
        
    except Exception as e:
        print(f"程序运行错误: {str(e)} ❌")
    finally:
        # 确保正确关闭电机
        controller.close()

if __name__ == "__main__":
    main()
