import serial
import time
from typing import Optional
from dataclasses import dataclass
from enum import Enum

class SysParams(Enum):
    """系统参数枚举"""
    VERSION = 0         # 读取固件版本和对应的硬件版本
    RL = 1             # 读取读取器参数设置
    PID = 2            # 读取PID参数
    ORG = 3            # 读取回零参数
    VBUS = 4           # 读取母线电压
    CBUS = 5           # 读取母线电流
    CPHA = 6           # 读取相电流
    ENC = 7            # 读取编码器原始值
    CPUL = 8           # 读取实时脉冲计数和实时位置及设定的脉冲计数等
    ENCL = 9           # 读取编码器比例或校准后的编码器值
    TPUL = 10          # 读取脉冲计数器
    TPOS = 11          # 读取电机目标位置
    OPOS = 12          # 读取电机实时设定的目标位置（速度模式下实时位置）
    VEL = 13           # 读取电机实时转速
    CPOS = 14          # 读取电机实时位置（用于角度闭环控制累加的电机实时位置）
    PERR = 15          # 读取电机位置误差
    TEMP = 16          # 读取电机实时温度
    SFLAG = 17         # 读取状态标志位
    OFLAG = 18         # 读取错误状态标志位
    CONF = 19          # 读取配置参数
    STATE = 20         # 读取系统状态参数

class ZDTX42V2:
    """ZDT X42 V2电机控制类"""
    
    def __init__(self, port: str = 'COM7', baudrate: int = 115200):
        """
        初始化电机控制器
        
        Args:
            port: 串口号
            baudrate: 波特率
        """
        self.serial_config = {
            'port': port,
            'baudrate': baudrate,
            'bytesize': 8,
            'parity': 'N',
            'stopbits': 1,
            'timeout': 1
        }
        self.ser: Optional[serial.Serial] = None
        
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()
        
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(**self.serial_config)
            print(f"串口已打开: {self.serial_config['port']}")
        except Exception as e:
            print(f"串口打开失败: {str(e)}")
            raise
            
    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
            
    def send_command(self, cmd_bytes: bytes) -> Optional[bytes]:
        """
        发送命令并接收响应
        
        Args:
            cmd_bytes: 命令字节序列
            
        Returns:
            响应数据
        """
        try:
            if not self.ser or not self.ser.is_open:
                raise Exception("串口未打开")
                
            self.ser.write(cmd_bytes)
            print(f"已发送数据: {' '.join([f'{x:02X}' for x in cmd_bytes])}")
            
            # 等待响应
            time.sleep(0.1)
            if self.ser.in_waiting:
                response = self.ser.read(self.ser.in_waiting)
                print(f"收到响应: {' '.join([f'{x:02X}' for x in response])}")
                return response
                
        except Exception as e:
            print(f"命令发送错误: {str(e)}")
            return None
            
    def reset_position(self, addr: int):
        """
        重置当前位置为零点
        
        Args:
            addr: 电机地址
        """
        cmd = bytes([
            addr,    # 地址
            0x0A,   # 命令码
            0x6D,   # 数据码
            0x6B    # 校验字节
        ])
        self.send_command(cmd)
        
    def enable_motor(self, addr: int, enable: bool = True, sync_flag: int = 0):
        """
        电机使能控制
        
        Args:
            addr: 电机地址
            enable: True为使能，False为关闭
            sync_flag: 运动同步标志，0为不同步，其他值同步
        """
        cmd = bytes([
            addr,           # 地址
            0xF3,          # 命令码
            0xAB,          # 数据码
            int(enable),   # 使能状态
            sync_flag,     # 运动同步标志
            0x6B           # 校验字节
        ])
        self.send_command(cmd)
        
    def set_velocity(self, addr: int, direction: int, ramp: int, velocity: float, sync_flag: int = 0):
        """
        速度模式控制
        
        Args:
            addr: 电机地址
            direction: 0为CW，其他值为CCW
            ramp: 速度斜率(RPM/s)，范围0-65535
            velocity: 目标速度(RPM)，范围0.0-4000.0
            sync_flag: 运动同步标志，0为不同步，其他值同步
        """
        vel = int(abs(velocity * 10.0))  # 速度放大10倍转换
        
        cmd = bytes([
            addr,                    # 地址
            0xF6,                   # 命令码
            direction,              # 方向
            (ramp >> 8) & 0xFF,     # 速度斜率高字节
            ramp & 0xFF,            # 速度斜率低字节
            (vel >> 8) & 0xFF,      # 速度高字节
            vel & 0xFF,             # 速度低字节
            sync_flag,              # 运动同步标志
            0x6B                    # 校验字节
        ])
        self.send_command(cmd)
        
    def stop_motor(self, addr: int, sync_flag: int = 0):
        """
        停止电机运动
        
        Args:
            addr: 电机地址
            sync_flag: 运动同步标志，0为不同步，其他值同步
        """
        cmd = bytes([
            addr,       # 地址
            0xFE,      # 命令码
            0x98,      # 数据码
            sync_flag, # 运动同步标志
            0x6B       # 校验字节
        ])
        self.send_command(cmd)
        
    def read_sys_params(self, addr: int, param: SysParams):
        """
        读取系统参数
        
        Args:
            addr: 电机地址
            param: 系统参数枚举值
        """
        cmd = [addr]  # 地址
        
        # 根据参数类型设置命令码
        if param == SysParams.VERSION:
            cmd.extend([0x1F])
        elif param == SysParams.RL:
            cmd.extend([0x20])
        elif param == SysParams.PID:
            cmd.extend([0x21])
        elif param == SysParams.ORG:
            cmd.extend([0x22])
        elif param == SysParams.VBUS:
            cmd.extend([0x24])
        elif param == SysParams.CBUS:
            cmd.extend([0x26])
        elif param == SysParams.CPHA:
            cmd.extend([0x27])
        elif param == SysParams.ENC:
            cmd.extend([0x29])
        elif param == SysParams.CPUL:
            cmd.extend([0x30])
        elif param == SysParams.ENCL:
            cmd.extend([0x31])
        elif param == SysParams.TPUL:
            cmd.extend([0x32])
        elif param == SysParams.TPOS:
            cmd.extend([0x33])
        elif param == SysParams.OPOS:
            cmd.extend([0x34])
        elif param == SysParams.VEL:
            cmd.extend([0x35])
        elif param == SysParams.CPOS:
            cmd.extend([0x36])
        elif param == SysParams.PERR:
            cmd.extend([0x37])
        elif param == SysParams.TEMP:
            cmd.extend([0x39])
        elif param == SysParams.SFLAG:
            cmd.extend([0x3A])
        elif param == SysParams.OFLAG:
            cmd.extend([0x3B])
        elif param == SysParams.CONF:
            cmd.extend([0x42, 0x6C])
        elif param == SysParams.STATE:
            cmd.extend([0x43, 0x7A])
            
        # 添加校验字节
        if param in [SysParams.CONF, SysParams.STATE]:
            cmd.extend([0x6B])
            self.send_command(bytes(cmd))
        else:
            cmd.extend([0x6B])
            self.send_command(bytes(cmd))

    def modify_ctrl_mode(self, addr: int, save: bool, ctrl_mode: int):
        """
        修改开环/闭环控制模式
        
        Args:
            addr: 电机地址
            save: 是否存储标志，False为不存储，True为存储
            ctrl_mode: 控制模式，0是关闭电机所有输出，1是开环模式，2是闭环模式，
                      3将En端口改为闭环回位触发输入，Dir端口改为回位触发高电平输入
        """
        cmd = bytes([
            addr,           # 地址
            0x46,          # 命令码
            0x69,          # 数据码
            int(save),     # 是否存储标志
            ctrl_mode,     # 控制模式
            0x6B           # 校验字节
        ])
        self.send_command(cmd)

    def set_torque(self, addr: int, sign: int, t_ramp: int, torque: int, sync_flag: int = 0):
        """
        力矩模式控制
        
        Args:
            addr: 电机地址
            sign: 方向，0为正向，其他值为负
            t_ramp: 斜率(Ma/s)，范围0-65535Ma/s
            torque: 力矩(Ma)，范围0-4000Ma
            sync_flag: 运动同步标志，0为不同步，其他值同步
        """
        cmd = bytes([
            addr,                    # 地址
            0xF5,                   # 命令码
            sign,                   # 方向
            (t_ramp >> 8) & 0xFF,   # 力矩斜率高字节
            t_ramp & 0xFF,          # 力矩斜率低字节
            (torque >> 8) & 0xFF,   # 力矩高字节
            torque & 0xFF,          # 力矩低字节
            sync_flag,              # 运动同步标志
            0x6B                    # 校验字节
        ])
        self.send_command(cmd)

    def set_position_bypass(self, addr: int, direction: int, velocity: float, 
                          position: float, rel_abs_flag: int, sync_flag: int = 0):
        """
        直通控制位置模式
        
        Args:
            addr: 电机地址
            direction: 方向，0为CW，其他值为CCW
            velocity: 运行速度(RPM)，范围0.0-4000.0RPM
            position: 位置(°)，范围0.0°-(2^32-1)°
            rel_abs_flag: 相对位置/绝对位置标志，0为绝对位置，其他值为相对位置
            sync_flag: 运动同步标志，0为不同步，其他值同步
        """
        vel = int(abs(velocity * 10.0))  # 速度放大10倍
        pos = int(abs(position * 10.0))  # 位置放大10倍
        
        cmd = bytes([
            addr,                    # 地址
            0xFB,                   # 命令码
            direction,              # 方向
            (vel >> 8) & 0xFF,      # 运行速度高字节
            vel & 0xFF,             # 运行速度低字节
            (pos >> 24) & 0xFF,     # 位置(bit24-bit31)
            (pos >> 16) & 0xFF,     # 位置(bit16-bit23)
            (pos >> 8) & 0xFF,      # 位置(bit8-bit15)
            pos & 0xFF,             # 位置(bit0-bit7)
            rel_abs_flag,           # 相对/绝对位置标志
            sync_flag,              # 运动同步标志
            0x6B                    # 校验字节
        ])
        self.send_command(cmd)

    def set_position_traj(self, addr: int, direction: int, acc: int, dec: int,
                         velocity: float, position: float, rel_abs_flag: int, sync_flag: int = 0):
        """
        梯形轨迹位置模式
        
        Args:
            addr: 电机地址
            direction: 方向，0为CW，其他值为CCW
            acc: 加速加速度(RPM/s)，范围0-65535RPM/s
            dec: 减速加速度(RPM/s)，范围0-65535RPM/s
            velocity: 运行速度(RPM)，范围0.0-4000.0RPM
            position: 位置(°)，范围0.0°-(2^32-1)°
            rel_abs_flag: 相对位置/绝对位置标志，0为绝对位置，其他值为相对位置
            sync_flag: 运动同步标志，0为不同步，其他值同步
        """
        vel = int(abs(velocity * 10.0))  # 速度放大10倍
        pos = int(abs(position * 10.0))  # 位置放大10倍
        
        cmd = bytes([
            addr,                    # 地址
            0xFD,                   # 命令码
            direction,              # 方向
            (acc >> 8) & 0xFF,      # 加速加速度高字节
            acc & 0xFF,             # 加速加速度低字节
            (dec >> 8) & 0xFF,      # 减速加速度高字节
            dec & 0xFF,             # 减速加速度低字节
            (vel >> 8) & 0xFF,      # 运行速度高字节
            vel & 0xFF,             # 运行速度低字节
            (pos >> 24) & 0xFF,     # 位置(bit24-bit31)
            (pos >> 16) & 0xFF,     # 位置(bit16-bit23)
            (pos >> 8) & 0xFF,      # 位置(bit8-bit15)
            pos & 0xFF,             # 位置(bit0-bit7)
            rel_abs_flag,           # 相对/绝对位置标志
            sync_flag,              # 运动同步标志
            0x6B                    # 校验字节
        ])
        self.send_command(cmd)

    def trigger_sync_motion(self, addr: int):
        """
        触发同步运动
        
        Args:
            addr: 电机地址
        """
        cmd = bytes([
            addr,    # 地址
            0xFF,   # 命令码
            0x66,   # 数据码
            0x6B    # 校验字节
        ])
        self.send_command(cmd)

    def set_origin_point(self, addr: int, save: bool):
        """
        设置当圈零点参考位置
        
        Args:
            addr: 电机地址
            save: 是否存储标志，False为不存储，True为存储
        """
        cmd = bytes([
            addr,        # 地址
            0x93,       # 命令码
            0x88,       # 数据码
            int(save),  # 是否存储标志
            0x6B        # 校验字节
        ])
        self.send_command(cmd)

    def modify_origin_params(self, addr: int, save: bool, origin_mode: int, origin_dir: int,
                           origin_vel: int, origin_timeout: int, slow_vel: int, slow_ma: int,
                           slow_ms: int, pot_flag: bool):
        """
        修改回零参数
        
        Args:
            addr: 电机地址
            save: 是否存储标志，False为不存储，True为存储
            origin_mode: 回零模式，0为当圈和近接回零，1为当圈和碰撞回零，2为当圈和限位碰撞回零，3为当圈和限位回归零点
            origin_dir: 回零方向，0为CW，其他值为CCW
            origin_vel: 回零速度(RPM)
            origin_timeout: 回零超时时间(秒)
            slow_vel: 限位回零碰撞后的转速(RPM)
            slow_ma: 限位回零碰撞后的电流(Ma)
            slow_ms: 限位回零碰撞后的时间(Ms)
            pot_flag: 上电自动触发回零，False为不使能，True为使能
        """
        cmd = bytes([
            addr,                           # 地址
            0x4C,                          # 命令码
            0xAE,                          # 数据码
            int(save),                     # 是否存储标志
            origin_mode,                   # 回零模式
            origin_dir,                    # 回零方向
            (origin_vel >> 8) & 0xFF,      # 回零速度高字节
            origin_vel & 0xFF,             # 回零速度低字节
            (origin_timeout >> 24) & 0xFF, # 回零超时时间(bit24-bit31)
            (origin_timeout >> 16) & 0xFF, # 回零超时时间(bit16-bit23)
            (origin_timeout >> 8) & 0xFF,  # 回零超时时间(bit8-bit15)
            origin_timeout & 0xFF,         # 回零超时时间(bit0-bit7)
            (slow_vel >> 8) & 0xFF,        # 限位回零碰撞转速高字节
            slow_vel & 0xFF,               # 限���回零碰撞转速低字节
            (slow_ma >> 8) & 0xFF,         # 限位回零碰撞电流高字节
            slow_ma & 0xFF,                # 限位回零碰撞电流低字节
            (slow_ms >> 8) & 0xFF,         # 限位回零碰撞时间高字节
            slow_ms & 0xFF,                # 限位回零碰撞时间低字节
            int(pot_flag),                 # 上电自动回零标志
            0x6B                           # 校验字节
        ])
        self.send_command(cmd)

    def trigger_origin_return(self, addr: int, origin_mode: int, sync_flag: bool):
        """
        触发回零运动
        
        Args:
            addr: 电机地址
            origin_mode: 回零模式，0为当圈和近接回零，1为当圈和碰撞回零，2为当圈和限位碰撞回零，3为当圈和限位回归零点
            sync_flag: 运动同步标志，False为不同步，True为同步
        """
        cmd = bytes([
            addr,           # 地址
            0x9A,          # 命令码
            origin_mode,   # 回零模式
            int(sync_flag),# 运动同步标志
            0x6B           # 校验字节
        ])
        self.send_command(cmd)

    def interrupt_origin(self, addr: int):
        """
        强制中断并退出回零
        
        Args:
            addr: 电机地址
        """
        cmd = bytes([
            addr,    # 地址
            0x9C,   # 命令码
            0x48,   # 数据码
            0x6B    # 校验字节
        ])
        self.send_command(cmd)