#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
保存所有电机位置零点脚本
功能：通过USB CDC发送命令，将所有电机的当前位置设置为零点
"""

import serial
import serial.tools.list_ports
import time

# ======== 命令协议定义 ========
CMD_FRAME_HEADER = 0x7C
RESP_FRAME_HEADER = 0x7D
CMD_FRAME_LENGTH = 16
RESP_FRAME_LENGTH = 8

# 命令ID
CMD_ENABLE_MOTOR = 0x01
CMD_DISABLE_MOTOR = 0x02
CMD_SAVE_ZERO = 0x03
CMD_CLEAR_ERROR = 0x04

# 操作类型
OP_TYPE_SINGLE = 0x00
OP_TYPE_ALL = 0xFF

# 状态码
STATUS_OK = 0x00
STATUS_INVALID_CMD = 0x01
STATUS_INVALID_MOTOR_ID = 0x02
STATUS_CHECKSUM_ERROR = 0x03
STATUS_EXEC_FAIL = 0x04

# ======== 配置 ========
DEFAULT_BAUDRATE = 921600
TIMEOUT = 1.0

# ======== 通用函数 ========
def calculate_checksum(data):
    """计算校验和（累加和，取低8位）"""
    return sum(data) & 0xFF

def make_command_frame(cmd_id, op_type, motor_id=0xFF):
    """构造命令帧（16字节）"""
    frame = bytearray(CMD_FRAME_LENGTH)
    frame[0] = CMD_FRAME_HEADER
    frame[1] = cmd_id
    frame[2] = op_type
    frame[3] = motor_id
    frame[15] = calculate_checksum(frame[:15])
    return bytes(frame)

def parse_response_frame(data):
    """解析响应帧（8字节）"""
    if len(data) != RESP_FRAME_LENGTH:
        return None
    
    if data[0] != RESP_FRAME_HEADER:
        return None
    
    calculated_checksum = calculate_checksum(data[:7])
    if calculated_checksum != data[7]:
        return None
    
    cmd_id = data[1]
    status = data[2]
    op_type = data[3]
    motor_id = data[4]
    
    return (cmd_id, status, op_type, motor_id)

def send_command(ser, cmd_id, op_type=OP_TYPE_ALL, motor_id=0xFF):
    """发送命令并等待响应"""
    cmd_frame = make_command_frame(cmd_id, op_type, motor_id)
    
    ser.write(cmd_frame)
    print(f"TX -> CMD={cmd_id:02X}, OP={op_type:02X}, ID={motor_id:02X}, "
          f"Frame={cmd_frame.hex(' ').upper()}")
    
    start_time = time.time()
    resp_data = bytearray()
    
    while time.time() - start_time < TIMEOUT:
        if ser.in_waiting > 0:
            resp_data.extend(ser.read(ser.in_waiting))
            if len(resp_data) >= RESP_FRAME_LENGTH:
                for i in range(len(resp_data) - RESP_FRAME_LENGTH + 1):
                    if resp_data[i] == RESP_FRAME_HEADER:
                        resp_frame = resp_data[i:i+RESP_FRAME_LENGTH]
                        result = parse_response_frame(resp_frame)
                        if result:
                            cmd_id_resp, status, op_type_resp, motor_id_resp = result
                            print(f"RX -> CMD={cmd_id_resp:02X}, STATUS={status:02X}, "
                                  f"OP={op_type_resp:02X}, ID={motor_id_resp:02X}, "
                                  f"Frame={resp_frame.hex(' ').upper()}")
                            return (status == STATUS_OK, status, result)
                resp_data = bytearray()
        time.sleep(0.01)
    
    print("RX -> 超时，未收到响应")
    return (False, None, None)

def choose_port():
    """选择串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        raise RuntimeError("未找到COM端口")
    
    print("\n可用串口：")
    for i, p in enumerate(ports):
        print(f"  {i}: {p.device} - {p.description}")
    
    while True:
        try:
            idx = int(input("\n请选择串口索引: "))
            if 0 <= idx < len(ports):
                return ports[idx].device
            else:
                print(f"无效的索引，请输入 0-{len(ports)-1} 之间的数字")
        except ValueError:
            print("请输入有效的数字")
        except KeyboardInterrupt:
            raise

# ======== 主函数 ========
def main():
    print("=" * 50)
    print("保存所有电机位置零点脚本")
    print("=" * 50)
    print("警告：此操作会将所有电机的当前位置设置为零点！")
    
    confirm = input("\n确认继续？(y/n): ")
    if confirm.lower() != 'y':
        print("操作已取消")
        return
    
    port = choose_port()
    ser = serial.Serial(port, DEFAULT_BAUDRATE, timeout=0.1)
    print(f"\n已打开串口: {port}, 波特率: {DEFAULT_BAUDRATE}")
    
    ser.reset_input_buffer()
    time.sleep(0.1)
    
    try:
        print("\n发送保存所有电机位置零点命令...")
        success, status, response = send_command(ser, CMD_SAVE_ZERO, OP_TYPE_ALL, 0xFF)
        
        if success:
            print("\n✓ 命令执行成功！所有电机位置零点已保存。")
        else:
            if status is not None:
                status_names = {
                    STATUS_INVALID_CMD: "无效命令",
                    STATUS_INVALID_MOTOR_ID: "无效电机ID",
                    STATUS_CHECKSUM_ERROR: "校验和错误",
                    STATUS_EXEC_FAIL: "执行失败"
                }
                status_name = status_names.get(status, f"未知错误(0x{status:02X})")
                print(f"\n✗ 命令执行失败: {status_name}")
            else:
                print("\n✗ 未收到有效响应")
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        ser.close()
        print("\n串口已关闭")

if __name__ == "__main__":
    main()

