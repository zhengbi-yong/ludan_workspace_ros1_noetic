#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
电机自动发现和查询功能测试脚本

功能：通过USB CDC发送查询命令，检查电机自动发现功能是否正常工作

支持命令：
- 查询所有电机
- 查询指定总线的电机
- 查询指定电机的详细信息
- 触发重新扫描
- 查询ID冲突
"""

import serial
import serial.tools.list_ports
import time
import struct

# ======== 查询协议定义 ========
QUERY_CMD_HEADER = 0x7D
QUERY_RESP_HEADER = 0x7E
QUERY_CMD_LENGTH = 4

# 命令ID
CMD_QUERY_ALL_MOTORS = 0x01
CMD_QUERY_BUS_MOTORS = 0x02
CMD_QUERY_MOTOR_INFO = 0x03
CMD_TRIGGER_SCAN = 0x04
CMD_QUERY_CONFLICTS = 0x05

# 响应ID (命令ID + 0x80)
RESP_QUERY_ALL_MOTORS = 0x81
RESP_QUERY_BUS_MOTORS = 0x82
RESP_QUERY_MOTOR_INFO = 0x83
RESP_TRIGGER_SCAN = 0x84
RESP_QUERY_CONFLICTS = 0x85

# 电机状态标志
MOTOR_STATUS_ONLINE = 0x01
MOTOR_STATUS_ENABLED = 0x02
MOTOR_STATUS_CONFLICT = 0x04
MOTOR_STATUS_SCANNING = 0x08

# ======== 配置 ========
DEFAULT_BAUDRATE = 921600
TIMEOUT = 2.0

# ======== 通用函数 ========


def calculate_checksum(data):
    """计算校验和（累加和）"""
    return sum(data) & 0xFF


def make_query_command(cmd_id, param=0x00):
    """构造查询命令帧（4字节）"""
    frame = bytearray(QUERY_CMD_LENGTH)
    frame[0] = QUERY_CMD_HEADER
    frame[1] = cmd_id
    frame[2] = param
    frame[3] = calculate_checksum(frame[:3])
    return bytes(frame)


def parse_response_frame(data):
    """解析响应帧"""
    if len(data) < 4:
        return None

    if data[0] != QUERY_RESP_HEADER:
        return None

    resp_id = data[1]
    data_len = data[2]

    if len(data) < 4 + data_len:
        return None

    # 验证校验和
    calculated_checksum = calculate_checksum(data[:3 + data_len])
    if calculated_checksum != data[3 + data_len]:
        return None

    resp_data = data[3:3 + data_len]
    return (resp_id, resp_data)


def send_query_command(ser, cmd_id, param=0x00, timeout=TIMEOUT, debug=False):
    """发送查询命令并等待响应"""
    cmd_frame = make_query_command(cmd_id, param)

    print(f"TX -> CMD={cmd_id:02X}, PARAM={param:02X}, "
          f"Frame={cmd_frame.hex(' ').upper()}")

    # 清空输入缓冲区，等待一小段时间让缓冲区清空
    ser.reset_input_buffer()
    time.sleep(0.2)  # 增加等待时间，确保缓冲区完全清空

    # 发送命令
    ser.write(cmd_frame)
    ser.flush()
    time.sleep(0.05)  # 等待命令发送完成

    if debug:
        print(f"   发送后等待响应，超时={timeout}秒...")

    start_time = time.time()
    resp_buffer = bytearray()
    last_data_time = start_time

    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            new_data = ser.read(ser.in_waiting)
            resp_buffer.extend(new_data)
            last_data_time = time.time()

            if debug:
                print(
                    f"   收到 {len(new_data)} 字节数据，总缓冲区: {len(resp_buffer)} 字节")
                print(f"   原始数据: {new_data.hex(' ').upper()}")

            # 查找响应帧头（从后往前查找，找到最新的响应）
            # 先找到所有可能的响应帧头位置
            resp_positions = []
            for i in range(len(resp_buffer) - 3):
                if resp_buffer[i] == QUERY_RESP_HEADER:
                    resp_positions.append(i)

            if debug and len(resp_positions) > 0:
                print(
                    f"   找到 {len(resp_positions)} 个可能的响应帧头位置: {resp_positions}")

            # 从最新的响应开始尝试解析
            for i in reversed(resp_positions):
                # 检查是否有足够的数据
                if len(resp_buffer) >= i + 4:
                    data_len = resp_buffer[i + 2]
                    frame_len = 4 + data_len

                    if debug:
                        print(
                            f"   尝试解析响应帧，位置={i}, 数据长度={data_len}, 帧长度={frame_len}")

                    if len(resp_buffer) >= i + frame_len:
                        resp_frame = resp_buffer[i:i + frame_len]
                        result = parse_response_frame(resp_frame)
                        if result:
                            resp_id, resp_data = result
                            print(f"RX -> RESP={resp_id:02X}, LEN={len(resp_data)}, "
                                  f"Frame={resp_frame.hex(' ').upper()}")
                            return (True, resp_id, resp_data)
                        elif debug:
                            print(f"   响应帧解析失败: {resp_frame.hex(' ').upper()}")
                            # 显示校验和计算
                            if len(resp_frame) > 0:
                                calc_checksum = calculate_checksum(
                                    resp_frame[:len(resp_frame)-1])
                                print(
                                    f"   计算的校验和: {calc_checksum:02X}, 收到的校验和: {resp_frame[-1]:02X}")
                    elif debug:
                        print(
                            f"   数据不足，需要 {frame_len} 字节，当前只有 {len(resp_buffer) - i} 字节")

            # 如果缓冲区太大，清理旧数据
            if len(resp_buffer) > 512:
                resp_buffer = resp_buffer[-256:]

        # 如果超过0.5秒没有新数据，检查是否有部分数据
        if time.time() - last_data_time > 0.5 and len(resp_buffer) > 0:
            if debug:
                print(f"   长时间无新数据，当前缓冲区: {resp_buffer.hex(' ').upper()}")

        time.sleep(0.01)

    # 超时后尝试从已接收的数据中查找响应帧
    if len(resp_buffer) > 0:
        print(f"RX -> 超时，但收到 {len(resp_buffer)} 字节数据")

        # 显示数据的前64字节和后64字节（如果数据太长）
        if len(resp_buffer) > 128:
            preview = resp_buffer[:64].hex(' ').upper()
            tail = resp_buffer[-64:].hex(' ').upper()
            print(f"   数据预览（前64字节）: {preview}")
            print(f"   数据预览（后64字节）: {tail}")
        else:
            print(f"   完整数据: {resp_buffer.hex(' ').upper()}")

        # 尝试查找所有可能的响应帧头（0x7E）
        resp_positions = []
        for i in range(len(resp_buffer) - 3):
            if resp_buffer[i] == QUERY_RESP_HEADER:
                resp_positions.append(i)

        print(
            f"   发现 {len(resp_positions)} 个可能的响应帧头位置 (0x7E): {resp_positions}")

        if len(resp_positions) > 0:
            # 从最新的响应开始尝试解析
            for i in reversed(resp_positions):
                if len(resp_buffer) >= i + 4:
                    data_len = resp_buffer[i + 2]
                    frame_len = 4 + data_len

                    print(f"   尝试解析位置 {i}: 数据长度={data_len}, 帧长度={frame_len}")

                    if len(resp_buffer) >= i + frame_len:
                        resp_frame = resp_buffer[i:i + frame_len]
                        print(f"   提取的帧: {resp_frame.hex(' ').upper()}")
                        result = parse_response_frame(resp_frame)
                        if result:
                            resp_id, resp_data = result
                            print(f"RX -> RESP={resp_id:02X}, LEN={len(resp_data)}, "
                                  f"Frame={resp_frame.hex(' ').upper()}")
                            return (True, resp_id, resp_data)
                        else:
                            # 显示校验和详情
                            calc_checksum = calculate_checksum(
                                resp_frame[:len(resp_frame)-1])
                            print(
                                f"   校验和验证失败: 计算={calc_checksum:02X}, 收到={resp_frame[-1]:02X}")
                    else:
                        print(
                            f"   数据不足: 需要 {frame_len} 字节，只有 {len(resp_buffer) - i} 字节")
        else:
            # 检查是否有0x7D（命令帧头，不应该在响应中出现）
            cmd_positions = [i for i in range(
                len(resp_buffer)) if resp_buffer[i] == 0x7D]
            if len(cmd_positions) > 0:
                print(f"   警告: 发现 {len(cmd_positions)} 个命令帧头 (0x7D) 在响应数据中")
            # 检查是否有0x7B（观测数据帧头）
            obs_positions = [i for i in range(
                len(resp_buffer)) if resp_buffer[i] == 0x7B]
            if len(obs_positions) > 0:
                print(f"   信息: 发现 {len(obs_positions)} 个观测数据帧头 (0x7B)")
            print("   未发现响应帧头 (0x7E)，可能响应被观测数据淹没")
    else:
        print("RX -> 超时，未收到任何数据")

    return (False, None, None)


def parse_motor_status(status):
    """解析电机状态标志"""
    flags = []
    if status & MOTOR_STATUS_ONLINE:
        flags.append("在线")
    if status & MOTOR_STATUS_ENABLED:
        flags.append("已使能")
    if status & MOTOR_STATUS_CONFLICT:
        flags.append("ID冲突")
    if status & MOTOR_STATUS_SCANNING:
        flags.append("扫描中")
    return " | ".join(flags) if flags else "未知"


def query_all_motors(ser):
    """查询所有电机"""
    print("\n" + "=" * 60)
    print("查询所有电机")
    print("=" * 60)

    success, resp_id, data = send_query_command(ser, CMD_QUERY_ALL_MOTORS)

    if not success or resp_id != RESP_QUERY_ALL_MOTORS:
        print("✗ 查询失败")
        return False

    if len(data) < 3:
        print(f"✗ 响应数据格式错误，数据长度={len(data)}，至少需要3字节")
        print(f"  收到的数据: {data.hex(' ').upper()}")
        return False

    offset = 0

    # 解析FDCAN1电机
    bus1_count = data[offset]
    offset += 1
    print(f"\nFDCAN1总线: 发现 {bus1_count} 个电机")

    if bus1_count > 0:
        if len(data) < offset + bus1_count:
            print("✗ 数据长度不足（FDCAN1电机ID列表）")
            return False

        bus1_ids = list(data[offset:offset + bus1_count])
        offset += bus1_count
        print(f"  电机ID: {bus1_ids}")

        # 解析状态和时间戳
        if len(data) < offset + bus1_count * 5:
            print("✗ 数据长度不足（FDCAN1状态和时间戳）")
            return False

        for i, motor_id in enumerate(bus1_ids):
            status = data[offset]
            timestamp = struct.unpack('<I', data[offset + 1:offset + 5])[0]
            offset += 5

            status_str = parse_motor_status(status)
            print(f"  电机ID {motor_id}: {status_str}, 最后反馈时间戳: {timestamp}")

    # 解析FDCAN2电机
    if len(data) < offset + 1:
        print("✗ 数据长度不足（FDCAN2电机数量）")
        return False

    bus2_count = data[offset]
    offset += 1
    print(f"\nFDCAN2总线: 发现 {bus2_count} 个电机")

    if bus2_count > 0:
        if len(data) < offset + bus2_count:
            print("✗ 数据长度不足（FDCAN2电机ID列表）")
            return False

        bus2_ids = list(data[offset:offset + bus2_count])
        offset += bus2_count
        print(f"  电机ID: {bus2_ids}")

        # 解析状态和时间戳
        if len(data) < offset + bus2_count * 5:
            print("✗ 数据长度不足（FDCAN2状态和时间戳）")
            return False

        for i, motor_id in enumerate(bus2_ids):
            status = data[offset]
            timestamp = struct.unpack('<I', data[offset + 1:offset + 5])[0]
            offset += 5

            status_str = parse_motor_status(status)
            print(f"  电机ID {motor_id}: {status_str}, 最后反馈时间戳: {timestamp}")

    # 解析冲突数量
    if len(data) < offset + 1:
        print(f"✗ 数据长度不足（冲突数量），当前offset={offset}，数据长度={len(data)}")
        return False

    conflict_count = data[offset]
    if conflict_count > 0:
        print(f"\n⚠ 检测到 {conflict_count} 个ID冲突")
    else:
        print(f"\n✓ 未检测到ID冲突")

    total_motors = bus1_count + bus2_count
    print(f"\n总计: {total_motors} 个电机")

    if total_motors == 0:
        print("\n提示: 未发现任何电机。可能的原因：")
        print("  1. 电机未连接或未上电")
        print("  2. 电机未使能")
        print("  3. CAN总线通信异常")
        print("  4. 电机ID配置错误")
        print("\n建议: 尝试使用选项5触发重新扫描")

    return True


def query_bus_motors(ser, bus_id):
    """查询指定总线的电机"""
    print("\n" + "=" * 60)
    print(f"查询FDCAN{bus_id}总线的电机")
    print("=" * 60)

    success, resp_id, data = send_query_command(
        ser, CMD_QUERY_BUS_MOTORS, bus_id)

    if not success or resp_id != RESP_QUERY_BUS_MOTORS:
        print("✗ 查询失败")
        return False

    if len(data) < 1:
        print("✗ 响应数据格式错误")
        return False

    offset = 0
    count = data[offset]
    offset += 1

    print(f"发现 {count} 个电机")

    if count > 0:
        if len(data) < offset + count:
            print("✗ 数据长度不足（电机ID列表）")
            return False

        motor_ids = list(data[offset:offset + count])
        offset += count
        print(f"电机ID: {motor_ids}")

        # 解析状态和时间戳
        if len(data) < offset + count * 5:
            print("✗ 数据长度不足（状态和时间戳）")
            return False

        for motor_id in motor_ids:
            status = data[offset]
            timestamp = struct.unpack('<I', data[offset + 1:offset + 5])[0]
            offset += 5

            status_str = parse_motor_status(status)
            print(f"  电机ID {motor_id}: {status_str}, 最后反馈时间戳: {timestamp}")

    return True


def query_motor_info(ser, bus_id, motor_id):
    """查询指定电机的详细信息"""
    print("\n" + "=" * 60)
    print(f"查询FDCAN{bus_id}总线电机ID {motor_id} 的详细信息")
    print("=" * 60)

    # 参数编码: 高4位=bus_id, 低4位=motor_id
    param = ((bus_id & 0x0F) << 4) | (motor_id & 0x0F)

    success, resp_id, data = send_query_command(
        ser, CMD_QUERY_MOTOR_INFO, param)

    if not success or resp_id != RESP_QUERY_MOTOR_INFO:
        print("✗ 查询失败")
        return False

    if len(data) == 1 and data[0] == 0xFF:
        print("✗ 电机未找到")
        return False

    if len(data) < 8:
        print("✗ 响应数据格式错误")
        return False

    motor_id_resp = data[0]
    bus_id_resp = data[1]
    status = data[2]
    conflict_count = data[3]
    timestamp = struct.unpack('<I', data[4:8])[0]

    print(f"电机ID: {motor_id_resp}")
    print(f"总线ID: {bus_id_resp}")
    print(f"状态: {parse_motor_status(status)}")
    print(f"冲突计数: {conflict_count}")
    print(f"最后反馈时间戳: {timestamp}")

    return True


def trigger_scan(ser):
    """触发重新扫描"""
    print("\n" + "=" * 60)
    print("触发电机重新扫描")
    print("=" * 60)
    print("注意: 扫描需要约3-4秒，请耐心等待...")

    success, resp_id, data = send_query_command(
        ser, CMD_TRIGGER_SCAN, timeout=5.0)

    if not success or resp_id != RESP_TRIGGER_SCAN:
        print("✗ 触发扫描失败或未收到响应")
        print("  提示: 扫描可能仍在进行中，请等待几秒后查询结果")
        return False

    if len(data) >= 1 and data[0] == 0x01:
        print("✓ 扫描已触发，请等待2-3秒后查询结果")
        return True
    else:
        print("✗ 扫描触发失败")
        return False


def query_conflicts(ser):
    """查询ID冲突"""
    print("\n" + "=" * 60)
    print("查询ID冲突")
    print("=" * 60)

    success, resp_id, data = send_query_command(ser, CMD_QUERY_CONFLICTS)

    if not success or resp_id != RESP_QUERY_CONFLICTS:
        print("✗ 查询失败")
        return False

    if len(data) >= 1:
        conflict_count = data[0]
        if conflict_count > 0:
            print(f"⚠ 检测到 {conflict_count} 个ID冲突")
        else:
            print("✓ 未检测到ID冲突")
        return True

    print("✗ 响应数据格式错误")
    return False


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
    print("=" * 60)
    print("电机自动发现和查询功能测试脚本")
    print("=" * 60)
    print("\n可用命令：")
    print("  1. 查询所有电机")
    print("  2. 查询FDCAN1总线的电机")
    print("  3. 查询FDCAN2总线的电机")
    print("  4. 查询指定电机的详细信息")
    print("  5. 触发重新扫描")
    print("  6. 查询ID冲突")
    print("  7. 自动测试（执行所有查询）")
    print("  8. 调试模式（查询所有电机，带详细输出）")
    print("  0. 退出")

    # 选择串口
    port = choose_port()
    ser = serial.Serial(port, DEFAULT_BAUDRATE, timeout=0.1)
    print(f"\n已打开串口: {port}, 波特率: {DEFAULT_BAUDRATE}")

    ser.reset_input_buffer()
    time.sleep(0.1)

    try:
        while True:
            print("\n" + "-" * 60)
            choice = input("请选择命令 (0-8): ").strip()

            if choice == '0':
                print("退出程序")
                break

            elif choice == '1':
                query_all_motors(ser)

            elif choice == '8':
                # 调试模式：查询所有电机（带详细输出）
                print("\n" + "=" * 60)
                print("调试模式：查询所有电机")
                print("=" * 60)
                success, resp_id, data = send_query_command(
                    ser, CMD_QUERY_ALL_MOTORS, debug=True)
                if success:
                    print("✓ 查询成功")
                else:
                    print("✗ 查询失败")

            elif choice == '2':
                query_bus_motors(ser, 1)

            elif choice == '3':
                query_bus_motors(ser, 2)

            elif choice == '4':
                bus_id = int(input("请输入总线ID (1=FDCAN1, 2=FDCAN2): "))
                motor_id = int(input("请输入电机ID (1-16): "))
                if 1 <= bus_id <= 2 and 1 <= motor_id <= 16:
                    query_motor_info(ser, bus_id, motor_id)
                else:
                    print("无效的参数")

            elif choice == '5':
                trigger_scan(ser)

            elif choice == '6':
                query_conflicts(ser)

            elif choice == '7':
                print("\n" + "=" * 60)
                print("自动测试模式")
                print("=" * 60)

                print("\n[1/6] 查询所有电机...")
                query_all_motors(ser)
                time.sleep(0.5)

                print("\n[2/6] 查询FDCAN1总线的电机...")
                query_bus_motors(ser, 1)
                time.sleep(0.5)

                print("\n[3/6] 查询FDCAN2总线的电机...")
                query_bus_motors(ser, 2)
                time.sleep(0.5)

                print("\n[4/6] 触发重新扫描...")
                trigger_scan(ser)
                time.sleep(2.0)  # 等待扫描完成

                print("\n[5/6] 再次查询所有电机（扫描后）...")
                query_all_motors(ser)
                time.sleep(0.5)

                print("\n[6/6] 查询ID冲突...")
                query_conflicts(ser)

                print("\n" + "=" * 60)
                print("自动测试完成")
                print("=" * 60)

            else:
                print("无效的选择，请重新输入")

            # 清空输入缓冲区（丢弃所有待读取的数据）
            ser.reset_input_buffer()
            time.sleep(0.1)

            # 再次清空，确保没有残留数据
            if ser.in_waiting > 0:
                ser.read(ser.in_waiting)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        ser.close()
        print("\n串口已关闭")


if __name__ == "__main__":
    main()
