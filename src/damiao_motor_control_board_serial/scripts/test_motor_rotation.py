import serial
import serial.tools.list_ports
import time
import functools

# ======== 基础配置 ========
FRAME_HEADER = 0x7B
CHECKSUM_MODE = "xor"  # "xor" 或 "sum"
PRINT_INTERVAL = 0.5   # 每隔多少秒打印一次
SEND_INTERVAL = 0.01   # 发送间隔（s）
DEFAULT_BAUDRATE = 921600

MOTOR_NUM = 16
# ======== 电机配置 ========
MOTOR_CONFIGS = {
    "damiao4310": {
        # "ids": [0, 1,2,3,4,7,8,9,10,11,12,13,14,15,MOTOR_NUM+0],   # 多个同型号电机 ID
        # "ids": [0,MOTOR_NUM+0, MOTOR_NUM+1,MOTOR_NUM+2,MOTOR_NUM+3,MOTOR_NUM+4,MOTOR_NUM+7,MOTOR_NUM+8,MOTOR_NUM+9,MOTOR_NUM+10,MOTOR_NUM+11,MOTOR_NUM+12,MOTOR_NUM+13,MOTOR_NUM+14,MOTOR_NUM+15],   # 多个同型号电机 ID
        "ids": [0, 4],   # 多个同型号电机 ID
        "payload": [0x7F, 0xFF, 0x84, 0x30, 0x00, 0x33, 0x38, 0xCC],
    },
    "damiao6248p": {
        # "ids": [5,6],
        "ids": [MOTOR_NUM+5,MOTOR_NUM+6],
        "payload": [0x7F, 0xFF, 0x86, 0x50, 0x00, 0x33, 0x38, 0x10],
    },
    # 可以继续添加新型号
    # "your_motor_name": {"ids": [X, Y], "payload": [8 bytes]},
}


# ======== 通用函数 ========
def checksum(bytes_like):
    if CHECKSUM_MODE == "xor":
        return functools.reduce(lambda a, b: a ^ b, bytes_like)
    elif CHECKSUM_MODE == "sum":
        return sum(bytes_like) & 0xFF
    else:
        raise ValueError("Unknown CHECKSUM_MODE")


def make_packet(motor_id, payload8):
    if len(payload8) != 8:
        raise ValueError("payload must be 8 bytes")
    pkt10 = bytearray(10)
    pkt10[0] = FRAME_HEADER
    pkt10[1] = motor_id
    pkt10[2:10] = payload8
    cks = checksum(pkt10)
    return pkt10 + bytes([cks])


def choose_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        raise RuntimeError("No COM ports found")
    for i, p in enumerate(ports):
        print(f"{i}: {p.device} - {p.description}")
    idx = int(input("Select index: "))
    return ports[idx].device


# ======== 主逻辑 ========
def send_packet(ser, name, motor_id, payload):
    pkt = make_packet(motor_id, payload)
    ser.write(pkt)
    print(f"[{name}] TX -> id={motor_id}, bytes={pkt.hex(' ')}")


def test_model(ser, name, ids, payload, interval=SEND_INTERVAL):
    print(f"开始测试型号: {name}, IDs={ids}")
    t0 = time.time()
    try:
        while True:
            for mid in ids:
                send_packet(ser, name, mid, payload)
                time.sleep(interval)
            if time.time() - t0 > PRINT_INTERVAL:
                print(f"[{name}] 已发送至 {len(ids)} 个电机")
                t0 = time.time()
    except KeyboardInterrupt:
        print(f"停止 {name} 电机测试。")


def main():
    print("=== 多型号多电机测试脚本 ===")
    port = choose_port()
    ser = serial.Serial(port, DEFAULT_BAUDRATE, timeout=0)
    print(f"已打开串口: {port}")

    # ===== 模式选择 =====
    single_model_mode = False   # 测试单个型号（同型号多个ID）
    multi_mode = True       # 测试所有型号的所有电机

    if single_model_mode:
        motor_name = "damiao4310"
        cfg = MOTOR_CONFIGS[motor_name]
        test_model(ser, motor_name, cfg["ids"], cfg["payload"])

    elif multi_mode:
        print("开始批量测试所有型号所有电机：")
        try:
            while True:
                for name, cfg in MOTOR_CONFIGS.items():
                    for mid in cfg["ids"]:
                        send_packet(ser, name, mid, cfg["payload"])
                        time.sleep(SEND_INTERVAL)
        except KeyboardInterrupt:
            print("批量测试结束。")

    ser.close()
    print("串口已关闭。")


if __name__ == "__main__":
    main()