#include <ros/ros.h>
#include <serial/serial.h>
#include <cmath>
#include <vector>
#include <stdint.h>

// ====== 控制范围（根据电机型号自行调整）======
#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -30.0f
#define V_MAX   30.0f
#define KP_MIN  0.0f
#define KP_MAX 500.0f
#define KD_MIN  0.0f
#define KD_MAX   5.0f
#define T_MIN   -10.0f
#define T_MAX    10.0f

// ===== 浮点数 → 整数编码 =====
uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / (x_max - x_min));
}

// ====== MIT 编码（符合 STM32 解码格式）======
void encode_mit_cmd(uint8_t* tx, uint8_t motor_id,
                    float p_des, float v_des,
                    float kp, float kd, float torque)
{
    uint16_t p_int = float_to_uint(p_des,  P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des,  V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
    uint16_t t_int  = float_to_uint(torque,T_MIN, T_MAX, 12);

    tx[0] = 0x7B;
    tx[1] = motor_id;
    tx[2] = p_int >> 8;
    tx[3] = p_int & 0xFF;
    tx[4] = v_int >> 4;
    tx[5] = ((v_int & 0x0F) << 4) | (kp_int >> 8);
    tx[6] = kp_int & 0xFF;
    tx[7] = kd_int >> 4;
    tx[8] = ((kd_int & 0x0F) << 4) | (t_int >> 8);
    tx[9] = t_int & 0xFF;

    // XOR checksum
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum ^= tx[i];
    tx[10] = sum;
}

// ====== 激活 payload ======
uint8_t activate_payload[8] =
{
    0x7F, 0xFF, 0x84, 0x30, 0x00, 0x33, 0x38, 0xCC
};

std::vector<uint8_t> make_packet(uint8_t id, const uint8_t* payload)
{
    std::vector<uint8_t> pkt(11);
    pkt[0] = 0x7B;
    pkt[1] = id;
    for (int i = 0; i < 8; i++)
        pkt[2 + i] = payload[i];

    uint8_t sum = 0;
    for (int k = 0; k < 10; k++) sum ^= pkt[k];
    pkt[10] = sum;
    return pkt;
}

// ========== 主程序（控制多个电机） ==========
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle nh;

    // -------------- 读取电机 ID 列表 --------------
    std::vector<int> motor_ids;
    nh.getParam("motor_ids", motor_ids);

    if (motor_ids.empty())
    {
        ROS_ERROR("motor_ids 参数为空，请在 launch 文件中设置 motor_ids:=[...]");
        return -1;
    }

    ROS_INFO("MIT control motor list:");
    for (int id : motor_ids)
        ROS_INFO("  motor_id = %d", id);

    // -------------- 打开串口 --------------
    serial::Serial ser;
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(921600);
    serial::Timeout to = serial::Timeout::simpleTimeout(20);
    ser.setTimeout(to);
    ser.open();

    if (!ser.isOpen())
    {
        ROS_ERROR("Failed to open serial port");
        return -1;
    }
    ROS_INFO("Serial opened OK.");

    // -------------- 激活所有电机 --------------
    ROS_INFO("Activating all motors...");

    for (int k = 0; k < 20; k++)
    {
        for (int id : motor_ids)
        {
            auto pkt = make_packet(id, activate_payload);
            ser.write(pkt);
            ros::Duration(0.003).sleep();
        }
    }

    ROS_INFO("All motors MIT mode activated.");

    ros::Rate loop(500);
    float t = 0;

    // ============= MIT 正弦运动循环 =============
    while (ros::ok())
    {
        float pos = 0.5 * sin(2 * M_PI * 0.5 * t);
        float vel = 0.5 * 2 * M_PI * 0.5 * cos(2 * M_PI * 0.5 * t);
        float kp = 20;
        float kd = 1;
        float tor = 0;

        for (int id : motor_ids)
        {
            uint8_t tx[11];
            encode_mit_cmd(tx, id, pos, vel, kp, kd, tor);
            ser.write(tx, 11);
        }

        ROS_INFO_THROTTLE(0.5, "MIT CMD : pos=%.3f vel=%.3f (sent to %lu motors)",
                          pos, vel, motor_ids.size());

        t += 0.002;
        loop.sleep();
    }

    return 0;
}
