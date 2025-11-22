#include <ros/ros.h>
#include <serial/serial.h>
#include <cmath>
#include <vector>
#include <stdint.h>

// ====== 控制范围（根据电机类型自行调整）======
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

// ===== 映射函数 =====
uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float value = (x - offset) * ((float)((1 << bits) - 1)) / span;
    if (value < 0) value = 0;
    if (value > (1 << bits) - 1) value = (1 << bits) - 1;
    return (uint16_t)value;
}

// ====== MIT 编码函数（匹配 STM32 解码格式）======
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

    // XOR 校验
    uint8_t sum = 0;
    for (int i=0; i<10; i++) sum ^= tx[i];
    tx[10] = sum;
}

// ====== 固定 payload（进入 MIT 模式）======
uint8_t activate_payload[8] =
{
    0x7F, 0xFF, 0x84, 0x30, 0x00, 0x33, 0x38, 0xCC
};

std::vector<uint8_t> make_packet(uint8_t id, const uint8_t* payload8)
{
    std::vector<uint8_t> pkt(11);
    pkt[0] = 0x7B;
    pkt[1] = id;
    for(int i=0; i<8; i++)
        pkt[2+i] = payload8[i];

    // XOR 校验
    uint8_t sum = 0;
    for (int k=0; k<10; k++) sum ^= pkt[k];
    pkt[10] = sum;
    return pkt;
}

// ========== 主程序 =============
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_single_motor");
    ros::NodeHandle nh;

    int motor_id = 0;
    nh.param("motor_id", motor_id, 0);

    serial::Serial ser;
    ser.setPort("/dev/mcu");
    ser.setBaudrate(921600);
    serial::Timeout to = serial::Timeout::simpleTimeout(20);
    ser.setTimeout(to);
    ser.open();

    if (!ser.isOpen())
    {
        ROS_ERROR("Serial open failed");
        return -1;
    }

    ROS_INFO("Serial opened OK. Activating motor...");

    // 激活电机模式
    for (int i=0; i<20; i++)
    {
        auto pkt = make_packet(motor_id, activate_payload);
        ser.write(pkt);
        ros::Duration(0.01).sleep();
    }

    ROS_INFO("MIT mode activated.");

    ros::Rate loop(500);
    float t = 0;

    while (ros::ok())
    {
        float pos = 1.0 * sin(2 * M_PI * 0.5 * t);
        float vel = 0.3 * 2 * M_PI * 0.5 * cos(2 * M_PI * 0.5 * t);

        float kp = 20.0;
        float kd = 1.0;
        float tor = 0.0;

        uint8_t tx[11];
        encode_mit_cmd(tx, motor_id, pos, vel, kp, kd, tor);
        ser.write(tx, 11);

        ROS_INFO("MIT CMD → id=%d  p=%.3f  v=%.3f", motor_id, pos, vel);

        t += 0.002;
        loop.sleep();
    }

    return 0;
}
