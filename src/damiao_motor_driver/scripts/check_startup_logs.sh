#!/bin/bash
# 检查启动日志中的关键信息

echo "=========================================="
echo "检查启动日志中的关键信息"
echo "=========================================="

echo ""
echo "请查看启动日志中是否有以下信息:"
echo ""
echo "✓ 正常信息:"
echo "  - 'MotorDriver constructor entered !!!'"
echo "  - 'feedback_loop started'"
echo "  - 'MotorHWInterface initialized'"
echo ""
echo "✗ 错误信息:"
echo "  - 'Failed to open motor serial port after retries!'"
echo "  - 'Serial IO exception'"
echo "  - 'Serial exception'"
echo "  - 'Motor feedback timeout'"
echo ""
echo "如果看到串口打开失败的错误，请:"
echo "  1. 检查串口设备: ls -l /dev/ttyACM* /dev/ttyUSB*"
echo "  2. 检查串口权限: ls -l /dev/ttyACM0"
echo "  3. 修复权限: sudo chmod a+rw /dev/ttyACM0"
echo "  4. 检查串口是否被占用: lsof /dev/ttyACM0"
echo ""
echo "=========================================="

