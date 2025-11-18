#!/bin/bash

# =========================================================
# 全局命令行代理设置脚本 (set_global_proxy.sh)
# 用途: 快速配置当前终端会话的全局环境变量代理
# 运行方式: source set_global_proxy.sh
# =========================================================

# --- 1. 定义您的代理配置 ---
# 请根据您的代理协议（http:// 或 socks5://）和地址进行修改
PROXY_ADDRESS="http://192.168.0.14:7897"
# 如果您的代理是 SOCKS5，请修改为：
# PROXY_ADDRESS="socks5://192.168.0.14:7897"

# --- 2. 设置环境变量 (export) ---
export http_proxy="${PROXY_ADDRESS}"
export https_proxy="${PROXY_ADDRESS}"
export ftp_proxy="${PROXY_ADDRESS}"

# 常用内网/本地地址不走代理 (可选)
export no_proxy="localhost,127.0.0.1,::1,10.*,192.168.*"

echo "========================================"
echo "🚀 全局命令行代理已设置，当前会话生效!"
echo "代理地址: ${PROXY_ADDRESS}"
echo "========================================"

# 验证设置
echo "HTTP Proxy: $http_proxy"
echo "No Proxy: $no_proxy"
echo "========================================"

# 提示如何让 Git 也使用代理 (如果 Git 内部配置冲突，需要先清除)
echo "💡 Git、curl、wget 将自动使用此代理。"