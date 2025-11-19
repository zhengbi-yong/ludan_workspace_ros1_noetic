#!/bin/bash

# =========================================================
# 全局命令行代理清除脚本 (unset_global_proxy.sh)
# 用途: 快速清除当前终端会话中的代理环境变量
# 运行方式: source unset_global_proxy.sh
# =========================================================

echo "========================================"
echo "🗑️ 正在清除全局命令行代理配置..."
echo "========================================"

# 清除环境变量 (unset)
unset http_proxy
unset https_proxy
unset ftp_proxy
unset no_proxy

# 验证清除
echo "✅ 代理清除完成，当前代理变量状态:"
echo "HTTP Proxy: $http_proxy (应为空)"
echo "HTTPS Proxy: $https_proxy (应为空)"
echo "========================================"