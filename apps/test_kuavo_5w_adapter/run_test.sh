#!/bin/bash
# Kuavo 5-W 应用层测试 - 快速启动脚本
# 
# 用法:
#   ./run_test.sh                          # 运行所有测试
#   ./run_test.sh 01_base_control         # 运行指定模块
#   ./run_test.sh 01_base_control/test_cmd_vel_base.py  # 运行单个测试

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}Kuavo 5-W 应用层测试 - 快速启动${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""

# 检查 ROS 环境
if ! roscore > /dev/null 2>&1 &; then
    echo -e "${YELLOW}⚠️  roscore 未运行，正在启动...${NC}"
    roscore &
    sleep 2
fi

# Source ROS 环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

# 检查参数
if [ $# -eq 0 ]; then
    # 无参数：运行所有测试
    echo -e "${GREEN}📋 运行所有测试模块...${NC}"
    echo ""
    
    for module in 01_base_control 02_lower_body 03_arm_control 04_timed_commands 05_force_control 06_services 07_debug_feedback; do
        if [ -d "$module" ]; then
            echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${BLUE}运行模块: $module${NC}"
            echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            
            # 查找该模块下的所有测试文件
            test_files=$(find "$module" -name "test_*.py" -o -name "sub_*.py" | sort)
            
            if [ -z "$test_files" ]; then
                echo -e "${YELLOW}⚠️  该模块暂无测试文件${NC}"
            else
                for test_file in $test_files; do
                    echo ""
                    echo -e "${GREEN}▶ 运行测试: $test_file${NC}"
                    python3 "$test_file" || echo -e "${RED}❌ 测试失败: $test_file${NC}"
                done
            fi
            
            echo ""
        fi
    done
    
    echo -e "${BLUE}============================================================${NC}"
    echo -e "${GREEN}✅ 所有测试完成！${NC}"
    echo -e "${BLUE}============================================================${NC}"
    
elif [ -d "$1" ]; then
    # 参数是目录：运行指定模块
    MODULE="$1"
    
    if [ ! -d "$MODULE" ]; then
        echo -e "${RED}❌ 错误: 目录 '$MODULE' 不存在${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}📋 运行模块: $MODULE${NC}"
    echo ""
    
    # 查找该模块下的所有测试文件
    test_files=$(find "$MODULE" -name "test_*.py" -o -name "sub_*.py" | sort)
    
    if [ -z "$test_files" ]; then
        echo -e "${YELLOW}⚠️  该模块暂无测试文件${NC}"
        exit 0
    fi
    
    for test_file in $test_files; do
        echo -e "${GREEN}▶ 运行测试: $test_file${NC}"
        python3 "$test_file" || echo -e "${RED}❌ 测试失败: $test_file${NC}"
        echo ""
    done
    
    echo -e "${BLUE}============================================================${NC}"
    echo -e "${GREEN}✅ 模块 $MODULE 测试完成！${NC}"
    echo -e "${BLUE}============================================================${NC}"
    
elif [ -f "$1" ]; then
    # 参数是文件：运行单个测试
    TEST_FILE="$1"
    
    if [ ! -f "$TEST_FILE" ]; then
        echo -e "${RED}❌ 错误: 文件 '$TEST_FILE' 不存在${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}📋 运行单个测试: $TEST_FILE${NC}"
    echo ""
    
    python3 "$TEST_FILE"
    
    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${BLUE}============================================================${NC}"
        echo -e "${GREEN}✅ 测试通过！${NC}"
        echo -e "${BLUE}============================================================${NC}"
    else
        echo ""
        echo -e "${BLUE}============================================================${NC}"
        echo -e "${RED}❌ 测试失败！${NC}"
        echo -e "${BLUE}============================================================${NC}"
        exit 1
    fi
    
else
    echo -e "${RED}❌ 错误: 无效的参数 '$1'${NC}"
    echo ""
    echo "用法:"
    echo "  ./run_test.sh                              # 运行所有测试"
    echo "  ./run_test.sh 01_base_control             # 运行指定模块"
    echo "  ./run_test.sh 01_base_control/test_cmd_vel_base.py  # 运行单个测试"
    exit 1
fi
