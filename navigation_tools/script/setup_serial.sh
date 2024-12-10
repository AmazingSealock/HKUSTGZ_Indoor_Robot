#!/bin/bash

# 设置颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "Starting serial port permission setup..."

# USB串口设置
for i in {0..7}
do
    if [ -e "/dev/ttyUSB$i" ]; then
        sudo chmod 777 /dev/ttyUSB$i
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Successfully set permissions for /dev/ttyUSB$i${NC}"
        else
            echo -e "${RED}Failed to set permissions for /dev/ttyUSB$i${NC}"
        fi
    else
        echo -e "${RED}/dev/ttyUSB$i does not exist${NC}"
    fi
done

# ACM串口设置
if [ -e "/dev/ttyACM0" ]; then
    sudo chmod 777 /dev/ttyACM0
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Successfully set permissions for /dev/ttyACM0${NC}"
    else
        echo -e "${RED}Failed to set permissions for /dev/ttyACM0${NC}"
    fi
else
    echo -e "${RED}/dev/ttyACM0 does not exist${NC}"
fi

# 列出所有串口设备的当前权限
echo -e "\nCurrent permissions for serial ports:"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

echo -e "\nSetup complete!"