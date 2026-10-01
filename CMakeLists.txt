# Bird Sound Detection & Localization System
cmake_minimum_required(VERSION 3.16)

# 添加本地组件目录 (edge_impulse SDK + 模型)
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(COMPONENTS main)

project(i2s_es7210_sound)
