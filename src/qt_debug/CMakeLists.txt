cmake_minimum_required(VERSION 3.0.2)
project(qt_debug)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

######################################### QT 配置 ###############################################
# --- Qt5 设置 ---
# 启用 CMake 的 Qt 功能，可以自动调用 moc, uic, 和 rcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# --- 定义源文件和头文件 ---
#  qtHelper 辅助库中的文件
file(GLOB HELPER_SOURCES
  "qtHelper/src/*.cpp"
)

#  主程序源文件 (main.cpp, head.h)
file(GLOB MAIN_SOURCES
  "src/*.cpp"
)

# common 子目录中的页面管理文件 (common.cpp, .h)
file(GLOB COMMON_SOURCES
  "src/common/*.cpp"
)

#  form 子目录中的文件 (frmmain.cpp, .h, .ui)
file(GLOB FORM_SOURCES
  "src/form/*.cpp"
  "src/form/*.ui"
)

#  rocker 子目录中的文件
file(GLOB ROCKER_SOURCES
  "src/rocker/*.cpp"
)

# --- 将资源文件直接包含进来 ---
set(RES_QRC
  src/other/main.qrc
  qtHelper/qrc/font.qrc
  qtHelper/qrc/qm.qrc
  qtHelper/qrc/image.qrc
  core_qss/qss.qrc
)
################################################################################################

######################################### 后端服务 ##############################################
# 健康监测模块的源文件
file(GLOB HEALTH_SOURCES
  "src/healthMod/*.cpp"
)

################################################################################################

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation  # 自定义消息需要添加这个
)

# 查找 Qt5 包
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
  Network
)

# 自定义消息文件
add_message_files(
  FILES
  personData.msg
)

# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qt_cmd_vel
#  CATKIN_DEPENDS qt_base roscpp std_msgs
#  DEPENDS system_lib
)

# --- 创建可执行文件 ---
add_executable(${PROJECT_NAME}_node

  # qt 文件
  ${HELPER_SOURCES}
  ${MAIN_SOURCES}
  ${COMMON_SOURCES}
  ${FORM_SOURCES}
  ${ROCKER_SOURCES}
  ${RES_QRC}  

  # 后端文件
  ${HEALTH_SOURCES}
)

# -- 添加头文件包含路径 --
include_directories(
  ${catkin_INCLUDE_DIRS}

  # qt 头文件
  "${CMAKE_CURRENT_SOURCE_DIR}/qtHelper/src"    # qtHelper 辅助库头文件路径
  "${CMAKE_CURRENT_SOURCE_DIR}/src"             # 主程序头文件路径
  "${CMAKE_CURRENT_SOURCE_DIR}/src/common"      # common页面头文件路径
  "${CMAKE_CURRENT_SOURCE_DIR}/src/form"        # qt主界面头文件路径
  "${CMAKE_CURRENT_SOURCE_DIR}/src/rocker"      # rocker绘制头文件路径

  # 后端服务头文件
  "${CMAKE_CURRENT_SOURCE_DIR}/src/healthMod"   # 健康监测模块头文件路径
  "${CMAKE_CURRENT_SOURCE_DIR}/bin/include"     # 外部库头文件路径
)

# -- 添加链接库搜素路径 --
target_link_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/bin/lib
)

# 链接 Qt5 和 Catkin 库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}

  # 人体生理参数检测模块静态库
  health_monitor_cpp   # cpp 库依赖于 c 库，所以要放在前面
  health_monitor_c

  # 系统库
  pthread              # 多线程支持

  # qt 库
  Qt5::Widgets
  Qt5::Network
  # Qt5::Core, Qt5::Gui 会被 Qt5::Widgets 自动链接，可以省略
)