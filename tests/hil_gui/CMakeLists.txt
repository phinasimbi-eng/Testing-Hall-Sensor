cmake_minimum_required(VERSION 3.16)
project(MotorTestGUI VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts SerialPort Network)

# Source files
set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/motor_simulator.cpp
    src/debugger_interface.cpp
    src/uart_monitor.cpp
    src/plot_widget.cpp
    src/test_sequence.cpp
)

set(HEADERS
    src/mainwindow.h
    src/motor_simulator.h
    src/debugger_interface.h
    src/uart_monitor.h
    src/plot_widget.h
    src/test_sequence.h
)

set(UI_FILES
    ui/mainwindow.ui
)

# Executable
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${UI_FILES}
)

# Link Qt libraries
target_link_libraries(${PROJECT_NAME}
    Qt6::Core
    Qt6::Widgets
    Qt6::Charts
    Qt6::SerialPort
    Qt6::Network
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Install
install(TARGETS ${PROJECT_NAME}
    BUNDLE DESTINATION .
    RUNTIME DESTINATION bin
)
