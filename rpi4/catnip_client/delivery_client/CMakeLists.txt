cmake_minimum_required(VERSION 3.16)
project(DeliveryOrderSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt6 자동 처리 설정
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Qt6 찾기
find_package(Qt6 COMPONENTS
    Core
    Gui
    Widgets
    REQUIRED
)

# 인클루드 디렉토리 설정
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 실행 파일 생성
add_executable(delivery_app
    src/main.cpp
    include/mainwindow.h
    src/mainwindow.cpp
    include/mqtt_client.h
    src/mqtt_client.cpp
    include/monitorwindow.h
    src/monitorwindow.cpp
    include/numpaddialog.h
    src/numpaddialog.cpp
)

# 링크 라이브러리
target_link_libraries(delivery_app
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    /usr/local/lib/libpaho-mqttpp3.so
    /usr/local/lib/libpaho-mqtt3as.so
)

# 컴파일 옵션
target_compile_options(delivery_app PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)