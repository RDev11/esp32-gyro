cmake_minimum_required(VERSION 3.5)
#file(GLOB SRC_LIST ./*.cpp bluetooth/*.cpp)
file(GLOB SRC_LIST ./*.cpp)

#idf_component_register(SRCS "wifi.cpp" "main.cpp" "SPI.cpp" "ili9341_font.cpp" "text2.cpp" "bluetooth/bt_test.cpp" 
#INCLUDE_DIRS ".")
idf_component_register(SRCS ${SRC_LIST} INCLUDE_DIRS ".")