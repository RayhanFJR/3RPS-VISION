include_directories(${CMAKE_SOURCE_DIR}/external/libmodbus/include)
target_link_libraries(server ${CMAKE_SOURCE_DIR}/external/libmodbus/lib/libmodbus.a)
