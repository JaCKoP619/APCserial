cmake_minimum_required(VERSION 3.30)
project(APCserial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Boost 1.87 COMPONENTS asio)
add_executable (file apcserial.cpp classes.cpp)

target_include_directories(file PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(file PRIVATE  Boost::asio ws2_32)