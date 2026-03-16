cmake_minimum_required(VERSION 3.20.5)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ws2812_test)

target_sources(app PRIVATE src/test_main.c)
