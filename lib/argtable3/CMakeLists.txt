cmake_minimum_required(VERSION 3.18)

project(argtable3)

add_library(argtable3 argtable3.c)
target_include_directories(argtable3
  PUBLIC "."
)
