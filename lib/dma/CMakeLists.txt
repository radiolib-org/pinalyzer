cmake_minimum_required(VERSION 3.18)

# from https://github.com/fandahao17/Raspberry-Pi-DMA-Tutorial

project(dma)

add_library(dma mailbox.c dma.c)
target_include_directories(dma
  PUBLIC "."
)
