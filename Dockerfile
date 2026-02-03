FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    clang \
    clangd \
    clang-format \
    clang-tidy \
    cmake \
    cppcheck \
    ccache \
    gcc-arm-none-eabi \
    binutils-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    gdb \
    lldb \
    git \
    ninja-build \
    pkg-config \
    valgrind \
  && rm -rf /var/lib/apt/lists/*

ENV CC=gcc
ENV CXX=g++
ENV CMAKE_GENERATOR=Ninja

WORKDIR /workspace
