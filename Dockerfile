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
    gdb \
    lldb \
    git \
    ninja-build \
    pkg-config \
    valgrind \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user for a seamless host UID/GID mapping.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}

ENV CC=gcc
ENV CXX=g++
ENV CMAKE_GENERATOR=Ninja

WORKDIR /workspace
