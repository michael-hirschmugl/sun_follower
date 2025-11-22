# Dockerfile for STM32 Firmware Build Environment
# This provides a portable build environment for the SunFollowerFw project

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    wget \
    xz-utils \
    git \
    make \
    ninja-build \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install CMake (version 3.27.7)
RUN wget -q https://github.com/Kitware/CMake/releases/download/v3.27.7/cmake-3.27.7-linux-x86_64.sh \
    && chmod +x cmake-3.27.7-linux-x86_64.sh \
    && ./cmake-3.27.7-linux-x86_64.sh --skip-license --prefix=/usr/local \
    && rm cmake-3.27.7-linux-x86_64.sh

# Install ARM GNU Toolchain (13.2.Rel1)
RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz \
    && tar -xf arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz -C /opt \
    && rm arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz

# Add ARM toolchain to PATH
ENV PATH="/opt/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin:${PATH}"

# Verify installations
RUN echo "=== Verifying tool installations ===" \
    && arm-none-eabi-gcc --version \
    && echo "\n" \
    && cmake --version \
    && echo "\n" \
    && ninja --version

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
