#!/bin/bash
# Docker Build Helper Script for SunFollowerFw
# This script simplifies building the firmware using Docker

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Print colored message
print_msg() {
    echo -e "${BLUE}[Docker Build]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[Success]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[Warning]${NC} $1"
}

print_error() {
    echo -e "${RED}[Error]${NC} $1"
}

# Function to build Docker image
build_image() {
    print_msg "Building Docker image..."
    docker build -t sunfollower-build:latest .
    print_success "Docker image built successfully"
}

# Function to run build in container
run_build() {
    local BUILD_TYPE="${1:-Debug}"
    
    print_msg "Building firmware (${BUILD_TYPE} mode)..."
    
    docker run --rm \
        -v "$(pwd)":/workspace \
        -w /workspace \
        sunfollower-build:latest \
        bash -c "
            mkdir -p build && cd build
            cmake -G Ninja -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
            ninja
        "
    
    print_success "Build completed successfully"
    print_msg "Binary location: build/SunFollowerFw.elf"
}

# Function to clean build artifacts
clean_build() {
    print_msg "Cleaning build artifacts..."
    rm -rf build/
    print_success "Build artifacts cleaned"
}

# Function to enter interactive shell
enter_shell() {
    print_msg "Starting interactive shell in build container..."
    docker run --rm -it \
        -v "$(pwd)":/workspace \
        -w /workspace \
        sunfollower-build:latest \
        /bin/bash
}

# Show help
show_help() {
    cat << EOF
Docker Build Helper for SunFollowerFw

Usage: $0 [COMMAND] [OPTIONS]

Commands:
    image           Build the Docker image
    build           Build the firmware (Debug mode by default)
    release         Build the firmware in Release mode
    clean           Clean build artifacts
    shell           Enter interactive shell in build container
    help            Show this help message

Examples:
    $0 image        # Build Docker image
    $0 build        # Build firmware in Debug mode
    $0 release      # Build firmware in Release mode
    $0 clean        # Clean build directory
    $0 shell        # Enter interactive shell

EOF
}

# Main script logic
case "${1:-help}" in
    image)
        build_image
        ;;
    build)
        run_build "Debug"
        ;;
    release)
        run_build "Release"
        ;;
    clean)
        clean_build
        ;;
    shell)
        enter_shell
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
