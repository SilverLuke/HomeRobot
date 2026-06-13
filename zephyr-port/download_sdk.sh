#!/usr/bin/env bash
set -e

# Prompt for the SDK version
read -p "Enter the Zephyr SDK version you want to download (e.g. 0.16.5-1): " SDK_VERSION

if [ -z "$SDK_VERSION" ]; then
    echo "Error: Version cannot be empty."
    exit 1
fi

TARGET_DIR="/home/luca/Projects/robotica/homeRobot/software/zephyr-port/zephyr-sdk-${SDK_VERSION}"

# The standard format for Zephyr SDK tarballs on Linux
TAR_FILE="zephyr-sdk-${SDK_VERSION}_linux-x86_64.tar.xz"
SDK_URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}/${TAR_FILE}"

echo "Setting up Zephyr SDK $SDK_VERSION at $TARGET_DIR"
echo "Downloading from $SDK_URL..."

mkdir -p "$TARGET_DIR"

wget -O "/tmp/$TAR_FILE" "$SDK_URL"

echo "Extracting..."
tar -xf "/tmp/$TAR_FILE" -C "$TARGET_DIR" --strip-components=1

# Zephyr SDK usually requires running setup.sh afterwards
if [ -f "$TARGET_DIR/setup.sh" ]; then
    echo "Running Zephyr SDK setup..."
    cd "$TARGET_DIR"
    ./setup.sh -c -h -t all
fi

rm -f "/tmp/$TAR_FILE"

echo "Done! The SDK is available at $TARGET_DIR"
