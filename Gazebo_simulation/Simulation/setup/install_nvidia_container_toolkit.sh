#!/bin/bash

# Exit if any command fails
set -e

echo "🔍 Detecting Ubuntu version..."
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)

echo "📥 Downloading and adding NVIDIA Docker GPG key..."
curl -fsSL https://nvidia.github.io/nvidia-docker/gpgkey | \
    sudo tee /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg > /dev/null

echo "📋 Adding NVIDIA Docker repository..."
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list > /dev/null

echo "🔄 Updating package lists..."
sudo apt-get update

echo "⬇️ Installing NVIDIA Container Toolkit..."
sudo apt-get install -y nvidia-container-toolkit

echo "♻️ Restarting Docker..."
sudo systemctl restart docker

echo "✅ NVIDIA Container Toolkit installed successfully!"

# Verify installation
echo "🛠 Checking NVIDIA GPU with Docker..."
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi

