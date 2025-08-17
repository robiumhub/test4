#!/bin/bash

# ROS workspace bootstrap script
# This script sets up the ROS workspace by importing dependencies and installing packages

set -e

echo "🚀 Bootstrapping ROS workspace..."

# Check if vcs is available
if ! command -v vcs &> /dev/null; then
    echo "❌ Error: vcs tool not found. Please install vcstool:"
    echo "   pip install vcstool"
    exit 1
fi

# Check if rosdep is available
if ! command -v rosdep &> /dev/null; then
    echo "❌ Error: rosdep not found. Please install rosdep:"
    echo "   sudo apt install python3-rosdep"
    echo "   sudo rosdep init"
    echo "   rosdep update"
    exit 1
fi

# Create src directory if it doesn't exist
mkdir -p src

# Import dependencies from vcs.yaml
echo "📦 Importing ROS packages from meta/core.vcs.yaml..."
if vcs import src < meta/core.vcs.yaml; then
    echo "✅ Successfully imported packages via vcs"
else
    echo "⚠️  vcs import failed, trying manual clone..."
    # Fallback: manually clone packages
    cd src
    for package in $(grep -A 2 "type: git" ../meta/core.vcs.yaml | grep "url:" | sed 's/.*url: //'); do
        package_name=$(echo $package | sed 's/.*\///' | sed 's/\.git//')
        echo "📦 Cloning $package_name from $package"
        git clone $package $package_name || echo "⚠️  Failed to clone $package_name"
    done
    cd ..
fi

# Install system dependencies
echo "🔧 Installing system dependencies..."
if rosdep install --from-paths src --ignore-src -y; then
    echo "✅ System dependencies installed successfully"
else
    echo "⚠️  Some dependencies could not be resolved via rosdep"
    echo "   This is normal for custom packages. Continuing with build..."
fi

echo "✅ ROS workspace bootstrap complete!"
echo ""
echo "Next steps:"
echo "  make build    # Build the workspace"
echo "  source install/setup.bash  # Source the workspace"
