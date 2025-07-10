#!/bin/bash

# Test script to verify ROS2 package structure

echo "Testing ROS2 package structure..."
echo "================================"

# Function to check package structure
check_package() {
    local package_name=$1
    echo -e "\nChecking $package_name package:"

    # Check essential files
    files=(
        "package.xml"
        "setup.py"
        "setup.cfg"
        "resource/$package_name"
        "$package_name/__init__.py"
        "$package_name/node.py"
    )

    for file in "${files[@]}"; do
        if [ -f "src/$package_name/$file" ]; then
            echo "  ✓ $file exists"
        else
            echo "  ✗ $file missing"
        fi
    done
}

# Check each package
packages=("command_interface" "llm_agent" "perception" "mission_planner" "navigation_bridge")

for package in "${packages[@]}"; do
    check_package "$package"
done

echo -e "\n================================"
echo "Package structure test complete!"
