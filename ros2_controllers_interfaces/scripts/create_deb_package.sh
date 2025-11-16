#!/bin/bash
set -e

# Configuration
PACKAGE_NAME="ros2_controllers_interfaces"
PACKAGE_DIR="/home/jjustin/gh_ws/src/groundhog/ros2_controllers/ros2_controllers_interfaces"
OUTPUT_DIR="/home/jjustin/gh_ws"
ROS_DISTRO="jazzy"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS2 Controllers Interfaces Debian Package Builder ===${NC}"
echo "Package: $PACKAGE_NAME"
echo "Directory: $PACKAGE_DIR"
echo "Output: $OUTPUT_DIR"
echo "ROS Distro: $ROS_DISTRO"
echo ""

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check dependencies
echo -e "${YELLOW}Checking dependencies...${NC}"
for cmd in bloom-generate fakeroot dpkg-deb; do
    if ! command_exists "$cmd"; then
        echo -e "${RED}Error: $cmd is not installed${NC}"
        echo "Install with: sudo apt install python3-bloom fakeroot dpkg-dev"
        exit 1
    fi
done
echo -e "${GREEN}All dependencies found${NC}"
echo ""

# Function to build the package
build_package() {
    echo -e "${YELLOW}=== Building $PACKAGE_NAME ===${NC}"
    
    # Check if package directory exists
    if [ ! -d "$PACKAGE_DIR" ]; then
        echo -e "${RED}Error: Package directory $PACKAGE_DIR does not exist${NC}"
        return 1
    fi
    
    # Clean up previous build artifacts
    echo "Cleaning previous build artifacts..."
    rm -rf "$PACKAGE_DIR/debian"
    
    # Generate debian files using bloom
    echo "Generating debian files..."
    cd "$PACKAGE_DIR"
    
    if ! bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro "$ROS_DISTRO"; then
        echo -e "${RED}Error: bloom-generate failed${NC}"
        return 1
    fi
    
    # Build the package
    echo "Building debian package..."
    cd "$PACKAGE_DIR"
    fakeroot debian/rules binary
    
    # Find the generated .deb file (in parent directory with hyphenated name)
    search_pattern=$(echo "$PACKAGE_NAME" | tr '_' '-')
    deb_file=$(ls ../*${search_pattern}*.deb 2>/dev/null | grep -v dbgsym | head -n 1)
    
    if [ -z "$deb_file" ]; then
        echo -e "${RED}Error: Could not find generated .deb file${NC}"
        echo "Looking for pattern: ../*${search_pattern}*.deb"
        ls -la ../*.deb 2>/dev/null || echo "No .deb files found"
        return 1
    fi
    
    # Move to output directory
    echo "Moving $deb_file to $OUTPUT_DIR/"
    mv "$deb_file" "$OUTPUT_DIR/"
    
    # Get just the filename
    deb_filename=$(basename "$deb_file")
    
    echo -e "${GREEN}Successfully built: $deb_filename${NC}"
    echo ""
    
    return 0
}

# Main build process
echo -e "${YELLOW}Starting build process...${NC}"
echo ""

if build_package; then
    echo -e "${GREEN}✓ Package built successfully${NC}"
else
    echo -e "${RED}✗ Failed to build package${NC}"
    exit 1
fi

echo -e "${GREEN}=== Build Complete ===${NC}"
echo ""
echo "Generated package in $OUTPUT_DIR:"
ls -lh "$OUTPUT_DIR"/ros-${ROS_DISTRO}-*${PACKAGE_NAME}*.deb 2>/dev/null | grep -v dbgsym || echo "No package found"
echo ""
echo -e "${GREEN}To install the package:${NC}"
echo "  sudo dpkg -i $OUTPUT_DIR/ros-${ROS_DISTRO}-ros2-controllers-interfaces_*.deb"
echo "  sudo apt-get install -f -y"
