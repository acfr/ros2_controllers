#!/bin/bash
set -e

# Configuration
PACKAGE_NAME="swerve_controller"
PACKAGE_DIR="/home/jjustin/gh_ws/src/groundhog/ros2_controllers/swerve_controller"
OUTPUT_DIR="/home/jjustin/gh_ws"
ROS_DISTRO="jazzy"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Swerve Controller Debian Package Builder ===${NC}"
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

# Clean up previous build artifacts
echo -e "${YELLOW}Cleaning previous build artifacts...${NC}"
cd "$PACKAGE_DIR"
rm -rf debian

# Generate debian files using bloom
echo -e "${YELLOW}Generating debian files...${NC}"
if ! bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro "$ROS_DISTRO"; then
    echo -e "${RED}Error: bloom-generate failed${NC}"
    exit 1
fi

# Build the package
echo -e "${YELLOW}Building debian package...${NC}"
fakeroot debian/rules binary

# Find the generated .deb file
search_pattern=$(echo "$PACKAGE_NAME" | tr '_' '-')
deb_file=$(ls ../*${search_pattern}*.deb 2>/dev/null | grep -v dbgsym | head -n 1)

if [ -z "$deb_file" ]; then
    echo -e "${RED}Error: Could not find generated .deb file${NC}"
    echo "Looking for pattern: ../*${search_pattern}*.deb"
    ls -la ../*.deb 2>/dev/null || echo "No .deb files found"
    exit 1
fi

# Move to output directory
echo "Moving $(basename "$deb_file") to $OUTPUT_DIR/"
mv "$deb_file" "$OUTPUT_DIR/"

# Get the final filename
deb_filename=$(basename "$deb_file")

echo ""
echo -e "${GREEN}=== Build Complete ===${NC}"
echo ""
echo "Generated package:"
ls -lh "$OUTPUT_DIR/$deb_filename"
echo ""
echo -e "${GREEN}Package built successfully!${NC}"
echo ""
echo "To install locally:"
echo "  sudo dpkg -i $OUTPUT_DIR/$deb_filename"
echo "  sudo apt-get install -f -y"
