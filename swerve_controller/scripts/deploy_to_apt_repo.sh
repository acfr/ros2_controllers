#!/bin/bash
set -e

# Configuration
PACKAGE_NAME="swerve_controller"
OUTPUT_DIR="/home/jjustin/gh_ws"
REMOTE_USER="jjustin"
REMOTE_HOST="avocado.acfr.usyd.edu.au"
REMOTE_REPO_PATH="/data/www/EHM/datasets/ubuntu-repo"
REMOTE_POOL_PATH="$REMOTE_REPO_PATH/pool/main"
REMOTE_DISTS_PATH="$REMOTE_REPO_PATH/dists/noble/main/binary-amd64"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Swerve Controller Deployment to ACFR APT Repository ===${NC}"
echo "Package: $PACKAGE_NAME"
echo "Remote: $REMOTE_USER@$REMOTE_HOST"
echo "Repository: $REMOTE_REPO_PATH"
echo ""

# Find the .deb file
search_pattern=$(echo "$PACKAGE_NAME" | tr '_' '-')
deb_file=$(ls "$OUTPUT_DIR"/ros-jazzy-${search_pattern}_*.deb 2>/dev/null | grep -v dbgsym | head -n 1)

if [ -z "$deb_file" ]; then
    echo -e "${RED}Error: Could not find .deb file${NC}"
    echo "Looking for: $OUTPUT_DIR/ros-jazzy-${search_pattern}_*.deb"
    exit 1
fi

deb_filename=$(basename "$deb_file")
echo -e "${GREEN}Found package: $deb_filename${NC}"
echo "Size: $(ls -lh "$deb_file" | awk '{print $5}')"
echo ""

# Upload the .deb file
echo -e "${YELLOW}Uploading $deb_filename to remote repository...${NC}"
scp "$deb_file" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_POOL_PATH/"

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to upload .deb file${NC}"
    exit 1
fi
echo -e "${GREEN}Upload complete${NC}"
echo ""

# Generate repository metadata on the remote server
echo -e "${YELLOW}Regenerating repository metadata...${NC}"
ssh "$REMOTE_USER@$REMOTE_HOST" << 'ENDSSH'
set -e

REPO_PATH="/data/www/EHM/datasets/ubuntu-repo"
POOL_PATH="$REPO_PATH/pool/main"
DISTS_PATH="$REPO_PATH/dists/noble"
BINARY_PATH="$DISTS_PATH/main/binary-amd64"

echo "Working in repository: $REPO_PATH"

# Create necessary directories if they don't exist
mkdir -p "$BINARY_PATH"

# Change to repository root
cd "$REPO_PATH"

# Generate Packages file
echo "Generating Packages file..."
dpkg-scanpackages --multiversion pool/main > "$BINARY_PATH/Packages"

# Compress Packages file
echo "Compressing Packages file..."
gzip -9fc "$BINARY_PATH/Packages" > "$BINARY_PATH/Packages.gz"

# Generate Release file with proper checksums
echo "Generating Release file..."
RELEASE_FILE="$DISTS_PATH/Release"

# Create Release file header
cat > "$RELEASE_FILE" << EOF
Origin: ACFR
Label: ACFR Ubuntu Repository
Suite: noble
Codename: noble
Architectures: amd64
Components: main
Description: ACFR ROS 2 Jazzy packages for Ubuntu 24.04
Date: $(LC_ALL=C date -u '+%a, %d %b %Y %H:%M:%S +0000')
EOF

# Function to calculate checksums (RHEL-compatible)
cd "$DISTS_PATH"

echo "MD5Sum:" >> "$RELEASE_FILE"
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        MD5=$(md5sum "$file" | awk '{print $1}')
        SIZE=$(stat --format=%s "$file")
        printf " %s %7d %s\n" "$MD5" "$SIZE" "$file" >> "$RELEASE_FILE"
    fi
done

echo "SHA1:" >> "$RELEASE_FILE"
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        SHA1=$(sha1sum "$file" | awk '{print $1}')
        SIZE=$(stat --format=%s "$file")
        printf " %s %7d %s\n" "$SHA1" "$SIZE" "$file" >> "$RELEASE_FILE"
    fi
done

echo "SHA256:" >> "$RELEASE_FILE"
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        SHA256=$(sha256sum "$file" | awk '{print $1}')
        SIZE=$(stat --format=%s "$file")
        printf " %s %7d %s\n" "$SHA256" "$SIZE" "$file" >> "$RELEASE_FILE"
    fi
done

echo ""
echo "Repository metadata regenerated successfully"
echo ""
echo "Repository contents:"
ls -lh "$POOL_PATH"/*.deb 2>/dev/null || echo "No .deb files found"
echo ""
echo "Metadata files:"
ls -lh "$BINARY_PATH"/Packages*
echo ""
echo "Release file:"
cat "$RELEASE_FILE"
ENDSSH

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to regenerate repository metadata${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}=== Deployment Complete ===${NC}"
echo ""
echo "Package deployed: $deb_filename"
echo "Repository URL: https://data.acfr.usyd.edu.au/ubuntu-repo/"
echo ""
echo "To install on a client machine:"
echo "  1. Ensure repository is configured in /etc/apt/sources.list.d/acfr.list"
echo "  2. Run: sudo apt update"
echo "  3. Run: sudo apt install ros-jazzy-${search_pattern}"
echo ""
echo -e "${YELLOW}Note: Client may need to clear cache:${NC}"
echo "  sudo rm /var/lib/apt/lists/data.acfr.*"
echo "  sudo apt update"
