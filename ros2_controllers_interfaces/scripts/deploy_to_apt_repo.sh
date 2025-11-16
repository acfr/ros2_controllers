#!/bin/bash
set -e

# Configuration
PACKAGE_PREFIX="ros2-controllers-interfaces"
LOCAL_DEB_DIR="/home/jjustin/gh_ws"
REMOTE_SERVER="avocado.acfr.usyd.edu.au"
REMOTE_USER="jjustin"
REMOTE_REPO_DIR="/data/www/EHM/datasets/ubuntu-repo"
ROS_DISTRO="jazzy"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS2 Controllers Interfaces Package Deployment Script ===${NC}"
echo "Package Prefix: $PACKAGE_PREFIX"
echo "Local Directory: $LOCAL_DEB_DIR"
echo "Remote Server: $REMOTE_SERVER"
echo "Remote Repository: $REMOTE_REPO_DIR"
echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Function to check if SSH connection works
check_ssh() {
    echo -e "${YELLOW}Checking SSH connection to $REMOTE_SERVER...${NC}"
    if ssh -o ConnectTimeout=5 "$REMOTE_USER@$REMOTE_SERVER" "echo 'SSH connection successful'" >/dev/null 2>&1; then
        echo -e "${GREEN}✓ SSH connection successful${NC}"
        return 0
    else
        echo -e "${RED}✗ SSH connection failed${NC}"
        echo "Please ensure:"
        echo "  1. SSH key is set up for $REMOTE_USER@$REMOTE_SERVER"
        echo "  2. You have access to the server"
        echo "  3. Server is reachable from your network"
        return 1
    fi
}

# Function to find .deb files
find_deb_files() {
    echo -e "${YELLOW}Finding .deb files...${NC}"
    
    # Find all ros2-controllers-interfaces-related .deb files (excluding debug symbols)
    DEB_FILES=($(ls "$LOCAL_DEB_DIR"/ros-${ROS_DISTRO}-*${PACKAGE_PREFIX}*.deb 2>/dev/null | grep -v dbgsym))
    
    if [ ${#DEB_FILES[@]} -eq 0 ]; then
        echo -e "${RED}Error: No .deb files found in $LOCAL_DEB_DIR${NC}"
        echo "Expected pattern: ros-${ROS_DISTRO}-*${PACKAGE_PREFIX}*.deb"
        echo ""
        echo "Please run the build script first:"
        echo "  ./scripts/create_deb_package.sh"
        return 1
    fi
    
    echo -e "${GREEN}Found ${#DEB_FILES[@]} package(s):${NC}"
    for deb in "${DEB_FILES[@]}"; do
        local size=$(ls -lh "$deb" | awk '{print $5}')
        echo "  - $(basename "$deb") ($size)"
    done
    echo ""
    
    return 0
}

# Function to upload files
upload_files() {
    echo -e "${YELLOW}Uploading .deb files to $REMOTE_SERVER...${NC}"
    
    for deb_file in "${DEB_FILES[@]}"; do
        local filename=$(basename "$deb_file")
        echo "Uploading $filename..."
        
        if scp "$deb_file" "$REMOTE_USER@$REMOTE_SERVER:$REMOTE_REPO_DIR/pool/main/"; then
            echo -e "${GREEN}✓ Uploaded $filename${NC}"
        else
            echo -e "${RED}✗ Failed to upload $filename${NC}"
            return 1
        fi
    done
    
    echo ""
    return 0
}

# Function to regenerate repository metadata
regenerate_metadata() {
    echo -e "${YELLOW}Regenerating repository metadata...${NC}"
    
    # Create a script to run on the remote server (RHEL-compatible)
    local remote_script=$(cat << 'EOF'
#!/bin/bash
set -e

REPO_DIR="/data/www/EHM/datasets/ubuntu-repo"
cd "$REPO_DIR"

echo "Current directory: $(pwd)"
echo ""

# Generate Packages file
echo "Generating Packages file..."
dpkg-scanpackages -m pool/main /dev/null > dists/noble/main/binary-amd64/Packages

# Compress Packages file
echo "Compressing Packages file..."
gzip -9c dists/noble/main/binary-amd64/Packages > dists/noble/main/binary-amd64/Packages.gz

# Generate Release file with proper date format (RHEL-compatible)
echo "Generating Release file..."
cd dists/noble
cat > Release << EOL
Origin: ACFR
Label: ACFR Ubuntu Repository
Suite: noble
Codename: noble
Architectures: amd64
Components: main
Description: ACFR Ubuntu Repository for ROS 2 Jazzy
Date: $(LANG=C date -u '+%a, %d %b %Y %H:%M:%S +0000')
EOL

# Add file hashes to Release (RHEL-compatible: use stat --format instead of -c)
echo "MD5Sum:" >> Release
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        printf " %s %s %s\n" "$(md5sum $file | cut -d' ' -f1)" "$(stat --format=%s $file)" "$file" >> Release
    fi
done

echo "SHA1:" >> Release
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        printf " %s %s %s\n" "$(sha1sum $file | cut -d' ' -f1)" "$(stat --format=%s $file)" "$file" >> Release
    fi
done

echo "SHA256:" >> Release
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        printf " %s %s %s\n" "$(sha256sum $file | cut -d' ' -f1)" "$(stat --format=%s $file)" "$file" >> Release
    fi
done

echo ""
echo "Repository metadata regenerated successfully!"
echo ""
echo "Package count:"
grep -c "^Package:" main/binary-amd64/Packages || echo "0"
EOF
)
    
    # Execute the script on the remote server
    if ssh "$REMOTE_USER@$REMOTE_SERVER" "bash -s" <<< "$remote_script"; then
        echo -e "${GREEN}✓ Repository metadata regenerated${NC}"
        return 0
    else
        echo -e "${RED}✗ Failed to regenerate repository metadata${NC}"
        return 1
    fi
}

# Function to verify installation
verify_installation() {
    echo ""
    echo -e "${YELLOW}Verifying repository access...${NC}"
    
    echo "Checking if packages are in repository index..."
    if ssh "$REMOTE_USER@$REMOTE_SERVER" "grep -q 'Package: ros-${ROS_DISTRO}-ros2-controllers-interfaces' $REMOTE_REPO_DIR/dists/noble/main/binary-amd64/Packages"; then
        echo -e "${GREEN}✓ Package found in repository index${NC}"
    else
        echo -e "${RED}✗ Package not found in repository index${NC}"
        return 1
    fi
    
    echo ""
    return 0
}

# Main deployment process
main() {
    # Check SSH connection
    if ! check_ssh; then
        exit 1
    fi
    echo ""
    
    # Find .deb files
    if ! find_deb_files; then
        exit 1
    fi
    
    # Confirm before proceeding
    echo -e "${BLUE}Ready to deploy ${#DEB_FILES[@]} package(s) to $REMOTE_SERVER${NC}"
    read -p "Continue? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Deployment cancelled."
        exit 0
    fi
    echo ""
    
    # Upload files
    if ! upload_files; then
        echo -e "${RED}Deployment failed during upload${NC}"
        exit 1
    fi
    
    # Regenerate metadata
    if ! regenerate_metadata; then
        echo -e "${RED}Deployment failed during metadata generation${NC}"
        exit 1
    fi
    
    # Verify installation
    if ! verify_installation; then
        echo -e "${YELLOW}Warning: Verification failed, but files may have been uploaded${NC}"
    fi
    
    echo ""
    echo -e "${GREEN}=== Deployment Complete ===${NC}"
    echo ""
    echo "The package is now available in the ACFR repository."
    echo ""
    echo -e "${GREEN}To install on a client machine:${NC}"
    echo "  1. Add the repository (if not already added):"
    echo "     echo 'deb [arch=amd64 trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/ noble main' | \\"
    echo "       sudo tee /etc/apt/sources.list.d/acfr.list"
    echo ""
    echo "  2. Update and install:"
    echo "     sudo apt update"
    echo "     sudo apt install ros-${ROS_DISTRO}-ros2-controllers-interfaces"
    echo ""
}

# Run main function
main
