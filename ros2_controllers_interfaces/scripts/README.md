# ROS2 Controllers Interfaces Package - Build and Deployment

This directory contains scripts for building and deploying the `ros2_controllers_interfaces` package as a Debian package to the ACFR APT repository.

## Overview

The `ros2_controllers_interfaces` package contains custom ROS 2 message definitions:
- **SwerveControllerStatus.msg**: Status message for swerve drive controllers

## Prerequisites

### Local System
```bash
sudo apt install python3-bloom fakeroot dpkg-dev
```

### SSH Access
- SSH key authentication set up for `avocado.acfr.usyd.edu.au`
- Access to `/data/www/EHM/datasets/ubuntu-repo/`

## Scripts

### 1. `create_deb_package.sh`
Builds a Debian package for ros2_controllers_interfaces.

**Usage:**
```bash
cd /home/jjustin/gh_ws/src/groundhog/ros2_controllers/ros2_controllers_interfaces
./scripts/create_deb_package.sh
```

**What it does:**
1. Generates debian build files using bloom
2. Builds the package using fakeroot
3. Outputs .deb file to `/home/jjustin/gh_ws/`

**Output file:**
- `ros-jazzy-ros2-controllers-interfaces_*.deb`

### 2. `deploy_to_apt_repo.sh`
Uploads package to the ACFR APT repository and regenerates metadata.

**Usage:**
```bash
./scripts/deploy_to_apt_repo.sh
```

**What it does:**
1. Finds the .deb file
2. Uploads to `avocado.acfr.usyd.edu.au:/data/www/EHM/datasets/ubuntu-repo/pool/main/`
3. Regenerates repository metadata in `dists/noble/main/binary-amd64/`
4. Verifies package is in repository index

## Complete Workflow

### Build and Deploy
```bash
# 1. Build the package
cd /home/jjustin/gh_ws/src/groundhog/ros2_controllers/ros2_controllers_interfaces
./scripts/create_deb_package.sh

# 2. Deploy to repository
./scripts/deploy_to_apt_repo.sh
```

### Manual Upload (Alternative)
If you prefer to upload files manually:
```bash
# 1. Build package
./scripts/create_deb_package.sh

# 2. Manually upload
scp /home/jjustin/gh_ws/ros-jazzy-ros2-controllers-interfaces*.deb \
    jjustin@avocado.acfr.usyd.edu.au:/data/www/EHM/datasets/ubuntu-repo/pool/main/

# 3. SSH to server and regenerate metadata
ssh jjustin@avocado.acfr.usyd.edu.au
cd /data/www/EHM/datasets/ubuntu-repo
dpkg-scanpackages -m pool/main /dev/null > dists/noble/main/binary-amd64/Packages
gzip -9c dists/noble/main/binary-amd64/Packages > dists/noble/main/binary-amd64/Packages.gz
# Then update Release file checksums
```

## Installation on Client Machines

### Add ACFR Repository
```bash
echo 'deb [arch=amd64 trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/ noble main' | \
  sudo tee /etc/apt/sources.list.d/acfr.list
```

### Install Package
```bash
sudo apt update
sudo apt install ros-jazzy-ros2-controllers-interfaces
```

This will install the ros2_controllers_interfaces message definitions package.

## Docker Integration

To use this package in Docker:

```dockerfile
# Add ACFR repository
RUN echo 'deb [arch=amd64 trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/ noble main' | \
    sudo tee /etc/apt/sources.list.d/acfr.list && sudo apt update

# Install package
RUN sudo apt install -y ros-jazzy-ros2-controllers-interfaces
```

## Troubleshooting

### Bloom fails
- Ensure you're in the package directory when running the build script
- Check that package.xml is valid

### SSH connection fails
- Ensure SSH keys are set up: `ssh-copy-id jjustin@avocado.acfr.usyd.edu.au`
- Test connection: `ssh jjustin@avocado.acfr.usyd.edu.au`

### Package not showing up after deployment
- Check that the file was uploaded to `pool/main/` directory
- Verify the Packages file was regenerated in `dists/noble/main/binary-amd64/`
- On client: `sudo rm /var/lib/apt/lists/data.acfr.usyd.edu.au_* && sudo apt update`

## Package Version

Current version (from package.xml): **0.0.1**

## Repository Structure

The ACFR repository uses standard Debian structure:
```
/data/www/EHM/datasets/ubuntu-repo/
├── dists/
│   └── noble/
│       ├── Release
│       └── main/
│           └── binary-amd64/
│               ├── Packages
│               └── Packages.gz
└── pool/
    └── main/
        └── ros-jazzy-*.deb
```

## Notes

- Package built for Ubuntu 24.04 (Noble)
- ROS 2 distribution: Jazzy
- The remote server is RHEL, so the scripts use compatible commands (e.g., `LANG=C date` instead of `date -R`)
- Debug symbol packages (*.dbgsym.deb) are not uploaded to the repository
