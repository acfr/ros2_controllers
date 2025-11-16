# Swerve Controller - Debian Package Build & Deployment

This directory contains scripts to build and deploy the `swerve_controller` ROS 2 package as a Debian package to the ACFR APT repository.

## Package Information

- **Package Name**: swerve_controller
- **Version**: 0.0.1
- **ROS Distribution**: Jazzy
- **Ubuntu Version**: 24.04 (Noble)
- **Dependencies**: ros2_controllers_interfaces, controller_interface, geometry_msgs, nav_msgs, and others

## Prerequisites

### Build Dependencies

```bash
sudo apt install python3-bloom fakeroot dpkg-dev
```

### Remote Server Access

- SSH access to `jjustin@avocado.acfr.usyd.edu.au`
- Write permissions to `/data/www/EHM/datasets/ubuntu-repo/`

### ROS 2 Dependencies

Ensure `ros2_controllers_interfaces` is available, either:
1. Already installed: `sudo apt install ros-jazzy-ros2-controllers-interfaces`
2. Added to custom rosdep rules at `/tmp/rosdep/custom.yaml`

## Scripts

### 1. `create_deb_package.sh`

Builds the Debian package for swerve_controller.

**Usage:**
```bash
cd /home/jjustin/gh_ws/src/groundhog/ros2_controllers/swerve_controller
./scripts/create_deb_package.sh
```

**What it does:**
1. Cleans previous build artifacts
2. Generates debian files using bloom
3. Builds the package with fakeroot
4. Outputs `.deb` file to `/home/jjustin/gh_ws/`

**Output:**
- `ros-jazzy-swerve-controller_0.0.1-0noble_amd64.deb`

### 2. `deploy_to_apt_repo.sh`

Deploys the package to the ACFR APT repository.

**Usage:**
```bash
./scripts/deploy_to_apt_repo.sh
```

**What it does:**
1. Uploads `.deb` file to `pool/main/` directory
2. Regenerates repository metadata (Packages, Packages.gz, Release)
3. Uses RHEL-compatible commands (server runs Red Hat Enterprise Linux)

**RHEL Compatibility Notes:**
- Uses `stat --format=%s` instead of `stat -c%s`
- Uses `LC_ALL=C date -u '+%a, %d %b %Y %H:%M:%S +0000'` for RFC 822 dates
- Uses `gzip -9fc` for safe compression

## Workflow

### Complete Build and Deployment

```bash
# 1. Build the package
cd /home/jjustin/gh_ws/src/groundhog/ros2_controllers/swerve_controller
./scripts/create_deb_package.sh

# 2. Deploy to repository
./scripts/deploy_to_apt_repo.sh
```

### Installation on Client Machines

After deployment, the package can be installed via:

```bash
# Update package lists
sudo apt update

# Install the package
sudo apt install ros-jazzy-swerve-controller
```

If the package doesn't appear, clear the APT cache:

```bash
sudo rm /var/lib/apt/lists/data.acfr.*
sudo apt update
```

## Repository Structure

```
/data/www/EHM/datasets/ubuntu-repo/
├── pool/
│   └── main/
│       ├── ros-jazzy-swerve-controller_0.0.1-0noble_amd64.deb
│       └── [other packages...]
└── dists/
    └── noble/
        ├── Release
        └── main/
            └── binary-amd64/
                ├── Packages
                └── Packages.gz
```

## Repository Configuration

Client machines need this APT source configuration:

**File**: `/etc/apt/sources.list.d/acfr.list`
```
deb [trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/ noble main
```

## Troubleshooting

### Package Not Found After Deployment

```bash
# Clear APT cache
sudo rm /var/lib/apt/lists/data.acfr.*

# Update package lists
sudo apt update

# Search for package
apt search ros-jazzy-swerve-controller
```

### Build Fails with Missing Dependencies

If `ros2_controllers_interfaces` is not found during build, add to `/tmp/rosdep/custom.yaml`:

```yaml
ros2_controllers_interfaces:
  ubuntu: [ros-jazzy-ros2-controllers-interfaces]
```

Then update rosdep:
```bash
rosdep update
```

### SSH Connection Issues

Ensure you have SSH keys configured for `avocado.acfr.usyd.edu.au`:

```bash
ssh-copy-id jjustin@avocado.acfr.usyd.edu.au
```

## Additional Information

- **Repository URL**: https://data.acfr.usyd.edu.au/ubuntu-repo/
- **Remote Server**: avocado.acfr.usyd.edu.au (RHEL)
- **Repository Path**: /data/www/EHM/datasets/ubuntu-repo/
- **Architecture**: amd64 only

## Related Packages

This package is part of the ros2_controllers suite:
- ros2_controllers_interfaces (dependency)
- Other controllers in the ros2_controllers workspace
