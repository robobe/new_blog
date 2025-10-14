---
tags:
    - nvidia
    - jetson
    - jetpack
---

# Nvidia jetson Jetpack

## SDK Manager

### Tips

```bash title="install from cli"
#from nvidia/nvidia_sdk/JetPack_6.2.1_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra
sudo ./nvsdkmanager_flash.sh --storage nvme0n1p1
```

---

```bash
# TODO: check when and way to command relevant
# I install from sdk and missing opencv and nvcc, the command fix it
sudo apt install nvidia-jetpack
```

## Mapping 
Mapping between **Jetpack** version and **Jetson Linux**

```bash
cat /etc/nv_tegra_release
#
# R36 (release), REVISION: 4.3,
```

it's map to Jetson linux 36.4.3

!!! note "Jetson linux Downloads"
    The version map to link for download roosfs and toolchain
    
    ```
    https://developer.nvidia.com/embedded/jetson-linux-r3643
    ```
     

---

## Build custom rootfs

![](docs/assets/images/under_construction.png)

[Root file System](https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/RootFileSystem.html)

The minimal root file system is the basic root file system that is used for NVIDIA Jetson develop kits. This file system does not provide the GUI mode, and all manipulations can be completed only by using the SSH or UART console. The file system does not also provide an OEM configuration, so we recommend that you create a default user before you flash the device. Refer to l4t_create_default_user.sh in Skipping oem-config for more information.

```bash
sudo ./nv_build_samplefs.sh --abi aarch64 --distro ubuntu --flavor minimal --version jammy
```

!!! warning "fix l4t_create_default_user.sh"

    function `show_eula()` replace if order

    ```bash
    if [ "${accept_license}" = true ]; then
		return
	fi
	
	if [ ! -f "${license_file}" ] && [ ! -f "${license_gz_file}" ]; then
		echo "ERROR: Cannot find the Tegra software license agreement"
		exit 1
	fi
    ```

     
```bash 
sudo ./apply_binaries.sh
```

```bash
sudo ./tools/l4t_create_default_user.sh -u user -p user -n prod --accept-license
```

```bash
cp -a desktop_rootfs/boot/ minimum_rootfs/
```