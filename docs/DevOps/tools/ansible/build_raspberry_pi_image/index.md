---
title: Build Raspberry Pi image with Ansible
tags:
    - ansible
    - image
    - raspberry-pi
    - chroot
---

# Build Raspberry Pi image with Ansible

This demo builds, or "cooks", a Raspberry Pi image by mounting its root
filesystem and running Ansible against it.

This is not normal SSH Ansible. The Ansible target is a mounted image rootfs,
and the inventory maps the host name `robot_image` to `/mnt/robot-root`.

## Workflow

- Download an image as the base
- Clone the image
- Attach the image as a loop device
- Mount the rootfs and boot partitions
- Prepare QEMU ARM emulation and chroot mounts
- Run Ansible with a chroot connection
- Cleanup and unmount
- Flash the final image to SD

```bash
unxz -k ubuntu-24.04.3-preinstalled-server-arm64+raspi.img.xz
```

```bash
cp --reflink=auto \
    ubuntu-24.04.3-preinstalled-server-arm64+raspi.img \
    robot-os.img
```

!!! tip "reflink"
    `cp --reflink=auto source.img copy.img`

    If the filesystem supports it, both files initially share the same disk
    blocks. When one file is modified, only the changed blocks are copied. This
    makes large file copies very fast and space efficient.

```bash
fdisk -l robot-os.img
...

Device                                              Boot   Start     End Sectors  Size Id Type
robot-os.img1 *       2048 1050623 1048576  512M  c W95 FAT32 (LBA)
robot-os.img2      1050624 7882899 6832276  3.3G 83 Linux
```

## Attach as image loop

`losetup --partscan` attaches the image as a loop device and asks the kernel to
scan its partitions. This exposes partitions like `/dev/loop0p1` and
`/dev/loop0p2`.

```bash
LOOP_DEV=$(sudo losetup --find --show --partscan robot-os.img)

echo "$LOOP_DEV"
```

## Mount the Ubuntu root filesystem

Create mount directories:

```bash
sudo mkdir -p /mnt/robot-root
sudo mkdir -p /mnt/robot-root/boot/firmware
```

Mount partition 2 as the root filesystem:

```bash
sudo mount "${LOOP_DEV}p2" /mnt/robot-root
```

Mount the Raspberry Pi boot partition inside it:

```bash
sudo mount "${LOOP_DEV}p1" /mnt/robot-root/boot/firmware
```

---

## Prepare ARM chroot from the host

QEMU and `binfmt-support` allow an x86 host to run ARM64 binaries inside the
mounted Raspberry Pi root filesystem.

```bash
sudo apt install qemu-user-static binfmt-support
```

```bash title="copy into /usr/bin"
sudo cp `which qemu-aarch64-static` /mnt/robot-root/usr/bin
```

Bind mounts provide the chroot with access to the host's runtime kernel
interfaces.

```bash
sudo mount --bind /dev  /mnt/robot-root/dev
sudo mount --bind /proc /mnt/robot-root/proc
sudo mount --bind /sys  /mnt/robot-root/sys
sudo mount --bind /run  /mnt/robot-root/run
```

```bash title="configure DNS"
sudo cp /etc/resolv.conf \
    /mnt/robot-root/etc/resolv.conf
```

```bash
sudo chroot /mnt/robot-root
```

---

## Configure Ansible chroot inventory

The important part is `ansible_connection: community.general.chroot`. It tells
Ansible to treat the mounted rootfs as the target host.

```yaml title="inventory.yml"
all:
  hosts:
    robot_image:
      ansible_host: /mnt/robot-root
      ansible_connection: community.general.chroot
      ansible_python_interpreter: /usr/bin/python3
```

```bash title="test connection"
sudo -E ansible \
    -i inventory.yml \
    robot_image \
    -m ansible.builtin.command \
    -a "uname -m"
```

!!! tip ""
    `robot_image` is the host name declared in `inventory.yml`.

## Create a playbook

Keep the playbook idempotent: install packages, copy files, create directories,
and configure state in repeatable steps.

```yaml title="robot.yml"
---
- name: Configure Raspberry Pi image
  hosts: robot_image
  gather_facts: true

  environment:
    DEBIAN_FRONTEND: noninteractive

  tasks:
    - name: Update apt cache
      ansible.builtin.apt:
        update_cache: true
        cache_valid_time: 3600

    - name: Install robot packages
      ansible.builtin.apt:
        name:
          - git
          - tmux
          - htop
          - python3
          - python3-pip
          - gstreamer1.0-tools
        state: present

    - name: Configure hostname
      ansible.builtin.copy:
        content: "robot-pi\n"
        dest: /etc/hostname
        owner: root
        group: root
        mode: "0644"

    - name: Create application directory
      ansible.builtin.file:
        path: /opt/robot
        state: directory
        owner: root
        group: root
        mode: "0755"
```

```bash title="Run it"
sudo -E ansible-playbook \
    -i inventory.yml \
    robot.yml
```

## Cleanup

Cleanup order matters. Remove the copied QEMU binary first, then unmount bind
mounts, then unmount the image partitions.

```bash
sudo rm -f /mnt/robot-root/usr/bin/qemu-aarch64-static
```

```bash
sudo umount -R /mnt/robot-root/run
sudo umount -R /mnt/robot-root/sys
sudo umount /mnt/robot-root/proc
sudo umount -R /mnt/robot-root/dev
```

```bash
sudo umount /mnt/robot-root/boot/firmware
sudo umount /mnt/robot-root
sync
```

## The complete flow

```text
robot-os.img
      ↓
mount root partition
      ↓
prepare QEMU ARM emulation
      ↓
Ansible chroot connection
      ↓
install packages and configuration
      ↓
unmount
      ↓
flash robot-os.img to SD
```
