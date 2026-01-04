---
title: LUKS on loop device
tags:
  - luks
  - encryption
---

**LUKS** stands for Linux Unified Key Setup. It’s the standard disk-encryption system on Linux, used to protect data at rest by encrypting entire disks or partitions.

A **loop device** is a virtual block device in Linux that lets you treat a regular file as if it were a disk or partition.

The idea is using blockfile as disk and use LUKS to protect the disk content

```
sudo apt install cryptsetup
```

## Demo
Create file that use as LUKS encrypt disk
Open the encrypted disk for usage and close it

The process has two main steps
- one time create the disk
    - open mount / umount close
    - create the encrypt disk
    - open it
    - format
    - mount
    - add files 
    - umount
    - add key
- Usage
    - Open and Mount
    - Umount and close


 
### Step1
One time step that contain the following steps
- create block file using `dd`
- run as a disk using loop device


1️⃣ Create blockfile using `dd`

```bash title="blockfile"
# create 100M blockfile
dd if=/dev/zero of=secure.img bs=1M count=100
```

2️⃣ Loop device

```bash title="run as a disk device"
sudo losetup --find --show ~/secure.img
# the command return the device loop to use
# /dev/loop3
```

3️⃣ Create the encrypt disk
- Initializes the device as a LUKS encrypted container
- Writes a LUKS header to the device
- Erases any existing filesystem or data references

```bash title="init"
sudo cryptsetup luksFormat /dev/loop3

# WARNING!
# ========
# This will overwrite data on /dev/loop3 irrevocably.
# 
# Are you sure? (Type 'yes' in capital letters): YES
# Enter passphrase for /home/user/secure.img: 
# Verify passphrase: 
```

4️⃣ Open the encrypted device
```bash
#Unlock a LUKS-encrypted block device and expose it as a new decrypted virtual device named secure_container.
# before /dev/loop3 are encrypted
sudo cryptsetup open /dev/loop3 secure_container
#
# the decrypt file open into /dev/mapper/secure_container
```

5️⃣ format
```bash title="format"
sudo mkfs.ext4 /dev/mapper/secure_container
```

6️⃣ Mount and add files
```bash title="mount"
sudo mount /dev/mapper/secure_container /mnt
```

```bash
echo "text" | sudo tee file
```

7️⃣ Umount and release resource

```bash
sudo umount /mnt
sudo cryptsetup close secure_container
sudo losetup -d /dev/loop3
```

7️⃣ Add key

```bash
dd if=/dev/urandom of=keyfile.bin bs=4096  count=1
sudo cryptsetup luksAddKey secure.img keyfile.bin

# Type the passphrase
```

---

## Encrypt disk usage

### Mount with key

```bash title="open and mount"
sudo cryptsetup open --type luks \
secure.img secure_container \
--key-file keyfile.bin

# open the encrypt to /dev/mapper/secure_container

# mount 
sudo mount /dev/mapper/secure_container /mnt
```

```bash title="close and umount"
sudo umount /mnt
sudo cryptsetup close secure_container
```