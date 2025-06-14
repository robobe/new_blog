---
tags:
    - opencv
    - cv_bridge
    - ros
    - custom build
---

# Build CV_Bridge for custom opencv build

Build [opencv 4.10](Programming/vision/opencv/build/) with cuda support
Use Docker to create build image

- clone github


## Build on docker
- docker base cuda **devel** with cudnn `FROM nvidia/cuda:12.6.0-cudnn-devel-ubuntu22.04` 
- Install dependencies
- Run colcon


```
sudo apt install
    libboost-all-dev \
    libboost-python-dev \
    python3-dev
```

```bash title="colcon"
colcon build --packages-up-to cv_bridge \
--cmake-args \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
  -DCMAKE_INCLUDE_PATH=/usr/include/opencv4/
```

---

## Pack to debian

### patch files

```bash title="patch_package.xml"
#/bin/bash

# comment libopencv-dev, python3-opencv
sed -i 's|^\([[:space:]]*\)<depend>libopencv-dev</depend>|\1<!-- <depend>libopencv-dev</depend> -->|' package.xml
sed -i 's|^\([[:space:]]*\)<depend>python3-opencv</depend>|\1<!-- <depend>python3-opencv</depend> -->|' package.xml
```

```bash title="patch_rules.sh"
#!/bin/bash
set -e
# remove current override_dh_shlibdeps
sed -i '/^override_dh_shlibdeps:/,/^$/d' /workspace/cv_bridge/debian/rules

 

cat <<'EOF' >> debian/rules

override_dh_shlibdeps:
    dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info


override_dh_builddeb:
    dh_builddeb --destdir=/workspace/debs
EOF

echo "[✅] Successfully patched debian/rules"
```

```bash title="patch_compat"
cat <<'EOF' > debian/compat
10
EOF
```

```bash title="add entry to changelog"
DEBEMAIL="you@example.com" DEBFULLNAME="Your Name" dch \
  --newversion 3.2.1-1jammy-cv410 \
  --distribution jammy \
  --urgency high \
  "build against opencv4.10"

```

!!! note "dch command"
     ```bash
     sudo apt install devscripts
     ```