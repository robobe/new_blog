---
tags:
    - osg
    - open scene graph
    - osgEarth
---


{{ page_folder_links() }}


## osg earth
[osg earth](https://docs.osgearth.org/en/latest/build.html)


```bash
# OSG
git clone https://github.com/openscenegraph/OpenSceneGraph.git
cd OpenSceneGraph && mkdir build && cd build
cmake .. -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

# osgEarth
git clone https://github.com/gwaldron/osgearth.git
cd osgearth && mkdir build && cd build
cmake .. -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

```