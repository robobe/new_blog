---
title: Open scene graph
tags:
    - osg
    - open scene graph
    - osgEarth
---

{{ page_folder_links() }}

OpenSceneGraph (OSG) is an open-source 3D graphics toolkit written in C++, built on top of OpenGL.
It provides a high-level scene graph abstraction for building, rendering, and managing complex 3D scenes efficiently.

### Common use case

| Area                         | Description                                                 |
| ---------------------------- | ----------------------------------------------------------- |
| **Simulation & Training**    | Flight simulators, military simulators, driving trainers    |
| **Scientific Visualization** | Displaying large 3D datasets or terrains                    |
| **Geospatial Systems**       | Used by **osgEarth** to render real-world maps and terrain  |
| **Virtual Reality (VR)**     | Basis for immersive 3D visualization                        |
| **Game Engines / Tools**     | Lightweight base for building visualization or engine tools |


<div class="grid-container">
     <div class="grid-item">
        <a href="osg_earth">
            <p>OSG Earth</p>
        </a>
    </div>
</div>



## Install

```
git clone https://github.com/openscenegraph/OpenSceneGraph.git
cd OpenSceneGraph
mkdir build
cd build
cmake ..
make
sudo make install
suod ldconfig
```

---

## Demo

```cpp
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>

int main() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0), 1.0)));

    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    return viewer.run();
}

```

```c
cmake_minimum_required(VERSION 3.10)
project(SimpleOSGApp)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgUtil osgViewer)

# Create executable
add_executable(osg_example hello.cpp)

# Link OSG libraries
target_link_libraries(osg_example
    PRIVATE
        osg
        osgDB
        osgUtil
        osgViewer
)
```

---

## Learn path create by chatGPT

Each week builds on the previous one.
For each topic, I’ve included:

- 🎓 Concepts — what you’ll learn
- 💻 Practice Goals — what you’ll implement
- 📚 Resources — where to learn more

🗓️ Week 1 – Setup & Core Concepts

Goal: Understand what OSG is, how it organizes scenes, and render your first object.

🎓 Concepts
- Scene graph structure (Nodes, Groups, Geodes)
- Viewer and event loop (osgViewer::Viewer)
- Geometry, Drawables, and simple shapes
- Installing OSG and using CMake with VSCode

💻 Practice

- ✅ Install OSG (sudo apt install libopenscenegraph-dev)
- ✅ Build your first program (sphere example)
- ✅ Add multiple shapes (cube + sphere + cone)
- ✅ [Use osg::Group to attach nodes](learn_path/week1/index.md)
- ✅ Experiment with osgViewer::Viewer::run()

📚 Resources

OSG Wiki – Getting Started

---

🗓️ Week 2 – Transforms & Cameras

Goal: Learn to position objects and control the camera.

🎓 Concepts

- Transform nodes (osg::MatrixTransform)
- Local vs global coordinates
- Orbit manipulator (osgGA::OrbitManipulator)
- Camera node (osg::Camera)

💻 Practice

- ✅ Move an object using MatrixTransform
- ✅ Animate a rotation using osg::AnimationPathCallback
- ✅ Add a camera and control with mouse (OrbitManipulator)
- ✅ Play with setHomeViewpoint() to set default view

📚 Resources

Example: examples/osganimationpath

Camera Manipulators Doc

---

🗓️ Week 3 – Models, Materials & Lighting

Goal: Load real 3D models and make your scene look realistic.

🎓 Concepts

- Loading models (osgDB::readNodeFile)
- Materials, textures, and lights
- StateSets (managing OpenGL state)
- Shaders overview

💻 Practice

- ✅ Load a .obj or .osgb model
- ✅ Add texture using osg::Texture2D
- ✅ Add a light source (osg::LightSource)
- ✅ Adjust materials via osg::Material

📚 Resources

- Example: examples/osglight
- Example: examples/osgshadercomp
- Book: Beginner’s Guide Ch. 4–6

---

🗓️ Week 4 – Interaction & Events

Goal: Make the scene respond to input.

🎓 Concepts

- Event handlers (osgGA::GUIEventHandler)
- Keyboard and mouse input
- Picking (selecting objects with mouse)
- Callbacks (update, cull, draw)

💻 Practice

- ✅ Create a custom event handler for key presses
- ✅ Implement simple object movement (e.g., arrow keys to move)
- ✅ Add text overlay using osgText::Text
- ✅ Implement a basic “object picker”

📚 Resources

- Example: examples/osgevent
- osgGA::GUIEventHandler API
- [OSG Callback System](https://github.com/openscenegraph/OpenSceneGraph/wiki/Callbacks)

---

🗓️ Week 5 – Performance & Advanced Features

Goal: Understand optimization and advanced scene graph concepts.

🎓 Concepts

- Level of Detail (osg::LOD)
- Culling and bounding volumes
- Paging (loading large datasets dynamically)
- Multithreaded rendering overview
- Using shaders (GLSL in OSG)

💻 Practice

- ✅ Create LOD nodes for distant objects
- ✅ Visualize bounding boxes (osg::BoundingBox)
- ✅ Try replacing default shader with custom GLSL
- ✅ Explore example: osgmultitexture

📚 Resources

- Example: examples/osglod
- OSG Rendering Optimization

---

🗓️ Week 6 – Integration & osgEarth

Goal: Combine what you learned and build a mini-project.

🎓 Concepts

- osgEarth basics (MapNode, ImageLayer, ElevationLayer)
- Adding UI (Qt or ImGui optional)
- Combining 3D models + terrain
- Exporting and saving scenes

💻 Practice

- ✅ Install osgEarth (sudo apt install osgEarth)
- ✅ Load OpenStreetMap base layer
- ✅ Add a model (e.g., aircraft or robot) at specific lat/lon
- ✅ Use OrbitManipulator to explore the map
- ✅ Export scene with osgDB::writeNodeFile()

📚 Resources

- osgEarth GitHub: https://github.com/gwaldron/osgearth

---

🧩 Optional Weeks (7+)

- Week 7 → integrate with Qt GUI (using osgQt::GLWidget)
- Week 8 → build plugin or viewer utility
- Week 9 → add VR support (OpenVR / Oculus)
- Week 10 → integrate with ROS 2 or simulation data (e.g., real sensor visualization)


---

## Reference
- [OpenSceneGraph Tutorial for Algorithms Development](https://medium.com/@ptkinvent/openscenegraph-for-algorithms-development-e4b0eb390e71)