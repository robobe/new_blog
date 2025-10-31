---
title: OSG viewer loop
tags:
    - osg
    - viewer
    - run
---


{{ page_folder_links() }}

understand what happens inside OSG’s main rendering loop and how you can customize or control it.

```cpp
return viewer.run();
```

It starts OSG’s main loop, which:
- Opens a rendering window
- Initializes the camera and graphics context
- Continuously:
    - Polls user input (keyboard/mouse)
    - Updates scene nodes (via callbacks)
    - Traverses the scene graph
    - Draws the current frame
    - Exits when the window closes or viewer.done() returns true.


---

## Manual 

```cpp title="run_loop.cpp"
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgGA/OrbitManipulator>

int main()
{
    // --- Create a shape (a simple box) ---
    osg::ref_ptr<osg::ShapeDrawable> box =
        new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 1.0));
    // Ensures visible color even if lighting is off.
    box->setColor(osg::Vec4(0.2f, 0.8f, 0.2f, 1.0f)); // green color

    // --- Create a geode (geometry node) and add the shape ---
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(box);

    // --- Create the viewer ---
    osgViewer::Viewer viewer;

    // Set up a visible window (x, y, width, height)
    // Ensures a proper rendering window is created (some systems need it).
    viewer.setUpViewInWindow(100, 100, 800, 600);

    // Add a manipulator so you can move the camera with the mouse
    viewer.setCameraManipulator(new osgGA::OrbitManipulator());

    // Set the scene data
    viewer.setSceneData(geode.get());

    // --- Main loop ---
    while (!viewer.done())
    {
         // Here you can:
        // 1. update object positions
        // 2. run physics or sensor code
        // 3. handle custom input
        viewer.frame(); // Render one frame
    }

    return 0;
}

```

```c
find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgUtil osgViewer osgGA)
set(OSG_LIBRARIES
    osg
    osgDB
    osgUtil
    osgViewer
    osgGA
)

add_executable(run_loop run_loop.cpp)
target_link_libraries(run_loop PRIVATE ${OSG_LIBRARIES})
```