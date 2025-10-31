---
title: OSG Group
tags:
    - osg
    - group
---


{{ page_folder_links() }}

- osg::Group is a container node — it can have one or more child nodes.
- It is used to organize your scene in a tree-like hierarchy called a scene graph.
- Each child can be a 3D object (osg::Geode), another Group, or even a transformation node.

You can think of it like a folder that holds multiple objects inside the 3D scene.


```
osg::Group (root)
 ├── osg::Geode (sphere)
 │     └── osg::ShapeDrawable (Sphere)
 ├── osg::Geode (box)
 │     └── osg::ShapeDrawable (Box)
 └── osg::Geode (cone)
       └── osg::ShapeDrawable (Cone)

```

<details>
    <summary>Demo</summary>

```cpp
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Group>
#include <osgViewer/Viewer>

int main() {
    // Create drawables (actual shapes)
    osg::ref_ptr<osg::ShapeDrawable> sphere =
        new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), 1.0));
    osg::ref_ptr<osg::ShapeDrawable> box =
        new osg::ShapeDrawable(new osg::Box(osg::Vec3(3, 0, 0), 1.5));
    osg::ref_ptr<osg::ShapeDrawable> cone =
        new osg::ShapeDrawable(new osg::Cone(osg::Vec3(-3, 0, 0), 1.0, 2.0));

    // Create geodes to hold each drawable
    osg::ref_ptr<osg::Geode> geodeSphere = new osg::Geode();
    osg::ref_ptr<osg::Geode> geodeBox = new osg::Geode();
    osg::ref_ptr<osg::Geode> geodeCone = new osg::Geode();

    geodeSphere->addDrawable(sphere);
    geodeBox->addDrawable(box);
    geodeCone->addDrawable(cone);

    // Create a group node and attach all geodes
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(geodeSphere);
    root->addChild(geodeBox);
    root->addChild(geodeCone);

    // Viewer setup
    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    return viewer.run();
}

```
</details>

