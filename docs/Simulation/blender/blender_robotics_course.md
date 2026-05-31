# Blender for Robotics Engineers – Beginner Course

## Course Goal

At the end of this course you will be able to:

- Create simple robot parts
- Import CAD models
- Export DAE/GLB/STL
- Build Gazebo environments
- Create terrains
- Optimize meshes
- Apply textures
- Create camera training scenes for YOLO

---

# Module 1 - Blender Basics (1 Day)

## Lesson 1.1 - Interface

Learn:

- Viewport
- Outliner
- Properties panel
- Scene hierarchy

Exercise:

- Create cube
- Move cube
- Delete cube
- Save project

## Lesson 1.2 - Navigation

Learn:

- Rotate View (Middle Mouse)
- Pan (Shift + Middle Mouse)
- Zoom (Mouse Wheel)
- Front View (Numpad 1)
- Side View (Numpad 3)
- Top View (Numpad 7)

Exercise:

- Create 5 cubes
- Navigate around the scene
- Focus on an object

---

# Module 2 - Basic Modeling (2 Days)

## Lesson 2.1 - Transformations

Learn:

- Move (G)
- Rotate (R)
- Scale (S)

Exercise:

- Box
- Cylinder
- Cone

## Lesson 2.2 - Edit Mode

Learn:

- Vertices
- Edges
- Faces

Shortcuts:

- Tab
- 1 Vertex
- 2 Edge
- 3 Face

Exercise:

- Simple drone arm
- Simple wall
- Simple table

## Lesson 2.3 - Extrude

Shortcut:

- E

Exercise:

- Building
- Road
- Landing pad

---

# Module 3 - Robotics Assets (2 Days)

## Lesson 3.1 - Import Existing Models

Import:

- STL
- OBJ
- GLB

## Lesson 3.2 - Fix Scale

Rule:

- 1 Blender Unit = 1 Meter

Exercise:

- 1m cube
- 2m wall

## Lesson 3.3 - Origin and Pivot

Learn:

- Set Origin
- Apply Transform

Important for:

- Robot links
- Wheels
- Gimbals

---

# Module 4 - Gazebo Workflow (2 Days)

## Lesson 4.1 - Export DAE

Exercise:

- Create a simple drone frame
- Export frame.dae

## Lesson 4.2 - Export GLB

Exercise:

- Building
- Tree
- Obstacle

## Lesson 4.3 - Create Environment

Build:

- Road
- Buildings
- Landing zone

Export to Gazebo Harmonic.

---

# Module 5 - Materials and Textures (2 Days)

## Lesson 5.1 - Materials

Create:

- Metal
- Plastic
- Concrete

## Lesson 5.2 - Textures

Apply:

- Road texture
- Grass texture
- Wall texture

Useful for:

- Optical flow
- Visual SLAM
- YOLO

---

# Module 6 - Terrain Creation (2 Days)

## Lesson 6.1 - Heightmaps

Create:

- Hill
- Valley
- Road

## Lesson 6.2 - Terrain Export

Export:

- PNG Heightmap

Use in Gazebo.

Exercise:

- Drone test area

---

# Module 7 - Mesh Optimization (1 Day)

## Lesson 7.1 - Polygon Count

Learn:

- Statistics Overlay

## Lesson 7.2 - Decimate Modifier

Reduce mesh complexity for simulation performance.

Example:

- 100,000 triangles → 5,000 triangles

---

# Module 8 - Synthetic Data Generation (Optional)

## Lesson 8.1 - Cameras

Create:

- Track
- Car
- Drone

## Lesson 8.2 - Lighting

Experiment with:

- Morning
- Noon
- Sunset
- Night

## Lesson 8.3 - Domain Randomization

Randomize:

- Colors
- Objects
- Textures
- Weather

Generate training images for YOLO.

---

# Final Projects

## Project 1 – Drone Landing Zone

Contains:

- Terrain
- Buildings
- Trees
- Landing pad

Export to Gazebo.

## Project 2 – Autonomous Vehicle Track

Contains:

- Road
- Signs
- Obstacles

Use with YOLO.

## Project 3 – Optical Flow Test Environment

Contains:

- Textured floor
- Moving objects
- Camera viewpoints

Use with a GStreamer optical flow pipeline.

---

# Recommended YouTube Tutorials

## Beginner Blender

1. Blender Beginner Tutorial Series – Blender Guru
   https://www.youtube.com/@blenderguru

2. Grant Abbitt – Blender Beginners Course
   https://www.youtube.com/@grabbitt

## Modeling

3. CG Essentials
   https://www.youtube.com/@TheCGEssentials

4. Ryan King Art
   https://www.youtube.com/@RyanKingArt

## Robotics / Simulation Related

5. Gazebo + Blender Mesh Workflow
   Search:
   "Blender Gazebo mesh tutorial"

6. ROS Mesh Export Tutorials
   Search:
   "Blender URDF STL DAE tutorial"

## Terrain and Environment Design

7. Blender Terrain Tutorials
   Search:
   "Blender terrain beginner"

8. Low Poly Environment Design
   Search:
   "Blender low poly environment"

---

# Learning Schedule

Week 1
- Navigation
- Interface
- Basic Modeling

Week 2
- Edit Mode
- Import/Export
- Scale and Origins

Week 3
- Gazebo Workflow
- Materials
- Textures

Week 4
- Terrain Creation
- Mesh Optimization
- Final Projects
