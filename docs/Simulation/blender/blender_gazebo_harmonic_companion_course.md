# Blender for Gazebo Harmonic Companion Course

## Course Goal

At the end of this course you will be able to:

- Create simulation-ready assets in Blender
- Export meshes correctly for Gazebo Harmonic
- Build complete simulation worlds
- Create terrains and heightmaps
- Optimize meshes for real-time simulation
- Create environments for drones, UGVs, optical flow, and YOLO training
- Understand Gazebo asset structure and resource paths

---

# Module 1 - Blender and Gazebo Basics

## Lesson 1.1 - How Gazebo Uses Meshes

Topics:

- Visual vs Collision
- SDF models
- Mesh formats
- Coordinate systems

Exercise:

- Inspect existing Gazebo models

---

## Lesson 1.2 - Gazebo Asset Pipeline

Pipeline:

CAD → Blender → GLB/DAE → SDF → Gazebo

Exercise:

- Import an STL
- Export GLB
- Load into Gazebo

---

# Module 2 - Coordinate Systems

## Lesson 2.1 - Blender Axes

Topics:

- X axis
- Y axis
- Z axis

Exercise:

- Create axis markers

---

## Lesson 2.2 - Gazebo Axes

Topics:

- Z-up convention
- Model orientation

Exercise:

- Export cube
- Verify orientation in Gazebo

---

# Module 3 - Building Gazebo Models

## Lesson 3.1 - Landing Pad

Build:

- Base plate
- Markings
- Materials

Export:

- landing_pad.glb

---

## Lesson 3.2 - Buildings

Build:

- Simple building
- Warehouse
- Hangar

Export:

- building.glb

---

## Lesson 3.3 - Obstacles

Create:

- Barriers
- Boxes
- Poles

Use in:

- Drone avoidance tests
- Navigation

---

# Module 4 - Materials and PBR

## Lesson 4.1 - Basic Materials

Create:

- Metal
- Concrete
- Plastic

---

## Lesson 4.2 - PBR Workflow

Learn:

- Albedo
- Roughness
- Metallic
- Normal maps

Exercise:

- Create realistic runway

---

# Module 5 - Drone Simulation Environment

## Lesson 5.1 - Drone Test Field

Create:

- Landing zone
- Trees
- Obstacles

---

## Lesson 5.2 - Optical Flow Environment

Create:

- Textured floor
- Rich visual features

Goals:

- Improve optical flow
- Improve SLAM testing

---

## Lesson 5.3 - Tracking Environment

Create:

- Moving target area
- High contrast objects

Use with:

- NanoTracker
- YOLO

---

# Module 6 - Terrain Creation

## Lesson 6.1 - Heightmaps

Create:

- Hills
- Valleys
- Roads

Export:

- PNG heightmap

---

## Lesson 6.2 - Large Outdoor World

Create:

- Mountains
- Forests
- Open fields

Use in Gazebo Harmonic

---

# Module 7 - Performance Optimization

## Lesson 7.1 - Polygon Count

Measure:

- Vertices
- Faces
- Triangles

---

## Lesson 7.2 - Decimate Modifier

Reduce:

- 100k triangles
- 50k triangles
- 10k triangles

Compare Gazebo performance

---

## Lesson 7.3 - Visual vs Collision Meshes

Create:

- High quality visual mesh
- Simplified collision mesh

Benefits:

- Better real-time factor
- Faster physics

---

# Module 8 - Gazebo Model Packaging

## Lesson 8.1 - Model Structure

Create:

model/
├── model.sdf
├── model.config
└── meshes/

---

## Lesson 8.2 - Resource Paths

Topics:

- GZ_SIM_RESOURCE_PATH
- Fuel assets

Exercise:

- Load custom model

---

# Module 9 - Synthetic Data Generation

## Lesson 9.1 - Camera Placement

Create:

- Fixed camera
- Drone camera
- Ground vehicle camera

---

## Lesson 9.2 - Domain Randomization

Randomize:

- Lighting
- Weather
- Object positions

Generate YOLO datasets

---

# Final Project 1

Drone Training World

Contains:

- Landing pad
- Trees
- Buildings
- Obstacles

Use with Gazebo Harmonic

---

# Final Project 2

Autonomous Vehicle Track

Contains:

- Road network
- Signs
- Obstacles

Use with:

- ROS 2
- Navigation

---

# Final Project 3

Computer Vision World

Contains:

- Targets
- Rich textures
- Lighting variations

Use with:

- YOLO
- Optical Flow
- Tracking

---

# Recommended YouTube Tutorials

## Blender Basics

- Blender Guru Donut Series
- Grant Abbitt Beginner Course

## Gazebo Related

Search:

- Blender to Gazebo tutorial
- Gazebo Harmonic custom model tutorial
- Gazebo GLB mesh tutorial
- Gazebo terrain tutorial

## Environment Design

Search:

- Blender terrain beginner
- Blender low poly environment
- Blender PBR materials

---

# Suggested Schedule

Week 1
- Blender basics
- Coordinate systems
- Export workflow

Week 2
- Buildings
- Obstacles
- Landing pads

Week 3
- Materials
- PBR
- Terrain

Week 4
- Model packaging
- Gazebo integration

Week 5
- Synthetic data
- Final projects

---

# Deliverables

By the end of the course:

1. Drone landing world
2. Outdoor terrain world
3. Autonomous vehicle track
4. Gazebo-ready asset library
5. YOLO training environment
