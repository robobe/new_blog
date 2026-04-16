---
title: Gazebo control GUI
tags:
    - gazebo
    - gui
    - plugins
    - layout
---

## Control simulation GUI

Gazebo world sdf contain `<gui>` section that control the gui layout and gui plugins that loaded, by default it use the defination in `gui.config` locate in `~/.gz/sim/8` and no need to add the `<gui>` tag.

| name  | usage  |
|---|---|
| MinimalScene  | Creates the render scene   |
| GzSceneManager  | GzSceneManager is the backend that manages the 3D scene  |
| CameraTracking  | makes the 3D camera automatically follow an entity (model or link) in the scene.  |
| WorldControl  |   |
| WorldStats  |   |
| ComponentInspector  |   |
| EntityTree  |   |


---

## Demo: Empty world
- Minimal world with `<gui>` section

<details>
<summary>world</summary>
```
--8<-- "docs/Simulation/Gazebo/gui_plugins/code/gui_world.sdf"
```
</details>