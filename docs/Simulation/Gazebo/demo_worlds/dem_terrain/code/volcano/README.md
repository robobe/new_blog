# Volcano DEM world

This world loads the volcano terrain from the local model folder:

```text
models/volcano_local
```

`dem_volcano.sdf` includes it with:

```xml
<uri>model://volcano_local</uri>
```

## Run

Set `GZ_SIM_RESOURCE_PATH` to the local `models` folder before starting Gazebo:

```bash
cd docs/Simulation/Gazebo/demo_worlds/dem_terrain/code/volcano
export GZ_SIM_RESOURCE_PATH=$PWD/models:$GZ_SIM_RESOURCE_PATH
gz sim dem_volcano.sdf
```

For one command:

```bash
cd docs/Simulation/Gazebo/demo_worlds/dem_terrain/code/volcano
GZ_SIM_RESOURCE_PATH=$PWD/models:$GZ_SIM_RESOURCE_PATH gz sim dem_volcano.sdf
```
