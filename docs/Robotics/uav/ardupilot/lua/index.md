---
title: Ardupilot Lua
tags:
    - lua
    - ardupilot
---

!!! tip "ardupilot use Lua version 5.3"

!!! tip "ardupilot lua API"
    - [Ardupilot API Documentation](https://ardupilot.org/copter/docs/common-lua-scripts.html#api-documentation)
    - Ardupilot Lua API declare in `docs.lua` locate `libraries/AP_Scripting/docs/docs.lua`
    

!!! tip "Lua demo script"
    Can be found `libraries/AP_Scripting/tests`
    
## Demo
Run lua script using SITL

- Create `scripts` folder in SITL executable folder
- Add `SCR_ENABLE=1` parameter to param file
- Place the lua script under the `scripts` folder


```c title="hello.lua"
function update()
    gcs:send_text(0, "Hello from Lua script!")
    return update, 1000  -- run every 1000 ms
end

return update()
```

## Parameters

| Parameter          | Purpose                     | Typical Use        |
| ------------------ | --------------------------- | ------------------ |
| `SCR_ENABLE`       | Enable Lua                  | Always set to 1    |
| `SCR_DIR_DISABLE`  | Disable auto script loading | Rare               |
| `SCR_HEAP_SIZE`    | Memory for Lua              | Increase if crash  |

There more parameters for logging and debugging check documentation

### Predefined parameters
`SCR_USERx` parameter are generic parameters meant exactly for Lua scripts.

| Parameter   | Type  | Purpose            |
| ----------- | ----- | ------------------ |
| `SCR_USER1` | float | user-defined value |
| `SCR_USER2` | float | user-defined value |
| `SCR_USER3` | float | user-defined value |
| `SCR_USER4` | float | user-defined value |
| `SCR_USER5` | float | user-defined value |
| `SCR_USER6` | float | user-defined value |


```c
local gain = param:get("SCR_USER1")
```


!!! tip "Custom parameter"
    [Ardupilot Accessing/Adding Parameters via Scripts](https://ardupilot.org/copter/docs/common-scripting-parameters.html)
    

---

## VSCode

[VSCode lua](https://marketplace.visualstudio.com/items?itemName=sumneko.lua)

### Using docs.lua

Get `libraries/AP_Scripting/docs/docs.lua` from ardupilot github (choose the right version)

- This file is not meant to run
- It’s a fake API definition for tooling (IntelliSense, completion)

```json title=".vscode/settings.json"
{
  "Lua.workspace.library": [
    "/path/to/ardupilot/libraries/AP_Scripting/docs"
  ],
  "Lua.diagnostics.globals": [
    "vehicle",
    "ahrs",
    "gcs",
    "arming",
    "param"
  ]
}
```

---

## Reference
- [Ardupilot Lua getting started](https://ardupilot.org/copter/docs/common-lua-scripts.html#getting-started)
- [Good project that use lua](https://github.com/yuri-rage/ArduRover_Mower)