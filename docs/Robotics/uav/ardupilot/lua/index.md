---
title: Ardupilot Lua
tags:
    - lua
    - ardupilot
---

!!! tip "ardupilot lua API"
    Ardupilot Lua API declare in `docs.lua` locate `libraries/AP_Scripting/docs/docs.lua`
    

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


---

## Reference
- []()