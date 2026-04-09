---
title: Lua metatable
tags:
    - lua
    - metatable
    - table
---

A metatable is a special table that changes how another table behaves.


## Core idea
Every table can have metatable

```c
setmetatable(table, metatable)
```

### Demo

```c
local defaults = {
    color = "red"
}

local obj = {
    size = 10
}

setmetatable(obj, {
    __index = defaults
})

print(obj.size)   -- 10 (from obj)
print(obj.color)  -- red (fallback to defaults)
```

!!! tip "__index"
    __index is a special field in a metatable that tells Lua:

    If a key is not found in this table, look somewhere else