---
title: Lua module
tags:
    - lua
    - module
    - require
---

A Lua module is Lua file that return a table

## Demo

```c
--8<-- "docs/Programming/lua/module/code/mymodule.lua"
```

```c title="main.lua"
--8<-- "docs/Programming/lua/module/code/main.lua"
```

---

## Lua search path

### package.path

### LUA_PATH

```bash
export LUA_PATH="./mylibs/?.lua;;"
```