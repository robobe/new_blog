---
title: Cpp build type
tags:
    - cmake
    - build
    - debug
    - release
---



- **Debug** : active debugging, easiest to inspect
- **RelWithDebInfo** : realistic performance + debug symbols
- **Release** : final optimized build
- **MinSizeRel**: build optimized binaries prioritizing smaller executable/library size.

```bash
 # Debug
  cmake -S . -B build/debug -DCMAKE_BUILD_TYPE=Debug
  cmake --build build/debug

  # Release
  cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release
  cmake --build build/release

```

## CMakePresets.json

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "base",
      "hidden": true,
      "generator": "Ninja"
    },
    {
      "name": "debug",
      "inherits": "base",
      "binaryDir": "${sourceDir}/build/debug",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug"
      }
    },
    {
      "name": "relwithdebinfo",
      "inherits": "base",
      "binaryDir": "${sourceDir}/build/relwithdebinfo",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "RelWithDebInfo"
      }
    },
    {
      "name": "release",
      "inherits": "base",
      "binaryDir": "${sourceDir}/build/release",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release"
      }
    }
  ]
}
```

```
{
  "version": 6,
  "configurePresets": [
    {
      "name": "base",
      "hidden": true,
      "generator": "Ninja"
    },
    {
      "name": "debug",
      "inherits": "base",
      "binaryDir": "${sourceDir}/build/debug",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug"
      }
    },
    {
      "name": "relwithdebinfo",
      "inherits": "base",
      "binaryDir": "${sourceDir}/build/relwithdebinfo",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "RelWithDebInfo"
      }
    },
    {
      "name": "release",
      "inherits": "base",
      "binaryDir": "${sourceDir}/build/release",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release"
      }
    }
  ]
}

```bash
# debug
cmake --preset debug
cmake --build build/debug

# release
cmake --preset release
cmake --build build/release~
```


---

## Tips

### if condition

```cmake
  target_compile_options(my_app PRIVATE
      $<$<CONFIG:Debug>:-Wall -Wextra>
  )
```

  - target_compile_options(my_app ...) adds compiler options to the my_app target.
  - PRIVATE means the options affect only my_app; they are not inherited by targets that link to it.
  - $<...> is a CMake generator expression, evaluated when CMake generates the build files.
  - $<CONFIG:Debug> evaluates to 1 for Debug and 0 otherwise.
  - $<$<CONFIG:Debug>:-Wall -Wextra> supplies -Wall -Wextra only when the condition is true.