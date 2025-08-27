---
tags:
    - cpp
    - vscode
    - settings
    - dev environment
---

# VSCode CPP dev environment

## Clang
- Install clang version 19 support cpp-20
- Config `update-alternative`
- Install clang libstd


```bash title="install"
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 19
#
sudo apt install libstdc++-12-dev
# work after fix update-alternatives
clang++ --version

```


```bash title="config clang using update alternative"
# sudo update-alternatives --install <symlink> <name> <path-to-binary> <priority>
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-19 100
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-19 100

sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-14 50
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-149 50

# switch
sudo update-alternatives --config clang
sudo update-alternatives --config clang++
```

---

## VSCode

### c_cpp

```json title="c_cpp_properties.json"
{
    "configurations": [
        {
            "name": "Clang",
            "includePath": [
                "${workspaceFolder}/**",
                "${default}"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/clang++",
            "cStandard": "c17",
            "cppStandard": "c++20",
            "intelliSenseMode": "linux-clang-x64"
        }
    ],
    "version": 4
}
```

### Tasks
- Add task that build current file using `clang` compiler


```json title="tasks.json"
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "build with clang",
      "type": "shell",
      "command": "clang++",
      "args": [
        "-std=c++20",
        "${file}",
        "-o",
        "${fileDirname}/${fileBasenameNoExtension}"
      ],
      "group": {
        "kind": "build",
        "isDefault": true
      },
      "problemMatcher": [
        "$gcc"
      ]
    }
  ]
}
```