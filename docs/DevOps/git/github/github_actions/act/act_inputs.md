---
tags:
    - act
    - github actions
    - input
---
# ACT
[user guide](https://nektosact.com/usage/index.html)

## actrc
command-line arguments for act so you don't have to type them every time.

```bash title="platform"
# ~/.actrc
# set platforms
-P ubuntu-latest=ubuntu:22.04
-P ubuntu-22.04=node:16-bullseye-slim
```

!!! tip "offline"
    Set `--action-offline-mode` to run act in offline mode. This is useful when you want to run act without internet access or when you want to avoid downloading images from the internet.


    ```init title="artrc"
    --action-offline-mode

    ```
     

---

## Pass inputs to workflow

### Demo

```yaml
name: demos
on: 
  workflow_dispatch:
    inputs:
      TITLE:
        description: "title arg"
        type: string

jobs:
  simple_input:
    runs-on: ubuntu-latest
    
    steps:
      - name: echo title
        run: echo "---> ${{ github.event.inputs.TITLE }}"
```

### usage

```bash
act -j simple_input --pull=false --input TITLE="hello world"
```

---

### Skip job and step using input ans if condition

```yaml title="skip step"
name: demos
on: 
  workflow_dispatch:
    inputs:

      SKIP:
        description: "true for skip"

jobs:
  simple_skip:
    runs-on: ubuntu-latest
    
    steps:
      - name: step1
        run: echo "---> step 1"
      - name: step2
        if: ${{ github.event.inputs.SKIP != 'true' }}
        run: echo "---> step 2"
      - name: step3
        run: echo "---> step 3"
```

### usage

```bash
act -j simple_skip --pull=false --input SKIP=true
act -j simple_skip --pull=false --input SKIP=false
```


```yaml title="skip job"
name: demos
on: 
  workflow_dispatch:
    inputs:

      SKIP:
        description: "true for skip"

jobs:
  simple_skip:
    runs-on: ubuntu-latest
    if: ${{ github.event.inputs.SKIP != 'true' }}
    
    steps:
      - name: step1
        run: echo "---> step 1"
      - name: step2
        run: echo "---> step 2"
      - name: step3
        run: echo "---> step 3"
```

### usage

```bash
act -j simple_skip --pull=false --input SKIP=true
act -j simple_skip --pull=false --input SKIP=false
```