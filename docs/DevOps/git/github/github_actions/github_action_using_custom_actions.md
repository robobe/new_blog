---
tags:
    - github
    - action
    - custom
    - composite
---

# Github actions using custom actions

---

## Runners
- [Act runners](https://nektosact.com/usage/runners.html)


There `actrc` file locate at `~/.config/act/actrc` for set the default runner in the file

```bash
-P ubuntu-latest=ubuntu:22.04
-P ubuntu-22.04=node:16-bullseye-slim
```

## Composite actions


### Simple demo
Using `act` to run the demo locally:

```bash
.github
    ├── actions
    │   └── my_action
    │       └── action.yaml
    └── workflows
        ├── demo.yml
        └── README.md
```


```yaml title="workflows/demo.yml" linenums="1" hl_lines="9-10"
name: demos
on: [workflow_dispatch]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - name: hello world
        run: echo "Hello from act!"
      - name: run my action
        uses: ./.github/actions/my_action
```

```yaml title="actions/my_action/action.yaml"  linenums="1" hl_lines="8"
name: "my Action"
description: "My Action"
runs:
  using: "composite"
  steps:
    - name: hello composite
      run: echo "Hello from my composite act!"
      shell: bash
```

#### usage

```bash
act -j test --bind --directory . --pull=false
```

!!! tip "bind directory"
    Using `--bind --directory .` to bind local directory to the container, it's need to find `actions` folder




---

### Composite with args

```bash
.github
    ├── actions
    │   └── action_with_args
    │       └── action.yaml
    └── workflows
        └── demo.yml
```

```yaml title="workflows/demo.yml" linenums="1" hl_lines="9-12"
name: demos
on: [workflow_dispatch]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - name: hello world
        run: echo "Hello from act!"
      - name: run my action
        uses: ./.github/actions/action_with_args
        with:
          arg1: "value 1"
```


```yaml title="actions/action_with_args/action.yaml"  linenums="1" hl_lines="3-10"
name: "my Action with args"
description: "My Action"
inputs:
  arg1:
    description: "arg1"
    required: true
  arg2:
    description: "arg2"
    required: false
    default: 'default value 2'
runs:
  using: "composite"
  steps:
    - name: print arg1
      run: 'echo "print arg1 value: ${{ inputs.arg1 }} "'
      shell: bash
    - name: print arg2
      run: 'echo "print arg2 value: ${{ inputs.arg2 }} "'
      shell: bash
```

---

### Composite with outputs

```bash

```bash
.github
    ├── actions
    │   └── action_output
    │       └── action.yaml
    └── workflows
        └── demo.yml
```

```yaml title="workflows/demo.yml" linenums="1" hl_lines="8 11"
name: demos
on: [workflow_dispatch]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - name: call action that return output
        id: call_action
        uses: ./.github/actions/action_output
      - name: use the output
        run: 'echo "print output value: ${{ steps.call_action.outputs.my_output }} "'
        shell: bash
```

```yaml title="actions/action_output/action.yaml"  linenums="1" hl_lines="3-4 11"
name: "my output demo"
description: "return output from action"
outputs:
  my_output:
    description: "my output"
    value: ""
runs:
  using: "composite"
  steps:
    - name: calc output
      run: echo "my_output=my output value from sub action" >> $GITHUB_OUTPUT
      shell: bash
```

#### usage

```bash 
act -j test --bind --directory . --pull=false
```