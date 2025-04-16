---
tags:
    - github actions
    - workflow
    - controlling
    - condition
    - strategy
    - matrix
---

# Controlling Workflow and Job Execution
GitHub Actions provides several ways to control the execution of workflows and jobs. This includes using conditions, strategies, and matrix builds. Below are some examples of how to use these features effectively.


## Strategy
The `strategy` keyword allows you to define a strategy for running jobs in parallel or sequentially. You can also use it to define a matrix of values for jobs to run with different configurations.

### Example: Using Strategy
- Run jobs in parallel with different configurations using a matrix strategy.
  
```yaml
name: demos
on: [workflow_dispatch]
jobs:

  strategy_demo_job:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        os: [ubuntu-latest, macos-latest, windows-latest]
    steps:
      - run: echo "Running on ${{ matrix.os }}"
```

## condition
The `if` condition allows you to control whether a job or step runs based on the result of previous jobs or steps. This is useful for skipping jobs or steps that are not needed based on certain conditions.

```yaml title="if condition"
name: demos
on: [workflow_dispatch]
jobs:

  strategy_demo_job:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        os: [ubuntu-latest, macos-latest, windows-latest]
    steps:
      - run: echo "Running on ${{ matrix.os }}"
      - name: if demo
        if: ${{ matrix.os == 'ubuntu-latest' }}
        run: echo "This step runs only on ubuntu-latest"
```
