---
tags:
  - github
  - actions
  - act
  - git
  - devops
---

# Run github action locally

GitHub Actions help automate tasks like building, testing, and deploying in your GitHub repository.
With `act` CLI tool we can write and test the GitHub action locally.

```
curl https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash
```

## config github action

- Add `.github/workflows` folder to project root
- Add yaml file and declare jobs


### view

from project root

```
act -l
```

### example

Run job on local docker

```
act -j build_demo -P name=docker_image --pull=false
```

| arg    | desc                                   |
| ------ | -------------------------------------- |
| -j     | job to run for workflow                |
| -P     | custom image to                        |
| --pull | pull image from hub or find it locally |


#### Workflow

```yml
name: My Action
on: [workflow_dispatch]
jobs:
  build_demo:
    runs-on: rome_arm
    steps:
      - name: Hello, World!
        run: echo "build job hello world"
```

| arg  | desc  |
|---|---|
| name  | workflow name  |
| on  | running trigger event [push, pull_request, workflow_dispatch]  |
| jobs  | workflow jobs  |
| runs-on | platform to run on (image name declare in **act** command )


!!! tip "multiple lines"
    use `|` to mark multiple lines

    ```yaml
    steps:
      - name: colcon
        run: | 
            line 1
            line 2
    ```
     
---

## Reference

- [How to Run GitHub Actions Locally Using the act CLI Tool](https://www.freecodecamp.org/news/how-to-run-github-actions-locally/)
