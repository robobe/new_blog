---
tags:
    - ros
    - deploy
    - tutorial
---

# ROS2 project from development to deployment, Part 2: Add udev and other system files

How to add additional system files to the package, like udev rules, systemd service, etc.


## Demo: Add udev rules
!!! note "github action"
    We use github actions and `act` to run it locally.

     
Add udev rule to `/etc/udev/rules.d/` 

- create udev rules file (put it in project root under `udev` folder)
- Add install script, this script **update** debian `rules` file after bloom generate
- Add step to github actions command

```bash title="udev rules"
# any udev rules file
```

```bash title="post_bloom_script.sh"
#!/bin/bash

cat <<EOF >> src/<package_name>/debian/rules
    mkdir -p \$(DEBIAN)/etc/udev/rules.d
    cp -r udev/* \$(DEBIAN)/etc/udev/rules.d
    dh_install
EOF
```

!!! note "check the $(DEBIAN) path"
    The `$(DEBIAN)` path is the path where the debian package is created. You can check it by running the following command:

!!! tip "TAB not space"
    Make sure to use tab and not space in the `post_bloom_script.sh` file. You can check it using `cat -T` command.


     

```yaml title="github action step"
- name: Add udev rules
  run: |
    ./src/<package_name>/post_bloom_script.sh
```


!!! tip "TAB VSCode"
    Config vscode to use tab and not spaces for shell script


    ```json title="set vscode to use tab and not spaces"
    {
        "[shellscript]": {
            "editor.insertSpaces": false,
            "editor.tabSize": 4,
            "editor.detectIndentation": false
        },
    }
    ```

!!! tip "TAB check"
    Using cat to check if file contains tab or space

    ```bash
    cat -T <file>
    ```

    ```bash
    #!/bin/bash

    cat <<EOF
    ^Ia
    ^Ib
    EOF
    ```
     