### zellij

#### Install
Download binary (tar) extract and run
[Download binary from github releases](https://github.com/zellij-org/zellij/releases)

#### Layout

```
layout {
    pane split_direction="vertical" {
        pane
        pane
    }
    pane split_direction="vertical" {
        pane
        pane
    }
}
```

```bash
zellij --layout /path/to/layout.kdl
```