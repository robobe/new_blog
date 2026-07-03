---
title: Codex
tags:
    - vscode
    - codex
    - chatgpt
---    

## Troubleshooting

**codex freeze and don't bring up the prompt area**

```bash
# close VS Code first
rm -rf ~/.codex/.tmp
rm -rf ~/.codex/cache/remote_plugin_catalog
# reopen
```