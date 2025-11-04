---
Title: Using vscode as rust IDE
tags:
    - vscode
    - rust
---


{{ page_folder_links() }}

## VSCode Ext.

| Extension  | Desc  | Install  |
|---|---|---|
| rust-analyzer  | Rust language support for Visual Studio Code  | `ext install rust-lang.rust-analyzer`  |
| codeLLDB  | A native debugger powered by LLDB. Debug C++, Rust and other compiled languages.  | `ext install vadimcn.vscode-lldb`  |
| Even Better TOML  | Fully-featured TOML support  | `ext install tamasfe.even-better-toml`  |
|   |   |   |
| **optional**  |   |   |
| Error Lens  | Improve highlighting of errors, warnings and other language diagnostics.  | `ext install usernamehw.errorlens`  |
| Dependi   |  manage dependencies  |  `ext install fill-labs.dependi` |


---

## Rust tools
- **rustfmt**: format rust code
- **Clippy**: static analyzer / linter
- **cargo fix**: automatically applied fixed to turustt code base on compiler diagnostics, useful when upgrading rust versions

### rustfmt

```
rustup component add rustfmt
```

```
cargo fmt
```

### Clippy

```
rustup component add clippy
```

```
cargo clippy
```

```json title="vscode settings"
"rust-analyzer.check.command": "clippy"
```

### cargo fix

```bash
#allow-dirty: Fix code even if the working directory is dirty or has staged changes
cargo fix --allow-dirty
```

---

!!! tip "VSCode profile"

    Using vscode to handle profile for each type of work
    Profiles in VS Code let you create separate environments with their own:

        - Extensions
        - Settings
        - Themes
        - UI layout
        - Keybindings
        - Snippets

    ```
    Profiles: Switch Profile  
    Profiles: New Profile  
    ```
     

---

## References
- [Ultimate VS Code setup for Rust development (2025) ](https://youtu.be/ZhedgZtd8gw)