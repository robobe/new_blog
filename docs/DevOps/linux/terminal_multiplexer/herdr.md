---
title: Herdr with Codex
tags:
    - herdr
    - codex
    - ai-agent
    - terminal-multiplexer
---

# Herdr with Codex

[Herdr](https://herdr.dev/) is a terminal multiplexer designed for AI coding
agents. Like `tmux`, it keeps terminal processes alive after you disconnect, but
it also detects agents such as Codex and shows whether each one is working,
waiting for input, done, or idle.

This makes Herdr useful when you:

- run Codex in more than one repository;
- want several long-running agent sessions in one terminal;
- connect to a development machine through SSH; or
- want to close the terminal and return to the same work later.

!!! note
    Herdr manages terminal sessions; it does not replace Codex. Codex still does
    the coding work, while Herdr organizes and preserves the terminal panes in
    which Codex runs.

This guide was inspired by [this YouTube video](https://youtu.be/2CR9tDNAzB0).
The commands below follow the current [official Herdr documentation](https://herdr.dev/docs/).

## Concepts

Herdr organizes the terminal in four levels:

| Level | Purpose |
|---|---|
| Session | The background Herdr server that survives detach/reattach |
| Workspace | A project or repository |
| Tab | A group of related terminal panes inside a workspace |
| Pane | A real shell, Codex session, editor, log viewer, or other process |

The sidebar groups detected agents and displays their state. A Codex pane may be
shown as `working`, `blocked`, `done`, or `idle`, making it easy to see which
session needs attention.

## Install Herdr

Herdr provides stable builds for Linux and macOS. Native Windows support is
currently a preview.

### Linux or macOS

```bash
curl -fsSL https://herdr.dev/install.sh | sh
```

Review remote install scripts before piping them into a shell. The alternative
is to download the correct binary from the
[Herdr releases page](https://github.com/ogulcancelik/herdr/releases), mark it
executable, and place it on your `PATH`.



## Configure Herdr for Codex

First, make sure Codex is already installed and has been run at least once:

```bash
codex --version
codex
```

Running Codex once creates its configuration directory (`~/.codex` by default).
Exit Codex, then install the official Herdr integration:

```bash
herdr integration install codex
herdr integration status
```

The installer updates the Codex hook configuration rather than your API or
login credentials. It writes a Herdr hook under `~/.codex`, updates
`~/.codex/hooks.json`, and enables Codex hooks in `~/.codex/config.toml`. If you
set `CODEX_HOME`, Herdr uses that directory instead.

The integration gives Herdr the native Codex session identity needed to resume
the conversation after a Herdr server restart. Agent state detection still
comes from what Codex displays in its terminal pane.

!!! tip
    Herdr can detect a running `codex` process without the integration, but the
    integration provides better session restore behavior.

To remove the integration later:

```bash
herdr integration uninstall codex
```

See the official [Codex integration details](https://herdr.dev/docs/integrations/#codex)
for the files changed by the current Herdr release.

## Basic usage with Codex

Open a repository and start Herdr:

```bash
cd ~/projects/my-project
herdr
```

Herdr creates or attaches to its background session. In a pane, start Codex as
usual:

```bash
codex
```

Herdr detects Codex automatically and adds it to the agents section of the
sidebar. You can now interact with Codex exactly as you would in a normal
terminal.

### Herdr config

Change Herdr’s prefix in its configuration file:

```bash
mkdir -p ~/.config/herdr
nano ~/.config/herdr/config.toml
```

#### Changed prefix key
Add or update:

```ini
[keys]
prefix = "ctrl+a"
```

 Reload the configuration:

```bash
herdr server reload-config
```




---

### Useful keys

Herdr is mouse-friendly, but keyboard actions use a prefix. `prefix+c` means
press `ctrl+b`, release both keys, and then press `c`.

| Action | Default key |
|---|---|
| Show active keybindings | `prefix+?` |
| Split side by side | `prefix+v` |
| Split above/below | `prefix+minus` |
| Create a tab | `prefix+c` |
| Next / previous tab | `prefix+n` / `prefix+p` |
| Open workspace navigation | `prefix+w` |
| Create a workspace | `prefix+shift+n` |
| Zoom the focused pane | `prefix+z` |
| Close the focused pane | `prefix+x` |
| Detach | `prefix+q` |

For example, use one pane for Codex and a second pane for tests or logs:

```text
Codex pane                         Test pane
-----------------------------     -----------------------------
codex                             pytest -q
```

### Detach and return

Press `prefix+q`, or close the terminal window. Herdr and Codex continue running
in the background. Return later with:

```bash
herdr
```

Detaching is different from stopping. To end the default Herdr session and all
processes in its panes, run:

```bash
herdr server stop
```

!!! warning
    Stopping the server exits the pane processes. Detach when you want Codex to
    continue working.

## Optional Herdr configuration

Herdr works without a configuration file. Its Linux and macOS configuration is
stored at `~/.config/herdr/config.toml`. Create the directory and open the file:

```bash
mkdir -p ~/.config/herdr
nano ~/.config/herdr/config.toml
```

!!! warning
    Do not run `herdr --default-config > ~/.config/herdr/config.toml` when the
    file already exists: `>` replaces all existing settings. Use
    `herdr --default-config` to inspect the defaults, or redirect it only when
    creating a new file.

### Terminator-style keys

The following profile feels familiar to a Terminator user 
.The direct shortcuts open and manage panes without
first pressing a prefix. In particular, `ctrl+shift+x` zooms the current pane
and pressing it again restores the complete layout.

<a href="code/herdr-config.toml" download>Download the Herdr configuration</a>

Install the downloaded file with:

```bash
mkdir -p ~/.config/herdr
cp ~/Downloads/herdr-config.toml ~/.config/herdr/config.toml
herdr server reload-config
```

Review the destination before replacing an existing configuration.

```toml title="~/.config/herdr/config.toml"
onboarding = false

[terminal]
shell_mode = "auto"
new_cwd = "follow"

[session]
resume_agents_on_restore = true

[keys]
prefix = "ctrl+a"

# Terminator-style pane creation:
# Ctrl+Shift+E opens a pane on the right.
# Ctrl+Shift+O opens a pane below.
split_vertical = ["prefix+v", "ctrl+shift+e"]
split_horizontal = ["prefix+minus", "ctrl+shift+o"]

# Move between panes like Terminator, with tmux-style h/j/k/l as a fallback.
focus_pane_left = ["prefix+h", "alt+left"]
focus_pane_down = ["prefix+j", "alt+down"]
focus_pane_up = ["prefix+k", "alt+up"]
focus_pane_right = ["prefix+l", "alt+right"]

# Pane and tab management.
zoom = ["prefix+z", "ctrl+shift+x"]
close_pane = ["prefix+x", "ctrl+shift+w"]
new_tab = ["prefix+c", "ctrl+shift+t"]
next_tab = ["prefix+n", "ctrl+shift+right"]
previous_tab = ["prefix+p", "ctrl+shift+left"]

# Session controls.
detach = "prefix+q"
help = "prefix+?"
toggle_sidebar = "prefix+b"
```

| Action | Terminator-style shortcut | Prefix fallback |
|---|---|---|
| Open pane on the right | `ctrl+shift+e` | `prefix+v` |
| Open pane below | `ctrl+shift+o` | `prefix+minus` |
| Focus another pane | `alt+arrow` | `prefix+h/j/k/l` |
| Zoom or restore one pane | `ctrl+shift+x` | `prefix+z` |
| Close the focused pane | `ctrl+shift+w` | `prefix+x` |
| Open a new tab | `ctrl+shift+t` | `prefix+c` |
| Change tab | `ctrl+shift+left/right` | `prefix+p/n` |

Herdr calls a side-by-side split `split_vertical` because the dividing line is
vertical. Terminator uses `ctrl+shift+e` for the same layout. Similarly,
`split_horizontal` creates a pane above or below with `ctrl+shift+o`.

### Recommended quality-of-life settings

For agent work, keeping the source pane's directory and restoring native Codex
sessions are the most useful defaults:

```toml
[terminal]
shell_mode = "auto"
new_cwd = "follow"

[session]
resume_agents_on_restore = true

[worktrees]
directory = "~/.herdr/worktrees"
```

The worktree directory is useful when multiple Codex agents work on the same
repository: each agent can use a separate checkout instead of editing the same
files. Herdr also supports completion and input-needed notifications; configure
these through its settings screen because the best delivery method depends on
whether the session is local or reached through SSH.

Reload it without closing the panes:

```bash
herdr server reload-config
```

The full configuration file is optional. If you only need the defaults, do not
create it. Refer to the [configuration guide](https://herdr.dev/docs/configuration/)
before adding theme, notification, sidebar, worktree, or custom-key settings.

!!! tip
    Direct shortcuts must pass through the desktop environment and outer
    terminal before Herdr can receive them. If a shortcut does nothing, remove
    the conflicting binding in Terminator, GNOME/KDE, or your current terminal,
    or choose another Herdr chord. Press `prefix+?` to confirm the active Herdr
    bindings.

    Terminator assigns ++alt+left++, ++alt+right++, ++alt+up++, and
    ++alt+down++ to its own `go_left`, `go_right`, `go_up`, and `go_down`
    actions by default. To use those keys inside Herdr, open **Terminator →
    Preferences → Keybindings**, clear or reassign those four Terminator
    bindings, restart Terminator, and reload the Herdr configuration.

### Using Alt+Arrow in the VS Code terminal

When Herdr runs in VS Code's integrated terminal, mouse focus may work while
++alt+arrow++ does not. This means VS Code is handling the shortcut before the
terminal application receives it. Open **Preferences: Open User Settings
(JSON)** from the Command Palette and add:

```json title="settings.json"
{
    "terminal.integrated.sendKeybindingsToShell": true,
    "terminal.integrated.allowMnemonics": false
}
```

Open a new integrated terminal, start Herdr, and try ++alt+arrow++ again. The
first setting sends most keyboard shortcuts to the running program whenever the
terminal has focus. Consequently, some VS Code shortcuts will no longer run
while the terminal is focused; move focus back to an editor before using them.

The `allowMnemonics` setting should remain `false`, otherwise Linux/Windows
++alt++ combinations may activate VS Code menu mnemonics instead of reaching
Herdr. See VS Code's
[keyboard shortcuts and the shell](https://code.visualstudio.com/docs/terminal/advanced#_keyboard-shortcuts-and-the-shell)
documentation for selective alternatives.

## A practical Codex workflow

1. Run `herdr` inside the repository.
2. Start `codex` in the first pane and give it one focused task.
3. Split a second pane for tests, a development server, or Git commands.
4. Create another workspace for a different repository instead of mixing both
   projects in one workspace.
5. Watch the agents sidebar for a blocked or completed Codex session.
6. Detach with `prefix+q` and reattach later with `herdr`.

When running several Codex agents against the same repository, use separate Git
worktrees or clearly separated tasks. Two agents editing the same files can
overwrite or conflict with each other's work.



## References

- [Herdr documentation](https://herdr.dev/docs/)
- [Herdr quick start](https://herdr.dev/docs/quick-start/)
- [Herdr and Codex integration](https://herdr.dev/docs/integrations/#codex)
- [Herdr configuration](https://herdr.dev/docs/configuration/)
- [Herdr source code](https://github.com/ogulcancelik/herdr)
- [YouTube video that inspired this post](https://youtu.be/2CR9tDNAzB0)
