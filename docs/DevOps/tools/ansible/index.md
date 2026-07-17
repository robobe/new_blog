---
title: Ansible
tags:
    - ansible
    - automation
    - devops
---

# Ansible

Ansible is an automation tool that configures computers over SSH.

Instead of manually typing commands on every machine, you describe the desired
state in a YAML file called a playbook, and Ansible makes the machine match that
state.

In this section, Ansible is also used in a different way: it configures a
mounted Raspberry Pi image through a chroot connection, not through SSH.

## Install

```bash
sudo apt install ansible
```

<div class="grid-container">
        <div class="grid-item">
                <a href="build_raspberry_pi_image">
                <p>Build Raspberry Pi image with Ansible</p>
                </a>
            </div>
</div>

---

## Ansible playbook in nutshell

A playbook is a YAML file that tells Ansible what state a host should have.

Basic idea:

- `hosts`: which machines from the inventory to configure
- `become`: run tasks with sudo
- `tasks`: ordered steps to apply
- each task usually calls one Ansible module

Good playbooks are usually idempotent: running them again should not break the
machine or repeat work that is already done.

```yaml title="simple_playbook.yml"
---
- name: Configure server
  hosts: all
  become: true

  tasks:
    - name: Install nginx
      ansible.builtin.apt:
        name: nginx
        state: present
        update_cache: true

    - name: Create welcome page
      ansible.builtin.copy:
        content: "hello from ansible\n"
        dest: /var/www/html/index.html
        mode: "0644"
```

Run it:

```bash
ansible-playbook -i inventory.yml simple_playbook.yml
```

### Common task examples

Install packages:

```yaml
- name: Install packages
  ansible.builtin.apt:
    name:
      - git
      - tmux
    state: present
```

Copy a file:

```yaml
- name: Copy config
  ansible.builtin.copy:
    src: app.conf
    dest: /etc/app.conf
    mode: "0644"
```

Create a directory:

```yaml
- name: Create app directory
  ansible.builtin.file:
    path: /opt/app
    state: directory
    mode: "0755"
```

Run a command:

```yaml
- name: Print kernel
  ansible.builtin.command:
    cmd: uname -r
```

Manage a service:

```yaml
- name: Enable and start nginx
  ansible.builtin.service:
    name: nginx
    state: started
    enabled: true
```

### Most used tasks

| Goal | Module | Short example |
|---|---|---|
| Install package | `ansible.builtin.apt` | `name: nginx`, `state: present` |
| Copy file | `ansible.builtin.copy` | `src: app.conf`, `dest: /etc/app.conf` |
| Create directory | `ansible.builtin.file` | `path: /opt/app`, `state: directory` |
| Run command | `ansible.builtin.command` | `cmd: uname -r` |
| Manage service | `ansible.builtin.service` | `name: nginx`, `state: started` |
| Template config | `ansible.builtin.template` | `src: app.conf.j2`, `dest: /etc/app.conf` |
