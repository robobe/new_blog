---
tags:
    - aptly
    - debian
---

# Aptly

Aptly is a swiss army knife for Debian repository management. [](aptly.info){:target="_blank"}

!!! note "install new version"
     
     [install new version 1.6.1](https://www.aptly.info/download/)

---

<div class="grid-container">
        <div class="grid-item">
        <a href="aptly_custom_repository">
            <img src="images/custom_repo.png" width="150" height="150">
            <p>Create custom repository</p>
             </a>
        </div>
    <div class="grid-item">
        <a href="aptly_rest_api">
            <img src="images/rest_api.png" width="150" height="150">
            <p>Using Aptly with REST api</p>
             </a>
    </div>
    <div class="grid-item">
        <p>TBD</p>
    </div>

</div>

---

## gpg keys
# TODO: how to create and assign key

## Aptly Repository
An Aptly repository is a collection of .deb packages that you create and manage locally using Aptly. Think of it like a staging area — it's where you add, update, or remove packages.

Repositories type

- local: custom repos.
- mirror: mirror official repos
  
## Aptly Snapshot

A snapshot is a frozen, read-only version of a repository (local or mirror) at a specific point in time. It’s like taking a picture of the repository.


---

## Serve

```
aptly serve
```

### Using nginx
Nginx (pronounced "engine-x") is a high-performance web server

```
server {
    listen 80;
    server_name myrepo.example.com;

    root /home/user/.aptly/public;

    location / {
        autoindex on;
    }
}
```

### Add to sources.list

```
deb http://myrepo.example.com/ jammy main
# Without sign key
deb [trusted=yes] http://myrepo.example.com/ jammy main
```