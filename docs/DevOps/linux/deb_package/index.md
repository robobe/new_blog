---
tags:
    - debian
    - package
    - linux
    - ubuntu
---

Create debian package for shell script, python project and other using debian tools  
Create custom install wizard using debconf and postinst scripts

<div class="grid-container">
        <div class="grid-item">
        <a href="debconf">
            <img src="images/debian_package.png" width="150" height="150">
            <p>debconf</p>
             </a>
        </div>
    <div class="grid-item">
        <a href="shell_script">
            <img src="images/pack_shell" width="150" height="150">
            <p>pack shell script</p>
             </a>
    </div>
    <div class="grid-item">
        <p>TBD</p>
    </div>

</div>

![alt text](image.png)

# Debian package

!!! tip "dpkg-deb"
    Basic tool to create debian package

!!! tip "dpkg-buildpackage"
    Debian has some pretty detailed rules on what a "proper" Debian package looks like, **dpkg-buildpackage** is a tool that enforces said workflow and structure, it's use `dpkg-deb` to create debian package (deb file)


!!! warning "Version"
    The "Version" field is gone. As explained above, dpkg-buildpackage infers the version number from the changelog file.
     

## Debhelper
Debhelper is a tool that automates various common aspects of package building

### command/method

#### dh_auto_configure

automatically run `./configure` or the cmake equivalent.  
- Applies architecture-specific flags.( CFLAGS, CXXFLAGS, LDFLAGS) |

#### dh_builddeb
dh_builddeb is a Debhelper tool that creates a Debian (.deb) package from the files staged in debian/tmp or debian/<package_name>/

| option   | desc  |
|---|---|
| `--destdir=<dir>`  | Change the output directory for .deb files.  |


```make
override_dh_builddeb:
	dh_builddeb --destdir=output_folder

```


--- 

## Resource
- [Debian Packaging For The Modern Developer (DPMD)](https://github.com/FooBarWidget/debian-packaging-for-the-modern-developer/tree/master)