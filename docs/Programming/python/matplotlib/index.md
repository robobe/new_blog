---
title: Matplotlib 
tags:
    - python
    - matplotlib
---

{{ page_folder_links() }}


## Hello matplotlib

```python
--8<-- "docs/Programming/python/matplotlib/code/hello.py"
```

![alt text](images/hello_matplotlib.png)

---

## Figure

![alt text](images/matplotlib_figure_parts.png)

The whole figure. The Figure keeps track of all the child Axes, a group of 'special' Artists (titles, figure legends, colorbars, etc.), and even nested subfigures.


### Axes

An Axes is an **Artist** attached to a Figure that contains a region for **plotting data**, and usually includes two (or three in the case of 3D) Axis objects

### Axis


### Artist

---

<div class="grid-container">
    <div class="grid-item">
        <a href="3d">
        <!-- <img src="images/vscode.png" width="150" height="150"> -->
        <p>3D plot</p>
        </a>
    </div>
    <div class="grid-item">
       <a href="animation">
        <!-- <img src="images/python.png" width="150" height="150"> -->
        <p>Animation</p>
        </a>
    </div>
    <div class="grid-item">
       <a href="gui">
        <!-- <img src="images/matplotlib.png" width="150" height="150"> -->
        <p>Embedded in QT</p>
        </a>
    </div>
    
</div>

---

### figure and subplots

```python
--8<-- "docs/Programming/python/matplotlib/code/subplots.py"
```

![alt text](images/subplots.png)


```python
--8<-- "docs/Programming/python/matplotlib/code/subplots_1.py"
```

![alt text](images/subplots_1.png)