---
title: Matplotlib animation
tags:
    - matplotlib
    - animation
---

{{ page_folder_links() }}

**FuncAnimation** is Matplotlib’s built-in way to make animations. You give it:

- a Figure to draw on,
- an update function that changes existing artists (lines, scatters, images) for each frame,
- an optional init function to set the starting state,
- and frames (how many frames or an iterable of frame values).

It repeatedly calls your update function and redraws.

```python
--8<-- "docs/Programming/python/matplotlib/animation/code/simple.py"
```

!!! Note
    The callable `update` must return an `iterable` of artist when `blit=True`


---

## 
Demo: Dynamic graph 
- Moving x axis

<details>
<summary>Code</summary>
```python
--8<-- "docs/Programming/python/matplotlib/animation/code/moving_x_axis.py"
```
</details>