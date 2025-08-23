---
title: Numpy reshape
tags:
    - numpy
    - reshape
---

{{ page_folder_links() }}

### methods
- reshape
- flatten
- ravel: flattens a multi-dimensional array into a 1D array.


## reshape
It returns a view of the same data with a new shape when possible (no copy).

- The number of elements must stay the same: `np.prod(a.shape) == np.prod(new_shape)`.
- Dtype doesn’t change.


!!! note "np.prod"
    numpy.prod() computes the product of the array elements over a specified axis.
     

```python
a = np.arange(12)
a.shape
# (12,)

a = a.reshape(3,4)
a.shape
# shape (3,4)

# A placeholder that tells NumPy to infer that dimension so the total size matches.
a = a.reshape(-1)
a.shape
# (12,)

# error
a = a.reshape(3,3)
# cannot reshape array of size 12 into shape (3,3)
```

```python title="reshape view vs copy"
a = np.arange(12)
a = a.reshape(3,4)

# View
r = a.reshape(-1)
r[0] = 10
a[0,0]
# np.int64(10)


# flat copy
r = a.flatten()
r[0] = 10
a[0,0]
# np.int64(0)

# View if possible
r = np.ravel(a)
r[0] = 10
a[0,0]
# np.int64(0)
```


```python
a = np.arange(12)
a = a.reshape(1,6,2)
a. shape
# (1, 6, 2)


a = np.arange(12)
a = a.reshape(1,6,2)
a = np.ravel(a)
a.shape
# (12,)
```