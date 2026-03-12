---
title: Kalman filter
tags:
    - kalman
    - filter
    - control
---
Kalman filter is an iterative mathematical process that use a set of equations and consecutive data input to quickly estimate the true value, position velocity etc of the object being measured when the measures values contains unpredicted or random error , uncertainty or variation [Michel Van biezen](https://youtu.be/CaCcOwJPytQ)


[Special Topics - The Kalman Filter (1 of 55) What is a Kalman Filter?](https://youtube.com/playlist?list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT&si=lGjt-xsTlZQi77o2)



---

## Video 5/6
[A Simple Example of the Kalman Filter](https://youtu.be/SIQJaqYVtuE?list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT)


$$
KG = \frac{E_{est}}{E_{est}+E_{meas}}
$$

$$
EST_t=EST_{t-1} + KG[MEAS-EST_{t-1}]
$$

$$
E_{est_t}=[1-KG](E_{est_{t-1}})
$$


### Demo:

- true temperature = 72
- init estimate = 68
- init $E_{est}$ = 2
- init measurement = 75
- error in measurement =4


|   | MEA  | $E_{MEA}$  | EST  | $E_{est_{t-1}}$  | KG  | $E_{est_t}$  |
|---|---|---|---|---|---|---|
| t-1  |     | 4 | 68     | 2  |       |   |
| t    | 75  | 4 | 70.33  |    | 0.33  | 1.33  |
| t+1  | 71  | 4 | 70.50  |    | 0.25  | 1.00  |
| t+2  | 70  | 4 | 70.40  |    | 0.20  | 0.80  |
| t+3  | 74  | 4 | 71     |    | 0.17  | 0.66  |


![1d](images/kalman_1d.png.png)

<details>
<summary>kalman 1d</summary>
```
--8<-- "docs/Robotics/filter_and_estimator/kalman_filter/code/kalman_1d.py"
```
</details>


---

## Reference
- [hummingbird - kalman filter](https://www.youtube.com/playlist?list=PLgG0XDQqJckmfolmM8y0GFS8l3x_r2p_S)
- [Why Use Kalman Filters? | Understanding Kalman Filters, Part 1](https://youtu.be/mwn8xhgNpFY?list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr)