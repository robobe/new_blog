---
title: PyBullet tutorials and learning
tags:
    - pybullet
    - tutorial
---


{{ page_folder_links() }}

- URDF
- Types of joints
- Robot Coordination
- Task Space
- Joint/Configuration space
- Base position, orientation


### urdf
[online play](http://mymodelrobot.appspot.com/5629499534213120)

### Types of joints 


- **Fixed**: rigid connection, no motion
- **Revolute**: support rotation in 1 dimension (along a single axis)
- **Continuous**: unlimited variant of revolute joints
- **Prismatic**: support translation in 1 dimension (along a single axis)
- **Planar**: translation in two dimensions
- **Floating**: unlimited motion (translation and rotation) in all 6 dimentions

![alt text](images/common_joint_types.png)


| PyBullet Constant     | Integer | Meaning               |
| --------------------- | ------- | --------------------- |
| `p.JOINT_REVOLUTE`    | 0       | Standard hinge joint  |
| `p.JOINT_PRISMATIC`   | 1       | Linear slider         |
| `p.JOINT_SPHERICAL`   | 2       | 3-DOF ball joint      |
| `p.JOINT_PLANAR`      | 3       | Planar joint          |
| `p.JOINT_FIXED`       | 4       | Fixed (no movement)   |
| `p.JOINT_POINT2POINT` | 5       | P2P constraint        |
| `p.JOINT_GEAR`        | 6       | Gear ratio constraint |


## Demo: RRBot

<details>
    <summary>urdf</summary>

```xml
--8<-- "docs/Simulation/PyBullet/tutorials/code/rrbot/rrbot.urdf"
```
</details>


### Joint name and types

```python
--8<-- "docs/Simulation/PyBullet/tutorials/code/rrbot/joint_name_types.py"
```

```bash title="output"
---
Total joints: 2
0 joint_1 | type = 0
1 joint_2 | type = 0
---
```

---

## Reference
- [PyBullet workshop](https://github.com/Robotics-Club-IIT-BHU/Robotics-Club-x-NTU-MAERC-collab/tree/main)