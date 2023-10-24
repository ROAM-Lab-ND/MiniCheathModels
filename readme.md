## Various Models for MIT Mini Cheetah
This package incorporates a few models (full-order and reduced-order) of the MIT Mini Cheath Robots. 

### Features
- Kinematic tree built using [spatial_v2_extended](https://github.com/ROAM-Lab-ND/spatial_v2_extended) (extension of Feathersone's algorithm)
- [Whole-Body KKT Dynamics](https://arxiv.org/abs/2006.08102)
- [Single-Rigid-Body Dynamics](https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf?sequence=2&isAllowed=y)
- Variations of all models above that incoporate **motor rotors**
- Functions to quickly generate C++ interfaces, including forward kinematics, dynamics, and first-order dynamics derivatives (using automatic differentiation)

### Dependencies
- [spatial_v2_extended](https://github.com/ROAM-Lab-ND/spatial_v2_extended)
- [Casadi](https://web.casadi.org/get/)
- [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html)(optional for the visulization )