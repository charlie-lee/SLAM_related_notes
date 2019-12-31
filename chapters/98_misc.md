# Miscellaneous

## 1. Basics

### 1.1 3D Rotation

#### 1.1.1 Rotation Representations
a. \( \V{R} \) itself
   - \( 3 \times 3 \) matrix
b. Axis/angle
   - Represent rotation by rotation axis \( \V{\hat{n}} \) 
     and angle \( \theta \)
   - Derivation of \( \V{R}(\V{\hat{n}}, \theta) \):  
     ![axis_angle_deriv](images/ch98/01_axis_angle_derivation.jpg){ width=30% }
     - A rotation from \( \V{v} \) to \( \V{u} \) based on rotation axis
       \( \V{\hat{n}} \) and rotation angle \( \theta \)
     - \( [\V{\hat{n}}]_{\times} \): matrix form of cross product operator
       with vector \( \V{\hat{n}} = (\hat{n}_x, \hat{n}_y, \hat{n}_z) \)
     - \(
          [\V{\hat{n}}]_{\times} =
          \begin{bmatrix}
                    0 & -\hat{n}_z &  \hat{n}_y \\
            \hat{n}_z &          0 & -\hat{n}_x \\
           -\hat{n}_y &  \hat{n}_x &          0
          \end{bmatrix}
       \)
     - \( \V{u} = \V{u}_{\perp} + \V{v}_{\parallel} 
                = \V{R}(\V{\hat{n}}, \theta) \V{v} \)
     - \(
          \V{R}(\V{\hat{n}}, \theta) 
          = \V{I} + \sin \theta [\V{\hat{n}}]_{\times} +
            (1 - \cos \theta)[\V{\hat{n}}]_{\times}^2
       \)
       - Known as _Rodrigues' formula_
c. Quaternions
   - 4-vector: \( \V{q} = (q_x, q_y, q_z, q_w) \) or \( \V{q} = (x, y, z, w) \)
   - Unit quaternions: live on the sphere \( ||\V{q}|| = 1 \)
   - _Antipodal_ (opposite sign) quaternions \( \V{q} \) and \( -\V{q} \)
     represent the same rotation
   - Representation is _continuous_: rotation matrices vary continuously
   - Quaternions can be derived from the axis/angle representation:
     - \(
          \V{q} 
          = (\V{v}, w) 
          = (\sin \frac {\theta} {2} \V{\hat{n}}, \cos \frac {\theta} {2})
       \)
       where \( \V{\hat{n}} \) and \( \theta \) are rotation axis and angle
   - Convert Rodrigues' formula using double-angle formulas:  
     \(
        \V{R}(\V{\hat{n}}, \theta) 
        = \V{I} + \sin \theta [\V{\hat{n}}]_{\times} +
          (1 - \cos \theta)[\V{\hat{n}}]_{\times}^2
        = \V{I} + 2 w [\V{v}]_{\times} + 2 [\V{v}]_{\times}^2
     \)
   - \( 
        \V{R}(\V{q}) = 
        \begin{bmatrix}
         1 - 2(y^2 + z^2) &       2(xy - zw) &       2(xz + yw) \\
               2(xy + zw) & 1 - 2(x^2 + z^2) &       2(yz - xw) \\
               2(xz - yw) &       2(yz + xw) & 1 - 2(x^2 + y^2)
        \end{bmatrix}
     \)  
     where \( \V{v} = (x, y, z) \) and 
     \( [\V{v}]_{\times} = 
        \begin{bmatrix}
          0 & -z &  y \\
          z &  0 & -x \\
         -y &  x &  0
        \end{bmatrix} \)
   - Quaternion multiply operator:  
     \( \V{q}_2 = \V{q}_0 \V{q}_1
                = (\V{v}_0 \times \V{v}_1 + w_0 \V{v}_1 + w_1 \V{v}_0, 
                   w_0 w_1 - \V{v}_0 \cdot \V{v}_1)
     \)
     - \( \V{R}(\V{q}_2) = \V{R}(\V{q}_0) \V{R}(\V{q}_1) \)
     - Multiply operator is _not commutative_
   - Quaternion division: just flip the sign of \( \V{v} \) or \( w \)
     in the multiply operator  
     \( \V{q}_2 = \V{q}_0 / \V{q}_1 = \V{q}_0 \V{q}_1^{-1}
                = (\V{v}_0 \times \V{v}_1 + w_0 \V{v}_1 - w_1 \V{v}_0, 
                   -w_0 w_1 - \V{v}_0 \cdot \V{v}_1)
     \)
d. Euler angles
   - 3 angles to describe the orientation of a rigid body w.r.t. 
     a fixed coordinate system
     - \( \alpha \), \( \beta \), \( \gamma \); or
       \( \phi \), \( \theta \), \( \psi \)
   - 2 groups of angles with 12 possible sequences of rotation axes
     (1) **Proper/Classic Euler angles**: 
         _z_-_x_-_z_, _x_-_y_-_x_, _y_-_z_-_y_, 
         _z_-_y_-_z_, _x_-_z_-_x_, _y_-_x_-_y_
     (2) **Tait-Bryan angles**: 
         _x_-_y_-_z_, _y_-_z_-_x_, _z_-_x_-_y_, 
         _x_-_z_-_y_, _z_-_y_-_x_, _y_-_x_-_z_
         - Also called: **Cardan angles**; **nautical angles**; 
           **heading, elevation, and bank**; **yaw, pitch, and roll**
   - Proper Euler angles
     - Geometrical definition (one possibility):  
       ![Euler_angles](images/ch98/02_Euler_angles.svg){ width=35% }
       - _x_, _y_, _z_ : axes of the original frame
       - _X_, _Y_, _Z_: axes of the rotated frame
       - \( N = z \times Z \) (i.e., the intersection of planes _xy_ and _XY_)
       - \( \alpha \), \( \beta \), \( \gamma \): Euler angles
     - Definition by intrinsic rotations
       - Intrinsic rotations: 
         _XYZ_ system rotates after each elementary rotation; 
         _xyz_ system is fixed
       - Rotated frame _XYZ_ from initial orientation to final orientation:
         - Initial: 
           \( x \)-\( y \)-\( z \) or \( x_0 \)-\( y_0 \)-\( z_0 \)
         - After 1st rotation: 
           \( x' \)-\( y' \)-\( z' \) or \( x_1 \)-\( y_1 \)-\( z_1 \)
         - After 2nd rotation:
           \( x'' \)-\( y'' \)-\( z'' \) or \( x_2 \)-\( y_2 \)-\( z_2 \)
         - Final (after 3rd rotation):
           \( X \)-\( Y \)-\( Z \) or \( x_3 \)-\( y_3 \)-\( z_3 \)
       - Definition of Euler angles:
         - \( \alpha \) (or \( \phi \)): a rotation around the \( z \) axis
         - \( \beta \) (or \( \theta \)): a rotation around the \( x' \) axis
         - \( \gamma \) (or \( \psi \)): a rotation around the \( z'' \) axis
      - Definition by extrinsic rotations
        - Extrinsic rotations: elemental rotations occur about the axes of
          fixed coordinate system _xyz_
        - Target orientation can be achieved by rotating \( \gamma \), 
          \( \beta \), and \( \alpha \) around _z_, _x_, _z_ axes respectively
   - Tait-Bryan angles
     - Difference with proper Euler angles: Tait-Bryan angles represent 
        rotations about 3 distinct axes (e.g. _x_-_y_-_z_ or 
        \( x \)-\( y' \)-\( z'' \)), while proper Euler angles use same axis
        for both 1st & 3rd elemental rotations (e.g. _z_-_x_-_z_ or
        \( z \)-\( x' \)-\( z'' \))
     - Geometrical definition (one possibility):  
       ![TB_zyx_angles](images/ch98/03_TaitBryan_zyx_angles.svg){ width=35% }
       - Different line of nodes compared with proper Euler angles
         - Intersection of planes _xy_ and _YZ_ for Tait-Bryan angles vs. 
           _xy_ and _XY_ for proper Euler angles
       - Rotation sequence: \( z \)-\( y' \)-\( x'' \)
         - Intrinsic rotations; \( N \) coincides with \( y' \)
         - Known as **yaw, pitch, and roll**
       - Angle rotation sequence: \( \psi \), \( \theta \), \( \phi \)

#### 1.1.2 Conversions Between Different Rotation Representations
- References: 
  - [wiki page](https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions)
- Rotation matrix decomposition ([wiki](https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix)):
  - Any orientation can be achieved by composing 3 elemental rotations
  - Example: \( \V{R} = \V{X}(\alpha) \V{Y}(\beta) \V{Z}(\gamma) \)
    - 2 possible representations
      (1) A composition of **extrinsic** rotations about axes _z_-_y_-_x_
      (2) A composition of **intrinsic** rotations about axes
          \( x \)-\( y' \)-\( z'' \)
  - Rotation matrix from all 12 kinds of Euler angles:  
    ![rotmat_euler](images/ch98/04_rotmat_euler.jpg)
  - Rotation matrix to yaw, pitch, roll in Eigen ([ref](https://eigen.tuxfamily.org/dox/group__Geometry__Module.html#title40)):
    - `R.eulerAngles(2, 1, 0).reverse; `{.cpp}

\newpage
