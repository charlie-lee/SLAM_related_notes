# Notes on Multiple View Geometry Book [@Hartley2004] [(site)](http://www.robots.ox.ac.uk/~vgg/hzbook/)

## Part 0 - The Background: Projective Geometry, Transformations and Estimation

### 2. Projective Geometry and Transformations of 2D

#### 2.2 The 2D Projective Plane

##### 2.2.1 Points and Lines
- Homogeneous representation
  - Line in a plane: \( \V{l}: ax + by + c = 0 \) \( \to \) vector 
    \( (a, b, c) \)
    - \( (ka)x + (kb)y + (kc) = 0 \) are the same line for \( k \neq 0 \)
  - Point: \( \V{x} = (x, y) \) \( \to \) \( (x, y, 1) \)
    - \( (x_1, x_2, x_3) \) \( \to \) \( (x_1/x_3, x_2/x_3) \) in \( \F{R}^2 \)
      for \( x_3 \neq 0 \)
- Point on a line: \( \V{x} \cdot \V{l} = 0 \) or \( \V{x}^T \V{l} = 0 \)
- Degrees of freedom (dof): both point & line are 2
- (Point) Intersection of lines: \( \V{x} = \V{l} \times \V{l}' \)
  - \( \V{l} \cdot (\V{l} \times \V{l}') 
       = \V{l}' \cdot (\V{l} \times \V{l}') 
       = 0 \)
    - \( \V{l}^T \V{x} = \V{l}'^T \V{x} = 0 \)
- Line passing through 2 points \( \V{x} \) & \( \V{x}' \): 
  \( \V{l} = \V{x} \times \V{x}' \)
  - \( \V{x} \cdot \V{l} = \V{x}' \cdot \V{l} = 0 \)
  
##### 2.2.2 Ideal Points and the Line at Infinity
- Intersection of parallel lines \( \V{l} = (a, b, c) \) & 
  \( \V{l}' = (a, b, c') \)
  - \( \V{l} \times \V{l}' = (c' - c)(b, -a, 0) \)
    - Intersection: point \( (b, -a, 0) \)
- Ideal point & the line at infinity
  - Homogeneous vectors \( \V{x} = (x_1, x_2, x_3) \)
    - Vectors with \( x_3 \neq 0 \): finite points in \( \F{R}^2 \)
    - Vectors with any \( x_3 \): set of all homogeneous 3-vectors, i.e., 
      projective space \( \F{P}^2 \)
  - Ideal point: \( (b, -a, 0) \) 
    - Set of ideal points: \( (x_1, x_2, 0) \) specified by the ratio 
      \( x_1 : x_2 \)
  - Line at infinity: \( \V{l}_\infty = (0, 0, 1) \)
  - Inhomogeneous notation \( (b, -a) \): represent line's direction
- Projective plane model  
  ![02_fig_01](images/ch04/02_fig_01.jpg){ width=70% }
  - \( \F{P}^2 \) \( \to \) \( \F{R}^3 \)
  - \( k(x_1, x_2, x_3) \)
    - Point in \( \F{P}^2 \): ray through the origin
    - Line in \( \F{P}^2 \): plane passing through the origin
  - 2 points form 1 line \( \to \) 2 rays are on exactly one plane
  - 2 lines intersect at 1 point \( \to \) 2 planes intersect in one ray
- Duality
  - Symmetry between line & point in homogeneous representation
    - \( \V{l}^T \V{x} = 0 \leftrightarrow \V{x}^T \V{l} = 0 \)
  - **_Duality principle_**: 
    To any theorem of 2-dimensional projective geometry 
    there corresponds a dual theorem, which may be derived by 
    interchanging the roles of points and lines in the original theorem
    - Concepts of incidence must be appropriately translated
    - Example: line through 2 points \( \leftrightarrow \) point through 
      (point of intersection 
      of) 2 lines

#### 2.3 Projective Transformations
- Projectivity: an invertible mapping \( h: \F{P}^2 \to \F{P}^2 \)
  such that 3 points \( \V{x}_1 \), \( \V{x}_2 \), and \( \V{x}_3 \) lie on
  the same line if and only if \( h(\V{x}_1) \), \( h(\V{x}_2) \), and 
  \( h(\V{x}_3) \) do
  - Synonym: _collineation_, _projective transformation_, _homography_
    - Collineation: collinear points remain collinear after the mapping
- A mapping \( h: \F{P}^2 \to \F{P}^2 \) is a **_projectivity_** if and only if
  there exists a non-singular matrix \( \V{H} \) such that for any point
  in \( \F{P}^2 \) represented by a vector \( \V{x} \) it is true that
  \( h(\V{x}) = \V{Hx} \)
  - \( \V{x}' = \V{Hx} \) where \( \V{x} \) and \( \V{x}' \) are homogeneous 
    3-vectors
    - \( \begin{pmatrix} x_1' \\ x_2' \\ x_3' \end{pmatrix}
         = \begin{bmatrix} 
            h_{11} & h_{12} & h_{13} \\
            h_{21} & h_{22} & h_{23} \\
            h_{31} & h_{32} & h_{33}
           \end{bmatrix}
           \begin{pmatrix} x_1 \\ x_2 \\ x_3 \end{pmatrix} \)
  - \( \V{H} \): _homogeneous_ matrix (only ratio of matrix elements is 
    significant)
    - DOF: 8 because there are 8 ratios among 9 elements of \( \V{H} \)

##### 2.3.1 Transformations of Lines and Conics
- Transformation of lines under mapping \( \V{H} \)
  \[
     \V{l}' = \V{H}^{-T} \V{l}
  \]
  - \( \V{x} \) lies on line \( \V{l} \) and \( \V{x}' = \V{H} \V{x} \) lies
    on line \( \V{l}' \)
    - \( \V{x}'^T \V{l}' = \V{x}^T \V{l} = 0 \)  
      \( \V{x}^T \V{H}^T \V{l}' = \V{x}^T \V{l} \)  
      \( \V{l}' = \V{H}^{-T} \V{l} \)
    
#### 2.4 A Hierarchy of Transformations
- Projective transformations form a group called _projective linear group_
- \( GL(n) \): (real) general linear group on \( n \) dimensions
  - Group of invertible \( n \times n \) matrices with real elements
- \( PL(n) \): projective linear group
  - Quotient group of \( GL(n) \) (matrices related by a scalar multiplier)
- Important subgroups of \( PL(3) \):
  - **_Affine group_**: matrices whose last row is \( (0, 0, 1) \)
  - **_Euclidean group_**: subgroup of affine group
    - Upper left hand \( 2 \times 2 \) matrix has determinant 1
- Subsequent 4 classes of transformations are from most specialized _class I_ 
  to most generalized _class IV_ transformations

##### 2.4.1 Class I: Isometries
- Isometries are transformations of plane \( \F{R}^2 \)
- Euclidean distance is preserved
- Representation:
  \[ 
     \begin{pmatrix} x' \\ y' \\ 1 \end{pmatrix}
     = \begin{bmatrix}
        \epsilon \cos \theta & - \sin \theta & t_x \\
        \epsilon \sin \theta &   \cos \theta & t_y \\
                           0 &             0 &   1
       \end{bmatrix}
       \begin{pmatrix} x \\ y \\ 1 \end{pmatrix}
  \]
  - \( \epsilon = \pm 1 \)
  - When \( \epsilon = 1 \): 
    - Isometry is orientation-preserving
    - Isometry is an Euclidean transformation (composition of 
      translation & rotation)
  - When \( \epsilon = -1 \): orientation is reversed
- Euclidean transformations model the motion of a _rigid object_
  \[ 
     \V{x}' 
     = \V{H}_E \V{x}
     = \begin{bmatrix} \V{R} & \V{t} \\ \V{0}^T & 1 \end{bmatrix} \V{x} 
  \]
  - \( \V{R} \): \( 2 \times 2 \) rotation matrix (orthogonal matrix: 
    \( \V{R}^T \V{R} = \V{R} \V{R}^T = \V{I} \))
  - \( \V{t} \): translation 2-vector
  - \( \V{0} \): null 2-vector
  - Special cases
    - Pure translation (\( \V{R} = \V{I} \))
    - Pure rotation (\( \V{t} = \V{0} \))
  - Also known as **_displacement_**
  - DOF for planar Euclidean transformation: 3 (1 + 2)
    - 1 for rotation & 2 for translation
    - Transformation can be computed from 2 point correspondences
- Invariants 
  - Length: distance between 2 points
  - Angle: angle between 2 lines
  - Area
- Orientation-preserving transformations form a group while 
  orientation-reversing ones do not

##### 2.4.2 Class II: Similarity Transformations
- Similarity transformation (_similarity_): 
  isometry composed with isotropic scaling
- Representation in case of Euclidean composed with scaling (no reflection)
  (matrix & block matrix form)
  \[
     \begin{pmatrix} x' \\ y' \\ 1 \end{pmatrix}
     = \begin{bmatrix}
        s \cos \theta & -s \theta & t_x \\
        s \sin \theta &  s \theta & t_y \\
                    0 &         0 &   1
       \end{bmatrix}
       \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} \\
     \V{x}' 
     = \V{H}_s \V{x} 
     = \begin{bmatrix} s \V{R} & \V{t} \\ \V{0}^T & 1 \end{bmatrix} \V{x} 
  \]
  - \( s \): isotropic scaling
- Also known as _equi-form_ transformation as it preserves "shape" (form)
- DOF for planar similarity transformation: 4 (1 + (1 + 2))
  - 1 for scaling and 3 for isometry
- Invariants
  - Angle: parallel lines are mapped to parallel lines
  - _Ratio_ of 2 lengths (before & after the transformation)
  - _Ratio_ of 2 areas
- _Metric structure_: the structure is defined up to a similarity

##### 2.4.3 Class III: Affine Transformations
- Affine transformation (_affinity_): non-singular linear transformation
  followed by a translation
- Representation (matrix & block matrix form)
  \[
     \begin{pmatrix} x' \\ y' \\ 1 \end{pmatrix}
     = \begin{bmatrix}
        a_{11} & a_{12} & t_x \\
        a_{21} & a_{22} & t_y \\
             0 &      0 &   1
       \end{bmatrix}
       \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} \\
     \V{x}' 
     = \V{H}_A \V{x} 
     = \begin{bmatrix} \V{A} & \V{t} \\ \V{0}^T & 1 \end{bmatrix} \V{x} 
  \]
  - \( \V{A} \): \( 2 \times 2 \) non-singular matrix
- DOF for planar affine transformation: 6
  - 4 for transformation \( \V{A} \) & 2 for translation
  - Transformation can be computed from 3 point correspondences
- Decomposition of \( \V{A} \) (using SVD decomposition):
  \begin{align*} 
   \V{A} &= \V{U} \V{D} \V{V}^T \\
         &= \V{U} \V{V}^T \V{V} \V{D} \V{V}^T \\
         &= (\V{U} \V{V}^T) (\V{V} \V{D} \V{V}^T) \\
         &= (\V{R}(\theta)) (\V{R}(-\phi) \V{D} \V{R}(\phi)) \\
         &= (\text{rotation}) (\text{deformation})
  \end{align*}
  - \( \V{R}(\theta) \) & \( \V{R}(\phi) \): rotations by \( \theta \) &
    \( \phi \)
  - \( \V{D} \): a diagonal matrix representing non-isotropic scaling
    \[ \V{D} = \begin{bmatrix} \lambda_1 & 0 \\ 0 & \lambda_2 \end{bmatrix} \]
  - \( \V{U} \) & \( \V{V} \) are orthogonal matrices
  - Affine matrix \( \V{A} \) is concatenation of the followings:
    - Deformation (non-isotropic scaling)
      - A rotation by \( \phi \)
      - A non-isotropic scaling by \( \lambda_1 \) & \( \lambda_2 \) 
        respectively in (rotated) \( x \) & \( y \) directions
      - A rotation back by \( \phi \)
    - Rotation
      - A rotation by \( \theta \)
  - Non-isotropic scaling (deformation):
    - Contribute 2 extra DOF to affinity over similarity
      - Angle \( \phi \) specifying the scaling direction
      - Ratio of scaling parameters \( \lambda_1 : \lambda_2 \)
  - Illustration of rotation & deformation  
    ![02_fig_07](images/ch04/02_fig_07.jpg){ width=70% }
- Invariants
  - Parallel lines
  - Ratio of lengths of parallel line segments
  - Ratio of areas
- \( \mathrm{det} \V{A} = \lambda_1 \lambda_2 \)
  - Sign of the determinant (scalings) determines whether the affinity
    is orientation-preserving (positive sign) or orientation-reversing
    (negative sign)

##### 2.4.4 Class IV: Projective Transformations
- Projective transformation is a general non-singular linear transformation
  of **_homogeneous_** coordinates
- Generalization of affinity, which is the composition of a general non-singular
  linear transformation of **_inhomogeneous_** coordinates and a translation
- Block form representation
  \[
     \V{x}' 
     = \V{H}_P \V{x} 
     = \begin{bmatrix} \V{A} & \V{t} \\ \V{v}^T & v \end{bmatrix} \V{x} 
  \]
  - \( \V{v} = (v_1, v_2)^T \)
- DOF: 8
  - 9 elements with only their ratio significant \( \to \) 8 parameters
  - Projective transformation between 2 planes can be computed from
    4 point correspondences (no 3 collinear points on either plane)
- Invariants
  - Cross ratio (ratio of ratios) of 4 collinear points

##### 2.4.7 The Number of Invariants
- _"Number"_: counting argument for the number of _functionally independent_
  invariants
- \( \text{number of invariants} \ge 
     \text{DOF}_{\text{configuration}} - \text{DOF}_{\text{transformation}} \)
- Summary  
  ![02_tab_01](images/ch04/02_tab_01.jpg)

#### 2.7 Recovery of Affine and Metric Properties from Images

##### 2.7.1 The Line at Infinity
- \( \V{l}_\infty \) remains a fixed line under projective transformation 
  \( \V{H} \) if and only if \( \V{H} \) is an affinity
- Fixed lines are fixed as a set, not fixed pointwise
  - A point on the line is mapped to another point on this line

#### 2.9 Fixed Points and Lines
- Fixed point/line of a transformation: correspond to an **_eigenvector_** 
  - \( \V{H} \V{e} = \lambda \V{e} \), where \( \V{e} \) and \( \lambda \V{e} \)
    represents the same point (eigenvector of \( \V{H} \))
  - \( \V{H}^{-T} \V{e} = \lambda \V{e} \), where \( \V{e} \) and 
    \( \lambda \V{e} \) represents the same line (eigenvector of \( \V{H}^T \))
- If 2 of the eigenvalues are identical, the corresponding fixed line will be
  fixed pointwise

### 3. Projective Geometry and Transformations of 3D

#### 3.1 Points and Projective Transformations
- Point \( \V{X} \) in 3-space is represented in homogeneous coordinates 
  as a 4-vector
  - \( \V{X} = (X_1, X_2, X_3, X_4) \) with \( X_4 \neq 0 \) represents the
    point \( (X, Y, Z) \) of \( \F{R}^3 \) with inhomogeneous coordinates
    - \( X = X_1 / X_4 \), \( Y = X_2 / X_4 \), \( Z = X_3 / X_4 \)
- Projective transformation acting on \( \F{P}^3 \): non-singular 
  \( 4 \times 4 \) matrix \( \V{H} \): \( \V{X}' = \V{H} \V{X} \)
  - DOF: 15 (16 elements and less one for overall scaling)
  
#### 3.2 Representing and Transforming Planes, Lines and Quadrics
- Duality in \( \F{P}^3 \): points & _planes_ are dual
  - Similar to point-line duality in \( \F{P}^2 \)
  - Lines are self-dual in \( \F{P}^3 \)

##### 3.2.1 Planes
- A plane in 3-space may be written as
  \( \pi_1 X + \pi_2 Y + \pi_3 Z + \pi_4 = 0 \)
  - Homogeneous representation: \( \BG{\pi} = (\pi_1, \pi_2, \pi_3, \pi_4) \)
    - DOF: 3
- \( \BG{\pi}^T \V{X} = 0 \) \( \to \) point \( \V{X} \) in on plane 
  \( \BG{\pi} \)
- \( \BG{\pi} = (\pi_1, \pi_2, \pi_3, \pi_4) \):
  - First 3 components: plane normal of Euclidean geometry
  - Plane equation in inhomogeneous notation:
    \( \V{n} \cdot \V{\tilde{X}} + d = 0 \)
    - \( \V{n} = (\pi_1, \pi_2, \pi_3) \)
    - \( \V{\tilde{X}} = (X, Y, Z) \) with \( X_4 = 1 \)
    - \( d = \pi_4 \)
    - \( d / \|\V{n}\| \): distance of the plane from the origin
- 3 points define a plane
  \[ 
     \begin{bmatrix} \V{X}_1^T \\ \V{X}_2^T \\ \V{X}_3^T \end{bmatrix}
     \BG{\pi} = \V{A} \BG{\pi} = \V{0} 
  \]
  - The \( 3 \times 4 \) matrix has a rank of 3 if the 3 points are not 
    collinear (linearly independent)
  - Plane \( \BG{\pi} \) is in the (right) nullspace (1-dimensional)
    - \( \V{A}: \F{R}^4 \to \F{R}^3 \)
      - \( \V{X}_i \in C(\V{A}^T) \)
      - \( \BG{\pi} \in N(\V{A}) \)
      - \( \mathrm{dim} C(\V{A}^T) + \mathrm{dim} N(\V{A}) = 3 + 1 = 4 \)
    - 1-dimensional: the plane is defined uniquely (up to scale)
- 3 planes define a point
  \[
     \begin{bmatrix} \BG{\pi}_1^T \\ \BG{\pi}_2^T \\ \BG{\pi}_3^T \end{bmatrix}
     \V{X} = \V{A} \V{X} = 0
  \]
  - Dual to the case of 3 points define a plane
  - The point: defined in the (right) nullspace (1-dimensional)
- Projective transformation (plane)
  - \( \V{X}' = \V{H} \V{X} \) \( \to \) \( \BG{\pi}' = \V{H}^{-T} \BG{\pi} \)

##### 3.2.2 Lines
- DOF of lines in \( \F{P}^3 \): 4
  - Can be defined by two points on 2 orthogonal planes, 2 DOF for each point
  - Hard to be represented normally because it needs a homogeneous 5-vector
- Line representations
  a. Nullspace and span representation
     - Line: pencil (1-parameter family) of collinear points
       - Defined by any of these 2 points
         - 3 DOF (homogeneous 4-vector) for each point
     - Line represented as the span of row space of \( 2 \times 4 \) matrix 
       \( \V{W} \) composed of 2 transposed 4-vectors (point) 
       \( \V{A}^T \) & \( \V{B}^T \)
       \[
          \V{W} = \begin{bmatrix} \V{A}^T \\ \V{B}^T \end{bmatrix}
       \]
       1) Span of \( \V{W}^T \): pencil of points 
          \( \lambda \V{A} + \mu \V{B} \) on the line
       2) Span of 2-dimensional \( N(\V{W}) \) (nullspace) of \( \V{W} \):
          pencil of planes with the line as axis
          - Suppose \( \V{P} \) & \( \V{Q} \) are a basis of the nullspace
          - \( \V{A}^T \V{P} = \V{B}^T \V{P} = \V{0} \): \( \V{P} \) is a plane
            containing the points \( \V{A} \) & \( \V{B} \)
            - Similarly for \( \V{Q} \): a distinct plane
          - As \( \V{A} \) and \( \V{B} \) lie on both (linearly independent)
            plane \( \V{P} \) & \( \V{Q} \), the line defined by \( \V{W} \)
            is the plane intersection
          - Any plane of the pencil with the line as axis can be represented
            by the span \( \lambda' \V{P} + \mu' \V{Q} \)
     - Dual representation: line represented as the span (of row space) of
       \( 2 \times 4 \) matrix \( \V{W}^* \) composed of plane \( \V{P}^T \)
       & \( \V{Q}^T \):
       \[
          \V{W}^* = \begin{bmatrix} \V{P}^T \\ \V{Q}^T \end{bmatrix}
       \]
       1) Span of \( \V{W}^{*T} \): pencil of planes 
          \( \lambda' \V{P} + \mu' \V{Q} \) with the line as axis
       2) Span of 2-dimensional \( N(\V{W}^*) \): pencil of points on the line
     - Relation of the 2 representations: 
       \( \V{W}^* \V{W}^T = \V{W} \V{W}^{*T} = \V{0}_{2 \times 2} \)
       - \( \V{0}_{2 \times 2} \): \( 2 \times 2 \) null matrix
  b. Pluecker matrices
     - Line represented by a \( 4 \times 4 \) skew-symmetric homogeneous matrix
       - Line \( \V{L} \) joining 2 points \( \V{A} \) & \( \V{B} \)
         - Elements: \( l_{ij} = A_i B_j - B_i A_j \)
         - Vector notation: \( \V{L} = \V{A} \V{B}^T - \V{B} \V{A}^T \)
       - Properties of \( \V{L} \)
         1) Rank of \( \V{L} \): 2
            - \( \V{A}^T \V{P} = \V{B}^T \V{P} \) 
              \( = \V{A}^T \V{Q} = \V{B}^T \V{Q} = \V{0} \)
              - \( \V{P} \) & \( \V{Q} \): distinct planes containing
                \( \V{A} \) & \( \V{B} \)
            - \( \V{L} \V{P} = \V{L} \V{Q} = \V{0} \) \( \to \) 
              \( \V{L} \begin{bmatrix} \V{P} & \V{Q} \end{bmatrix} = \V{0} \)
              \( \to \) \( \V{L} \V{W}^{*T} = \V{0}_{4 \times 2} \)
              - \( \V{P} \in N(\V{L}) \) & \( \V{Q} \in N(\V{L}) \)
              - \( \mathrm{dim} N(\V{L}) = 1 + 1 = 2 \)
         2) The representation has the 4 DOF requirement:
            - 6 independent non-zero elements in the \( 4 \times 4 \) 
              skew-symmetric matrix (upper/lower triangle) with 5 of their 
              ratios significant in homogeneous representation
            - \( \mathrm{det} \V{L} = 0 \) & elements satisfy a (quadric)
              constraint \( \to \) net DOF number: 4
         3) \( \V{L} = \V{A} \V{B}^T - \V{B} \V{A}^T \): generalization to
            4-space of the vector product formula 
            \( \V{l} = \V{x} \times \V{y} \) of \( \F{P}^2 \) for a line 
            \( \V{l} \) defined by 2 points \( \V{x} \) & \( \V{y} \) 
            represented by 3-vectors
         4) \( \V{L} \) is independent of the points used to define it
            - Choose another point \( \V{C} \) on the line containing
              \( \V{A} \) & \( \V{B} \) and \( \V{C} = \V{A} + \mu \V{B} \)
              \begin{align*}
               \V{\hat{L}} &= \V{A} \V{C}^T - \V{C} \V{A}^T \\
                           &= \V{A} (\V{A}^T + \mu \V{B}^T) -
                              (\V{A} + \mu \V{B}) \V{A}^T \\
                           &= \V{A} \V{B}^T - \V{B} \V{A}^T \\
                           &= \V{L}
              \end{align*}
         5) Under point transformation \( \V{X}' = \V{H} \V{X} \), the matrix
            transforms as \( \V{L}' = \V{H} \V{L} \V{H}^T \), 
            a valency-2 tensor (?)
     - Dual Pluecker representation \( \V{L}^* \) as the intersection of 2 
       planes \( \V{P} \) & \( \V{Q} \)
       - \( \V{L}^* = \V{P} \V{Q}^T - \V{Q} \V{P}^T \)
       - Has similar properties of \( \V{L} \)
       - \( \V{X}' = \V{H} \V{X} \) \( \to \) 
         \( \V{L}^{*\prime} = \V{H}^{-T} \V{L}^* \V{H}^{-1} \)
     - Join & incidence properties
       1) Plane defined by the join of point \( \V{X} \) & line \( \V{L} \):
          - \( \BG{\pi} = \V{L}^* \V{X} \)
            - \( \V{L}^* \V{X} = 0 \) if and only if \( \V{X} \) is on 
              \( \V{L} \)
       2) Point defined by the intersection of line \( \V{L} \) & plane 
          \( \BG{\pi} \)
          - \( \V{X} = \V{L} \BG{\pi} \)
            - \( \V{L} \BG{\pi} = \V{0} \) if and only if \( \V{L} \) is on
              \( \BG{\pi} \)
  c. Pluecker line coordinates
     - 6 non-zero elements of the \( 4 \times 4 \) skew-symmetric Pluecker
       matrix \( \V{L} \)
       - \( \mathcal{L} = \{l_{12}, l_{13}, l_{14}, l_{23}, l_{42}, l_{34}\} \)
         - Use \( l_{42} \) instead of \( l_{24} \) to eliminate negatives
           in some related formulae
     - Homogeneous 6-vector in \( \F{P}^5 \)
     - \( \mathrm{det} \V{L} = 0 \) \( \to \) 
       \( l_{12} l_{34} + l_{13} l_{42} + l_{14} l_{23} = 0 \)

#### 3.4 The Hierarchy of Transformations
- Specializations with additional properties of 3-space transformations over
  2-space counterparts (check 2.4.7 for general properties)  
  ![03_tab_02](images/ch04/03_tab_02.jpg)
- DOF (class I \( \to \) IV) in \( \F{P}^3 \)
  - Euclidean: 6 (3 + 3)
    - 3 for rotation
    - 3 for translation
  - Similarity: 7 (1 + 6)
    - 1 for isotropic scaling
    - 6 for Euclidean
  - Affinity: 12 (9 + 3)
    - 9 for transformation \( \V{A} \)
    - 3 for translation \( \V{t} \)
  - Projectivity: 15
    - 16 elements with only their ratio significant \( \to \) 15 parameters

#### 3.5 The Plane at Infinity
- Plane at infinity: \( \BG{\pi}_\infty = (0, 0, 0, 1) \) in affine 3-space
- 2 lines are parallel if and only if their line intersection is on 
  \( \BG{\pi}_\infty \)
- A line is parallel to another line/plane if the point of intersection is
  on \( \BG{\pi}_\infty \)
- \( \BG{\pi}_\infty \) is a fixed plane under projective transformation 
  \( \V{H} \) if and only if \( \V{H} \) is an affinity
  - \( \BG{\pi}_\infty \) is in general only fixed under an affinity as a set;
    it is not fixed pointwise
  - There may be additional planes fixed under a particular affinity, but
    only \( \BG{\pi}_\infty \) is fixed under any affinity

### 4. Estimation - 2D Projective Transformations
- Some concrete estimation problems
  a. 2D homography 
     - Given a set of points \( \V{x}_i \) & corresponding \( \V{x}_i' \)
       in \( \F{P}^2 \), compute projective transformation that maps 
       each \( \V{x}_i \) to \( \V{x}_i' \)
  b. 3D to 2D camera projection
     - Given a set of points \( \V{X}_i \) in 3D space and a set of 
       corresponding points \( \V{x}_i \) in an image, find the 3D to 2D
       projective mapping that maps \( \V{X}_i \) to \( \V{x}_i \) carried
       out by a projective camera
  c. Fundamental matrix computation
     - Given a set of points \( \V{x}_i \) in one image, and corresponding 
       set of points \( \V{x}_i' \) in another image, compute the fundamental
       matrix \( \V{F} \) consistent with these correspondences
     - Fundamental matrix \( \V{F} \): a singular \( 3 \times 3 \) matrix
       satisfying \( \V{x}_i^{T\prime} \V{F} \V{x}_i = 0 \) for all \( i \)
  d. Trifocal tensor computation
     - Given a set of point correspondences 
       \( \V{x}_i \leftrightarrow \V{x}_i' \leftrightarrow \V{x}_i'' \)
       across 3 images, compute the trifocal tensor
     - Trifocal tensor \( \mathcal{T}_i^{jk} \): a tensor relating points
       or lines in 3 views
- Focus on 2D homography problem in this chapter

#### 4.1 The Direct Linear Transformation (DLT) Algorithm
- \( \V{x}_i' = \V{H} \V{x}_i \to \V{x}_i' \times \V{H} \V{x}_i = \V{0} \)
  \[
     \V{x}_i' \times \V{H} \V{x}_i
     = \begin{pmatrix}
        y_i' \V{h}^{3T} \V{x}_i - w_i' \V{h}^{2T} \V{x}_i \\
        w_i' \V{h}^{1T} \V{x}_i - x_i' \V{h}^{3T} \V{x}_i \\
        x_i' \V{h}^{2T} \V{x}_i - y_i' \V{h}^{1T} \V{x}_i 
       \end{pmatrix}
     = \begin{pmatrix}
        \V{0}^T \V{h}^1 - w_i' \V{x}_i^T \V{h}^2 + y_i' \V{x}_i^T \V{h}^3 \\
        w_i' \V{x}_i^T \V{h}^1 + \V{0}^T \V{h}^2 - x_i' \V{x}_i^T \V{h}^3 \\
        -y_i' \V{x}_i^T \V{h}^1 + x_i' \V{x}_i^T \V{h}^2 + \V{0}^T \V{h}^3
       \end{pmatrix}
  \]
  \[
     \V{A}_i \V{h}
     = \begin{bmatrix}
        \V{0}^T         & -w_i' \V{x}_i^T & y_i' \V{x}_i^T \\
        w_i' \V{x}_i^T  & \V{0}^T         & -x_i' \V{x}_i^T \\
        -y_i' \V{x}_i^T & x_i' \V{x}_i^T  & \V{0}^T
       \end{bmatrix}
       \begin{pmatrix} \V{h}^1 \\ \V{h}^2 \\ \V{h}^3 \end{pmatrix}
     = \V{0} \tag{4-1} \label{eq4-1}
  \]
  - \( \V{x}_i' = (x_i', y_i', w_i') \)
- Notations for \eqref{eq4-1}
  \[
     \V{H} 
     = \begin{bmatrix} 
        h_1 & h_2 & h_3 \\ h_4 & h_5 & h_6 \\ h_7 & h_8 & h_9
       \end{bmatrix}
     = \begin{pmatrix} \V{h}^{1T} \\ \V{h}^{2T} \\ \V{h}^{3T} \end{pmatrix}, 
     \quad \V{h} = \begin{pmatrix} \V{h}^1 \\ \V{h}^2 \\ \V{h}^3 \end{pmatrix}
     \tag{4-2} \label{eq4-2}
  \]
  - \( \V{h} \): a 9-vector
  - \( \V{h}^{jT} \): a vector denoting the \( j \)-th row of \( \V{H} \)
- Basic DLT for 2D homography problem (without normalization)  
  ![04_alg_01](images/ch04/04_alg_01.jpg)
- Over-determined solution
  - No exact solution \( \to \) find an approximate solution:
    - A vector \( \V{h} \) that minimizes a cost function
      - Minimize \( \| \V{A} \V{h} \| \) with constraint \( \| \V{h} \| = 1 \)


## Part 1 - Camera Geometry and Single View Geometry

### 6. Camera Models
- Camera: a mapping between the 3D world (object space) and a 2D image

#### 6.1 Finite Cameras
- The basic pinhole model:  
  ![06_fig_01](images/ch04/06_fig_01.jpg)
  - \( \V{X} \): 3D point \( \V{X} = (X, Y, Z) \)
  - \( \V{x} \): 2D point on image/focal plane \( Z = f \)
  - \( \V{p} \): _principle point_ (where the principle axis meets the image 
    plane)
  - \( (X, Y, Z) \) mapped to \( (fX/Z, fY/Z, f) \) on image plane
    - An Euclidean 3-space \( \F{R}^3 \) to Euclidean 2-space \( \F{R}^2 \) 
      mapping
      \[ 
         (X, Y, Z) \mapsto (fX/Z, fY/Z) \tag{6-1} \label{eq6-1}
      \]
- Central projection using homogeneous coordinates
  - Written \eqref{eq6-1} as
    \[
       \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}
       \mapsto \begin{pmatrix} fX \\ fY \\ Z \end{pmatrix} 
       = \begin{bmatrix} f & & & 0 \\ & f & & 0 \\ & & 1 & 0 \end{bmatrix}
         \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}
       \tag{6-2} \label{eq6-2}
    \]
  - \eqref{eq6-2} can be written compactly as
    \[
       \V{x} = \V{P} \V{X}
    \]
    - Camera matrix 
      \( 
         \V{P} 
         = \mathrm{diag}(f, f, 1) 
           \left[ \begin{array}{c|c} \V{I} & \V{0} \end{array} \right]
      \)
- Principle point offset
  - For principle point at \( (p_x, p_y) \) in the image plane, the 3D \( \to \)
    2D mapping becomes \( (X, Y, Z) \mapsto (fX/Z + p_x, fY/Z + p_y) \)
    \[
       \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}
       \mapsto \begin{pmatrix} fX + Z p_x \\ fY + Z p_y \\ Z \end{pmatrix} =
       \begin{bmatrix} f & & p_x & 0 \\ & f & p_y & 0 \\ & & 1 & 0 \end{bmatrix}
       \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}
       \tag{6-3} \label{eq6-3}
    \]
    where 
    \[
       \V{K} = \begin{bmatrix} f & & p_x \\ & f & p_y \\ & & 1 \end{bmatrix}
    \]
    is called _camera calibration matrix_
  - \eqref{eq6-3} has the concise form
    \[
       \V{x} = \V{K} \left[ \begin{array}{c|c} \V{I} & \V{0} \end{array} \right]
               \V{X}_{\text{cam}}
       \tag{6-5} \label{eq6-5}
    \]
    - \( \V{X}_{\text{cam}} = (X, Y, Z, 1) \) is in _camera coordinate frame_
- Camera rotation and translation
  - Let \( \V{\tilde{X}} \) & \( \V{\tilde{X}}_{\text{cam}} \) be inhomogeneous
    3-vectors representing coordinates of a point in _world coordinate frame_
    and _camera coordinate frame_, respectively
    - \( \V{\tilde{X}}_{\text{cam}} = \V{R} (\V{\tilde{X}} - \V{\tilde{C}}) \)
      - \( \V{\tilde{C}} \) represents the coordinate of the camera center 
        in the world coordinate frame
    - \( \V{R} \): \( 3 \times 3 \) rotation matrix representing the orientation
      of the camera coordinate frame
  - Homogeneous version:
    \[
       \V{X}_{\text{cam}}
       = \begin{bmatrix} \V{R} & -\V{R} \V{\tilde{C}} \\ 0 & 1 \end{bmatrix}
         \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}
       = \begin{bmatrix} \V{R} & -\V{R} \V{\tilde{C}} \\ 0 & 1 \end{bmatrix} 
         \V{X}
       \tag{6-6} \label{eq6-6}
    \]
  - Putting \eqref{eq6-5} and \eqref{eq6-6} together:
    \[
       \V{x} = \V{K} \V{R} 
               \left[ \begin{array}{c|c} \V{I} & -\V{\tilde{C}} \end{array} 
               \right] \V{X}
    \]
    - \( \V{X} \): coordinate in the world coordinate frame
    - \( \V{P} = \V{K} \V{R} 
                 \left[ \begin{array}{c|c} \V{I} & -\V{\tilde{C}} \end{array} 
                 \right] 
      \): general pinhole camera
  - DOF for \( \V{P} \): 9 
    - 3 for \( \V{K} \) (\( f \), \( p_x \), \( p_y \)), called _internal_
      camera parameters, or _internal orientation_ of the camera
    - 3 for \( \V{R} \), and 3 for \( \V{\tilde{C}} \), which are called
      _external_ parameters, or the _exterior orientation_
  - The world to image transformation can be represented as 
    \( \V{\tilde{X}}_{\text{cam}} = \V{R} \V{\tilde{X}} + \V{t} \), and then
    the camera matrix can be 
    \[
       \V{P} = \V{K} \left[ \begin{array}{c|c} \V{R} & \V{t} \end{array} \right]
       \tag{6-8} \label{eq6-8}
    \]
    where \( \V{t} = -\V{R} \V{\tilde{C}} \)
- CCD Cameras
  - Possibly having non-square pixels
  - General form of calibration matrix
    \[
       \V{K} = \mathrm{diag}(m_x, m_y, 1)
               \begin{bmatrix} f & & p_x \\ & f & p_y \\ & & 1 \end{bmatrix}
             = \begin{bmatrix} 
                \alpha_x & & x_0 \\ & \alpha_y & y_0 \\ & & 1 
               \end{bmatrix}
    \]
    where
    - \( m_x \) & \( m_y \): number of pixels in per unit distance in 
      image coordinates in the \( x \) and \( y \) directions
    - \( \alpha_x = f m_x \) & \( \alpha_y = f m_y \) represent camera 
      focal length in terms of pixel dimensions in \( x \) and \( y \) 
      direction respectively
    - \( \V{\tilde{x}}_0 = (x_0, y_0) = (m_x p_x, m_y p_y) \): principal point
    - DOF: 10
- Finite projective camera
  - Add a _skew_ parameter \( s \) to the calibration matrix
    \[
       \V{K} = \begin{bmatrix}
                \alpha_x & s & x_0 \\ & \alpha_y & y_0 \\ & & 1
               \end{bmatrix}
       \tag{6-10} \label{eq6-10}
    \]
  - Finite projective camera: a camera \eqref{eq6-8} with calibration matrix 
    \( \V{K} \) of the form \eqref{eq6-10}
  - DOF: 11, same with a \( 3 \times 4 \) matrix defined up to arbitrary scale
    - The set of camera matrices of _finite projective cameras_ is **identical**
      with the set of _homogeneous \( 3 \times 4 \) matrices_ for which the 
      left hand \( 3 \times 3 \) submatrix is **non-singular**
  - Transformation from world coordinate \( \V{X} = (X, Y, Z, 1) \) 
    to pixel coordinate \( w \V{x} = (wx, wy, w) \) ([ref](http://www.cim.mcgill.ca/~langer/558/4-cameramodel.pdf)):
    \[
       w \V{x} 
       = \V{K} \V{R} 
         \left[ \begin{array}{c|c} \V{I} & -\V{\tilde{C}} \end{array} \right] 
         \V{X}
    \]
- General projective camera
  - Arbitrary homogeneous \( 3 \times 4 \) matrices with rank 3
  - DOF: 11
  - Left-hand \( 3 \times 3 \) submatrix needs not be non-singular


## Part 4 - N-View Geometry

### 18. N-View Computational Methods
- Topic: methods for estimating a projective/affine reconstruction 
  from a set of images
  - Number of views is large
- Contents
  - Bundle adjustment (BA) for projective reconstruction (general case)
  - Specialization of BA to affine cameras \( \to \) factorization algorithm
  - Generalization of the factorization algorithm to non-rigid scenes
  - Specialization of BA for scenes containing planes
  - Methods for obtaining point correspondences throughout image sequence
    and projective reconstruction from these correspondences

#### 18.1 Projective Reconstruction - Bundle Adjustment
- Formulation
  - A set of 3D points \( \V{X}_j \) is viewed by a set of cameras with
    matrices \( \V{P}^i \)
  - \( \V{x}_j^i \): coordinate of \( j \)-th point as seen by \( i \)-th camera
  - Reconstruction problem: given the set of image coordinates \( \V{x}_j^i \),
    find the set of camera matrices \( \V{P}^i \) and the points \( \V{X}_j \),
    such that \( \V{P}^i \V{X}_j = \V{x}_j^i \)
    - Projective reconstruction if no constraints given 
      on \( \V{P}^i \) or \( \V{X}_j \)
      - \( \V{X}_j \) may differ by an arbitrary 3D projective transformation
        from true reconstruction
- Bundle adjustment
  - \( \V{x}_j^i = \V{P}^i \V{X}_j \) will not be satisfied exactly if image
    measurements are noisy
  - Maximum Likelihood (ML) solution: assume measurement noise is Gaussian
    - Estimate \( \V{\hat{P}}^i \) and \( \V{\hat{X}}_j \) such that
      \( \V{\hat{x}}_j^i = \V{\hat{P}}^i \V{\hat{X}}_j \)
    - Minimize image distance between reprojected point \( \V{\hat{x}}_j^i \)
      and detected/measured point \( \V{x}_j^i \) for every view where 3D
      point appears
      \[
         \min_{\V{\hat{P}}^i, \V{\hat{X}}_j} 
         \Sigma_{i, j} d(\V{\hat{P}}^i \V{\hat{X}}_j, \V{x}_j^i)^2
         \tag{18-1} \label{eq18-1}
      \]
      - \( d(\V{x}, \V{y}) \): geometric image distance between homogeneous
        point \( \V{x} \) and \( \V{y} \)
    - Bundle adjustment: the above estimation involving minimizing 
      the reprojection error
      - Adjusting the bundle of rays between each camera center and the
        set of 3D points
  - Should generally be used as a final step of any reconstruction algorithm
  - Pros & cons
    - \+ Tolerance of missing data while providing true ML estimate
    - \+ Allow individual covariances (or more general PDFs) assignment to each
      measurement
    - \+ May be extended to include the following estimates: priors, 
      constraints on camera parameters, or point positions
    - \- Require good initialization
    - \- Can become extremely large optimization problem as the number of 
      parameters involved increases

#### 18.2 Affine Reconstruction - the Factorization Algorithm
- Algorithm  
  ![18_alg_01](images/ch04/18_alg_01.jpg)
  - \( \V{\hat{W}} \): rank 3 matrix closest to \( \V{W} \) in Frobenius norm
    - \( \V{\hat{W}}
         = \V{U}_{2m \times 3} \V{D}_{3 \times 3} \V{V}_{3 \times n}^T 
         = \V{\hat{M}} \V{\hat{X}} \)
  - Choice of \( \V{\hat{M}} \) & \( \V{\hat{X}} \) is not unique
    - E.g., 
      \( \V{\hat{M}} = \V{U}_{2m \times 3} \V{D}_{3 \times 3} \) &
      \( \V{\hat{X}} = \V{V}_{3 \times n}^T \);
      or
      \( \V{\hat{M}} = \V{U}_{2m \times 3} \) &
      \( \V{\hat{X}} = \V{D}_{3 \times 3} \V{V}_{3 \times n}^T \),
      etc.
- Affine ambiguity: 
  \( \V{\hat{W}} = \V{\hat{M}} \V{\hat{X}} 
                 = \V{\hat{M}} \V{A} \V{A}^{-1} \V{\hat{X}} 
                 = (\V{\hat{M}} \V{A}) (\V{A}^{-1} \V{\hat{X}}) \)
  - Affinity in \( \F{P}^3 \): 
    \( \begin{bmatrix} 
        \V{A}_{3 \times 3} & \V{t}_{3 \times 1} \\
        \V{0}_{1 \times 3} & 1_{1 \times 1}
       \end{bmatrix}_{4 \times 4}
    \)
  - Camera matrices \( \V{M}^i \) & 3D points \( \V{X}_j \) obtained from
    \( \V{\hat{M}} \) & \( \V{\hat{X}} \) (estimates) are determined up to 
    multiplication by a common matrix \( \V{A} \)
    - The MLE reconstruction is affine


## Part 5 - Appendices

### Appendix 4: Matrix Properties and Decompositions

#### A4.2 Symmetric and Skew-symmetric Matrices
- Cross products
  - \( \V{a} = (a_1, a_2, a_3) \) \( \to \) \( [\V{a}]_{\times} \)
    \[ 
       [\V{a}]_{\times} 
       = \begin{bmatrix} 
             0 & -a_3 &  a_2 \\
           a_3 &    0 & -a_1 \\
          -a_2 &  a_1 &    0
         \end{bmatrix}
    \]
  - \( \V{a} \times \V{b} = [\V{a}]_{\times} \V{b} 
                          = (\V{a}^T [\V{b}]_{\times})^T \)

### Appendix 6: Iterative Estimation Methods

#### A6.1 Newton Iteration
- Formulation
  - Given a hypothesized function relation \( \V{X} = \V{f}( \V{P} ) \)
    - \( \V{X} \): a _measurement vector_ in Euclidean space \( \F{R}^N \)
    - \( \V{P} \): a _parameter vector_ in Euclidean space \( \F{R}^M \)
  - Measured value \( \V{X} \) approximating true value \( \V{\bar{X}} \)
    is provided
  - Find the vector \( \V{\hat{P}} \) satisfying 
    \( \V{X} = \V{f}( \V{\hat{P}} ) - \BG{\epsilon} \)
    for which \( \| \BG{\epsilon} \| \) is minimized
    - Linear function \( \V{f}( \cdot ) \): least-squares problem where
      \( \V{f}( \V{P} ) = \V{A} \V{P} \)
- Solution
  - Start with an initial estimated value \( \V{P}_0 \)
    - Proceed to refine the estimate under the assumption that _function
      \( \V{f} \) is locally linear_
  - Let \( \BG{\epsilon} = \V{f}( \V{P}_0 ) - \V{X} \)
  - Assume \( f \) is approximated at \( \V{P}_0 \) by
    \( \V{f}( \V{P}_0 + \BG{\Delta} ) = \V{f}( \V{P}_0 ) + \V{J} \BG{\Delta} \)
    - \( \V{J} = \partial \V{f} / \partial \V{P} \) (Jacobian matrix)
  - Seek a point \( \V{P}_1 = \V{P}_0 + \BG{\Delta} \) which minimizes
    \( \V{f}( \V{P}_1 ) - \V{X} = \V{f}( \V{P}_0 ) + \V{J} \BG{\Delta} - \V{X}
                                = \BG{\epsilon}_0 + \V{J} \BG{\Delta} \)
    - Minimizing \( \| \BG{\epsilon}_0 + \V{J} \BG{\Delta} \| \) over
      \( \BG{\Delta} \) \( \to \) linear minimization problem
  - \( \BG{\Delta} \) is obtained by
    a. Solving _normal_ equations 
       \[ 
          \V{J}^T \V{J} \BG{\Delta} = - \V{J}^T \BG{\epsilon}_0 
          \tag{A6.1} \label{eqA6.1}
       \]
    b. Using the pseudo-inverse \( \BG{\Delta} = - \V{J}^{+} \BG{\epsilon}_0 \)
  - Solution vector \( \V{\hat{P}} \) is obtained by
    - Starting with an initial estimate \( \V{P}_0 \) and computing
      successive approximations by the formula
      \( \V{P}_{i + 1} = \V{P}_i + \V{J} \BG{\Delta}_i \)
      - \( \BG{\Delta}_i \): solution to the least-squares problem
        \( \V{J} \BG{\Delta}_i = - \BG{\epsilon}_i \)
        - \( \V{J} = \frac {\partial \V{f}} {\partial \V{P}} |_{ \V{P}_i } \)
        - \( \BG{\epsilon}_i = \V{f}( \V{P}_i ) - \V{X} \)
  - Disadvantage
    - The iteration procedure may converge to a local minimum or not converge
      at all
    - The algorithm depends very strongly on the initial estimate \( \V{P}_0 \)
- Weighted iteration
  - Measurement \( \V{X} \) satisfies a Gaussian distribution with covariance
    matrix \( \BG{\Sigma}_{\V{X}} \)
    - \( \BG{\Sigma}_{\V{X}} \): generally be an arbitrary symmetric & positive
      definite matrix
  - Minimize Mahalanobis distance 
    \( \| \V{f}( \V{P} ) - \V{X} \|_{\BG{\Sigma}_{\V{X}}} \) 
    instead of \( \| \V{f}( \V{P} ) - \V{X} \| \) 
  - Normal equation becomes 
    \( \V{J}^T \BG{\Sigma}_{\V{X}}^{-1} \V{J} \BG{\Delta}_i 
       = - \V{J}^T \BG{\Sigma}_{\V{X}}^{-1} \BG{\epsilon}_i \)
- Newton's method & the Hessian
  - Finding minima of function \( g( \V{P} ) \) of many variables
    - Expand \( g( \V{P} ) \) about \( \V{P}_0 \) in a Taylor series
      - \( g( \V{P}_0 + \BG{\Delta} ) 
           = g + g_{\V{P}} \BG{\Delta} + 
             \BG{\Delta}^T g_{\V{P} \V{P}} \BG{\Delta} / 2 + ... \)
    - Set derivative w.r.t. \( \BG{\Delta} \) to 0
      \[ 
         g_{\V{P} \V{P}} \BG{\Delta} = - g_{\V{P}} \tag{A6.2} \label{eqA6.2}
      \]
      - \( g_{\V{P} \V{P}} \): the **_Hessian_** where the \( (i, j) \)-th 
        entry of which is \( \partial^2 g / \partial p_i \partial p_j \)
  - **_Newton iteration_**: starting at an initial value \( \V{P}_0 \), and 
    iteratively computing parameter increment \( \BG{\Delta} \) using 
    \eqref{eqA6.2} until convergence occurs
  - Cost/error function
    \[
       g( \V{P} ) = \| \BG{\epsilon}( \V{P} ) \|^2 / 2
                  = \BG{\epsilon}( \V{P} )^T \BG{\epsilon}( \V{P} ) / 2
    \]
    - Gradient vector: \( g_{\V{P}} = \BG{\epsilon}_{\V{P}}^T \BG{\epsilon} \)
      - \( \BG{\epsilon}_{\V{P}} \): 
        \( \frac {\partial \BG{\epsilon}(\V{P})} {\partial \V{P}} \)
      - \( \BG{\epsilon} \): \( \BG{\epsilon}(\V{P}) \)
    - The Hessian
      \[
         g_{\V{P} \V{P}} = \BG{\epsilon}_{\V{P}}^T \BG{\epsilon}_{\V{P}}
                           + \BG{\epsilon}_{\V{P} \V{P}}^T \BG{\epsilon}
         \tag{A6.3} \label{eqA6.3}
      \]
  - Assume \( \V{f}( \V{P} ) \) is linear, 2nd term of \eqref{eqA6.3}
    becomes 0
    - \( g_{\V{P} \V{P}} 
         = \BG{\epsilon}_{\V{P}}^T \BG{\epsilon}_{\V{P}}
         = \V{J}^T \V{J} \)
    - \eqref{eqA6.2} becomes: 
      \( \V{J}^T \V{J} \BG{\Delta} = - \V{J}^T \BG{\epsilon} \)
      - Same as \eqref{eqA6.1}
  - **_Gaussian-Newton method_**: the iterative procedure where 
    \( \V{J}^T \V{J} \) is used as an approximation of the Hessian
    ( \( g_{\V{P} \V{P}} \) )
- Gradient descent
  - \( -g_{\V{P}} = -\BG{\epsilon}_{\V{P}}^T \BG{\epsilon} \): 
    negative/down-hill gradient vector defined the direction of most rapid 
    decrease of cost function
  - Parameter increment \( \BG{\Delta} \): computed from the equation
    \( \lambda \BG{\Delta} = -g_{\V{P}} \)
    - \( \lambda \): controls the length of the step
  - The Hessian is approximated (somewhat arbitrarily) by matrix 
    \( \lambda \V{I} \):   
    \( g_{\V{P} \V{P}} \BG{\Delta} = - g_{\V{P}} 
       \quad \to \quad \lambda \V{I} \BG{\Delta} = - g_{\V{P}} 
       \quad \to \quad \lambda \BG{\Delta} = - g_{\V{P}} \)
  - Not a good minimization strategy due to slow convergence by zig-zagging
- Summary on minimization of cost function 
  \( g(\V{P}) = \| \BG{\epsilon}(\V{P}) \|^2 / 2 \)
  a. **_Newton_** \( \to \) update equation:
     \[
        g_{\V{P}\V{P}} \BG{\Delta} = -g_{\V{P}}
     \]
     - \( g_{\V{P}\V{P}} = \BG{\epsilon}_{\V{P}}^T \BG{\epsilon}_{\V{P}}
                            + \BG{\epsilon}_{\V{P} \V{P}}^T \BG{\epsilon} \)
     - \( g_{\V{P}} = \BG{\epsilon}_{\V{P}}^T \BG{\epsilon} \)
  b. **_Gauss-Newton_** \( \to \) update equation:
     \[
        \BG{\epsilon}_{\V{P}}^T \BG{\epsilon}_{\V{P}} \BG{\Delta}
        = - \BG{\epsilon}_{\V{P}}^T \BG{\epsilon}
     \]
  c. **_Gradient Descent_** \( \to \) update equation:
     \[
        \lambda \BG{\Delta} 
        = -\BG{\epsilon}_{\V{P}}^T \BG{\epsilon} = -g_{\V{P}}
     \]

#### A6.2 Levenberg-Marquardt iteration
- Normal equations \( \V{J}^T \V{J} \BG{\Delta} = -\V{J}^T \BG{\epsilon} \)
  are replaced by **_augmented normal equations_** 
  \( (\V{J}^T \V{J} + \lambda \V{I}) \BG{\Delta} = -\V{J}^T \BG{\epsilon} \)
  - Typical value for \( \lambda \) is \( 10^{-3} \)
- Levenberg-Marquardt (LM) iteration
  - Repeatedly solving augmented normal equations for different \( \lambda \)
    until an acceptable \( \BG{\Delta} \) is found
    - If \( \BG{\Delta} \) solved leads to reduction in the error
      - Accept the increment
    - If \( \BG{\Delta} \) solved leads to increased error
      - Multiply \( \lambda \) by same factor and solve new augmented normal
        equation

\newpage
