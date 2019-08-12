# Notes on SLAM Survey [@Cadena2016] ([site](https://arxiv.org/abs/1606.05830)|[paper](https://arxiv.org/pdf/1606.05830.pdf))

## 1. Introduction
- SLAM development:
  - Classical age (1986 - 2004): covered in SLAM tutorial [I](#slam1) 
    [@Durrant-Whyte2006] & [II](#slam2) [@Bailey2006]
  - Algorithmic-analysis age (2004 - 2015): 
    covered in [@Dissanayake2011 [(link)](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6038117)]
  - Robust-perception age (2015 - )
    - Robust performance
    - High-level understanding
    - Resource awareness
    - Task-driven inference
- 2 questions:
  a. Do autonomous robots really need SLAM?
     - Visual-inertial Navigation (VIN) _is_ SLAM 
       - A _reduced_ SLAM system without loop closure / place recognition module
     - Advantage of SLAM: getting real topology of the environment by 
       enabling loop closure
       - Offers a natural defense against wrong data association & perceptual 
         aliasing, which will deceive place recognition
       - Key to robust operation
     - Many applications implicitly/explicitly require a globally consistent map
  b. Is SLAM solved?
     - Need to specify the robot/environment/performance combination
       - Robot
         - Type of motion
         - Available sensors
         - Available computational resources
       - Environment
         - Dimension: planar or three-dimensional
         - Presence of natural/artificial landmarks
         - Amount of dynamic elements
         - Amount of symmetry
         - Risk of perceptual aliasing
         - May depend on the sensor-environment pair
       - Performance requirements
         - Desired accuracy of the robot state estimation
         - Accuracy & type of representation of the environment
           (landmark-based or dense)
         - Success rate
         - Estimation latency
         - Maximum operation time
         - Maximum size of mapped area


## 2. Anatomy of a Modern SLAM System
- 2 parts of SLAM system  
  ![SLAM System](images/ch03/01_slam_system.jpg){ width=60% }
  - Front-end: abstract sensor data into estimation models
    - Data association module
      - Short-term data block: associate corresponding features in consecutive
        sensor measurements
      - Long-term data block (loop closure): associate new measurements to 
        older landmarks
    - Pre-processing: sensor dependent
  - Back-end: perform inference on abstracted data from front-end
    - Feed feedback info to the front-end to support loop closure detection
      & validation

### 2.1 SLAM Back-end

#### 2.1.1 Fisher Information Matrix
- Abbreviated as information matrix
- Fisher information \( \mathcal{I}(\theta) \) over parameter \( \theta \):
  \begin{align*}
      \mu
      &= \mathrm{E} [\frac {\partial} {\partial \theta}
                     \mathrm{log} f(X; \theta) | \theta] \\
      &= \int \frac {\partial} {\partial \theta} 
         (\mathrm{log} f(X; \theta)) f(X; \theta) \mathrm{d}X \\
      &= \int \frac {\frac {\partial} {\partial \theta} f(X; \theta)}
         {f(X; \theta)} f(X; \theta) \mathrm{d}X \\
      &= \frac {\partial} {\partial \theta} \int f(X; \theta) \mathrm{d}X \\
      &= \frac {\partial} {\partial \theta} 1 \\
      &= 0 \\
      \mathcal{I}(\theta) 
      &= \mathrm{E} [(\frac {\partial} {\partial \theta} 
                      \mathrm{log} f(X; \theta) - \mu)^2 | \theta] \\
      &= \mathrm{E} [(\frac {\partial} {\partial \theta} 
                      \mathrm{log} f(X; \theta))^2 | \theta]
  \end{align*}
  - \( \mu \): 1st moment / expected value of the "score" (i.e., 
    the partial derivative with respect to \( \theta \) of the 
    log-likelihood function \( f(X; \theta) \))
  - Fisher information: 2nd central moment / variance of 
    log-likelihood function \( f(X; \theta) \)
- If \( \mathrm{log} f(X; \theta) \) is twice differentiable w.r.t. 
  \( \theta \), then under certain regularity conditions
  \[
      \mathcal{I}(\theta)
      = \mathrm{E} [(\frac {\partial} {\partial \theta} 
                     \mathrm{log} f(X; \theta))^2 | \theta]
      = - \mathrm{E} [(\frac {\partial^2} {\partial \theta^2} 
                       \mathrm{log} f(X; \theta) | \theta]
  \]
- Fisher information matrix: Fisher information of a \( N \times N \)
  vector of parameter 
  \( \theta = [\theta_1, \theta_2, \dots, \theta_N]^T \)
  - Element \((i, j)\) of matrix \( \V{\mathcal{I}} \)
    \[
        [\mathcal{I}(\theta)]_{i, j} = \mathrm{E}
        [(\frac {\partial} {\partial \theta_i} \mathrm{log} f(X; \theta))
         (\frac {\partial} {\partial \theta_j} \mathrm{log} f(X; \theta))
         | \theta]
    \]
  - \( \mathrm{dim}\ \V{\mathcal{I}} = N \times N \)
  - [Example](https://math.stackexchange.com/questions/328835/get-a-fisher-information-matrix-for-linear-model-with-the-normal-distribution-fo)

#### 2.1.2 Maximum-a-posteriori Estimation
- Maximum-a-posteriori (MAP) estimation as SLAM formulation
  - Formulate SLAM as MAP estimation problem
  - Use _factor graphs_ to reason about the interdependence among variables
    - Example: \(
      g(X_1, X_2, X_3) = f_1(X_1) f_2(X_1, X_2) f_3(X_1, X_2) f_4(X_2, X_3) \)  
      ![Factor Graph](images/ch03/02_factor_graph_example.jpg){ width=35% }
- Math notations:
  - \( X \): robot trajectory (discrete set of poses) & 
    landmark positions in the environment
  - \( Z = \{z_k : k = 1, \dots, m\} \): set of measurements
  - \( z_k = h_k(X_k) + \epsilon_k \), where \( X_k \subseteq X \), 
    \( h_k(\cdot) \) being the observation/measurement model (known 
    _nonlinear_ function)
- MAP estimation: estimate \( X \) by computing variables \( X^* \) that
  attains the maximum of the posterior \( P(X|Z) \) (the _belief_ over \( X \)
  given the measurements)
  \[
      X^* \triangleq \argmax_X P(X|Z) 
          = \argmax_X \frac {P(Z|X) P(X)} {\int_X P(Z|X) P(X) dX}
          = \argmax_X P(Z|X) P(X)
  \]
  - The denominator of the posterior distribution (marginal likelihood)
    plays no role in the optimization because it is always positive & 
    does not depend on \( X \)
- Assume the measurements are independent (i.e., the noise terms are not
  correlated with the measurements)
  \[
      X^* = \argmax_X P(X) \prod_{k=1}^m P(z_k|X)
          = \argmax_X P(X) \prod_{k=1}^m P(z_k|X_k) \tag{3-1} \label{eq3-1}
  \]
- Assume \( \epsilon_k \) is a zero-mean Gaussian noise with information matrix
  \( \Omega_k \), and the prior is normally distributed
  \[
      P(z_k|X_k) 
      \propto \mathrm{exp}(-\frac{1}{2} \| h_k(X_k) - z_k \|_{\Omega_k}^2) 
      \tag{3-2} \label{eq3-2}
  \]
  \[
      P(X)
      \propto \mathrm{exp}(-\frac{1}{2} \| h_0(X) - z_0 \|_{\Omega_k}^2)
      \tag{3-3} \label{eq3-3}
  \]
  - \( P(z_k|X_k) \): measurement likelihood
  - \( P(X) \): the prior
  - \( \| e \|_{\Omega}^2 = e^T \Omega e \)
- According to \eqref{eq3-2} and \eqref{eq3-3}, \eqref{eq3-1}
  becomes the cost function below (nonlinear (generalized) least squares 
  problem):
  \[
      \begin{aligned}
       X^* &= \argmin_X (-\mathrm{log}(P(X) \prod_{k=1}^m P(z_k|X_k))) \\
           &= \argmin_X \sum_{k=0}^m \frac{1}{2} \| h(X_k) - z_k \|_{\Omega_k}^2
      \end{aligned} \tag{3-4} \label{eq3-4}
  \]
  - Derived from the assumption of normally distributed noise
  - Different assumptions for the noise distribution lead to different cost 
    functions
- Similarity & differences between \eqref{eq3-4} & bundle adjustment (BA):
  - Similarity: both stem from MAP formulation
  - Differences:
    - Factors in \eqref{eq3-4} are not constrained to model projective 
      geometry in BA but include a broad variety of sensor models
    - \eqref{eq3-4} is designed to be solved **_incrementally_** because
      new measurements are made available at each time step as the robot moves
- Minimization problem of \eqref{eq3-4} is solved via successive 
  **_linearizations_** using Gauss-Newton (GN) or Levenberg-Marquardt or other
  modern methods
  - GN approximate \eqref{eq3-4} as
    \[
        \delta_X^* = \argmin_{\delta_X} \frac{1}{2} \sum_{k=0}^m 
                     \| A_k \delta_X - b_k \|_{\Omega_k}^2
                   = \argmin_{\delta_X} \frac{1}{2} 
                     \| A \delta_X - b \|_{\Omega_k}^2
        \tag{3-5} \label{eq3-5}
    \]
    - \( \delta_X \): a small "correction" w.r.t. the linearization point 
      \( \hat{X} \)
    - \( A_k = \frac {\partial h_k(X)} {\partial X} \): the Jacobian of the
      measurement function \( h_k(\cdot) \) w.r.t. \( X \)
    - \( b_k \triangleq z_k - h_k(\hat{X}) \): the _residual error_ at 
      \( \hat{X} \)
    - \( A \) & \( b \) are obtained by stacking \( A_k \) & \( b_k \)
      - \( \mathrm{dim}\ X = \mathrm{dim}\ \delta_X \triangleq n \times 1 \)
      - \( \mathrm{dim}\ A_k = n \times n \)
      - \( \mathrm{dim}\ b_k = \mathrm{dim}\ \delta_X = n \times 1 \)
      - Stacking: 
        - \( \mathrm{dim}\ A = ((m+1) \times n) \times n \)
        - \( \mathrm{dim}\ b = ((m+1) \times n) \times 1 \)
    - \( \Omega_k \): measurement information matrix 
      - \( \mathrm{dim}\ \Omega_k = n \times n \)
    - \( \Omega \): block diagonal matrix obtained by stacking \( \Omega_k \)
      as its diagonal blocks
      - \( \mathrm{dim}\ \Omega = ((m+1) \times n) \times ((m+1) \times n) \)
  - Optimal correction \( \delta_X^* \) to minimize \eqref{eq3-5}
    \[
        \delta_X^* = (A^T \Omega A)^{-1} A^T \Omega b 
        \tag{3-6} \label{eq3-6}
    \]
    - \( A^T \Omega A \): the _Hessian_
    - Update of linearization point: at each iteration, 
      - \( \hat{X} \gets \hat{X} + \delta_X^* \) if \( X \) belongs to a vector
        space
      - \( \hat{X} \gets \hat{X} \oplus \delta_X^* \) if \( X \) includes
        variables belonging to a smooth manifold
        - \( \delta_X^* \) belongs to the tangent space of \( X \)
        - Operator \( \oplus \): retraction operator that maps the vector
          \( \delta_X^* \) in the tangent space of manifold at \( \hat{X} \)
          to an element of the manifold
          - The mapping from one tangent space to another one
          - Example: move the point on a sphere by a small amount, and the 
            result point will have a different tangent space (2D plane)
            with the original point
          - Tangent space illustration where the sphere can be seen as a 
            2D manifold  
            ![Tangent Space](images/ch03/03_tangent_space.jpg){ width=25% }

#### 2.1.3 Summary
- Key insights behind SLAM solvers
  - Jacobian matrix \( A \) in \eqref{eq3-6} is **_sparse_** and its 
    sparsity structure is dictated by **_the topology of underlying factor 
    graph_**
    - Enable the use of fast linear solvers to compute \( \delta_X^* \)
    - Allow the designing of _incremental_/_online_ solvers (update
      the estimate of \( X \) as new observations are acquired)
- Current SLAM libraries:
  - GTSAM, g2o, Ceres, iSAM, and SLAM++
- Modern SLAM formulation (variant names):
  - Maximum-a-posteriori estimation, factor graph optimization, graph-SLAM, 
    full smoothing, or smoothing and mapping (SAM)
- Nonlinear filtering (covered in [Section 1](#slam1) & [Section 2](#slam2)) 
  vs. MAP estimation
  - MAP estimation is more accurate & efficient than Nonlinear filtering
  - Performance mismatch between Nonlinear filtering & MAP estimation gets
    smaller for nonlinear filtering methods if:
    - Linearization point for EKF becomes more accurate
    - Sliding-window filters are used
    - Potential sources of inconsistency of EKF are taken care of
- Why referred to as SLAM **_back-end_**?
  - MAP estimation is performed on pre-processed sensor data
      
### 2.2 SLAM Front-end
- Motivation
  - Hard to directly write the sensor measurements as an analytic state function
    required in MAP estimation
  - Unable to design a sufficiently general yet tractable environment 
    representation
- Sensor-dependent SLAM front-end
  - Set up a module called **_front-end_** before the SLAM back-end
  - Features
    - Extract relevant features from sensor data
    - Associate each measurement to a specific landmark (_data association_)
    - Provide an initial guess for the variables in the nonlinear optimization
      \eqref{eq3-4}


## 3. Long-term Autonomy I: Robustness
- SLAM system failure
  - Algorithmic
    - Limitation of current SLAM algorithms
  - Hardware-related
    - Sensor/actuator degradation
  - Software-related
- Algorithmic robustness challenges
  - Data association failures
    - Feature-based visual SLAM: _perceptual aliasing_ \( \to \) wrong 
      measurement-state matches (false positives) \( \to \) 
      wrong estimates from the back-end
    - Unmodeled dynamics in the environment \( \to \) deceive 
      short/long-term data association
      - Typical SLAM system: system based on _static world assumption_
      - Examples: 
        - Drastic illumination changes \( \to \) failure in system relying on
          the repeatability of visual features
        - Disappearances of old structures \( \to \) failure in system relying on
          methods leveraging the geometry of the environment
    - Harsh environment with limited visibility & constantly changing conditions
      & impossibility of using conventional sensors
  - Solutions for data association failures
    - Make front-end reliable for data association establishment
      - Short-term data association: ensure sensors are of high 
        sampling rate (high framerate)
        - Viewpoint of the sensor does not change significantly from
          time \( t \) to \( t+1 \)
      - Long-term data association 
        - Involving loop closure _detection_ and _validation_
        - Loop closure detection
          - Detect features in environment and then try to match them with
            all previously detected features
          - Use searching algorithms with low computational complexity:
            **_Bag-of-words_**
            - Quantizing feature space for more efficient search
          - Improve bag-of-words based techniques to handle severe 
            illumination variations
        - Loop closure validation
          - Ascertain the quality of loop closures using additional 
            geometric steps
          - Vision-based application: _RANSAC_ is used for geometric 
            verification & outlier rejection
          - Laser-based application: check residual error of laser scan matching
    - Make back-end resilient against spurious measurements
      - Spurious measurements (wrong loop closures): 
        caused by perceptual aliasing
      - Methods:
        - During optimization: reason on the loop closure constraints validity 
          by looking at the residual error induced by the constraints
        - Before optimization: identify incorrect loop closures that are not 
          supported by the odometry
    - Make SLAM system better deal with dynamic environments
      - Challenge 1: SLAM system has to detect, discard/track changes
        - Include dynamic elements as part of the model
      - Challenge 2: SLAM system has to model permanent/semi-permanent changes
        & understand how and when to update the map accordingly
        - Maintain multiple (time-dependent) maps of the same location
- Open problems
  a. Failsafe SLAM and recovery
     - Current SLAM solvers (based on iterative optimization of nonconvex costs)
       are vulnerable to outliers
     - Goal: _fail-safe_ & _failure-aware_
       - Be aware of imminent failures & provide recovery mechanisms that can 
         re-establish proper operation
  b. Robustness to HW failure
     - Goal: detect degraded sensor operation & adjust sensor noise statistics
       (covariances & biases) accordingly
  c. Metric relocalization: estimate relative pose w.r.t. previously built map
     - Hard/impossible for feature-based methods (rely on visual sensors) to
       detect loop closures among different times (day/night/diff. season/...)
     - Localizing from significantly different viewpoints with heterogeneous 
       sensing
  d. Time varying and deformable maps
     - Limitations on mainstream SLAM methods: rigid & static world assumption
     - Ideal SLAM method: be able to deal with environment dynamics
  e. Automatic parameter tuning
     - SLAM systems require extensive parameter tuning for a specific scenario
       - Especially for data association module
       - Parameters: 
         - Thresholds to control feature matching
         - RANSAC parameters
         - Criteria to decide when to add new factors to the graph or when to
           trigger a loop closing algorithm to search for matches
     - Goal: automatic parameter tuning for arbitrary scenarios


## 4. Long-term Autonomy II: Scalability
- SLAM methods should be designed to be scalable to the specific scenario
  - Different scenarios require different computational time & memory footprint
  - Computational & memory complexity should remain bounded
- Approaches to control/reduce growth of SLAM problem size
  - **_Sparsification_** methods: trade-off information loss for memory & 
    computational efficiency
    - Node & edge sparsification
      - **_Reducing_** number of nodes _added_ to the graph, or **_pruning_**
        less "informative" nodes & factors to address scalability
  - **_Out-of-core_** & **_multi-robot_** methods: split computation among
    many robots/processors
    - Out-of-core (parallel) SLAM
      - Split computation & memory load of factor graph optimization among
        multiple processors
      - Key idea:
        global factor graph optimization \( \to \) localized subgraph 
        optimization + global refinement
      - Called _submapping_ algorithms
    - Distributed multi robot SLAM
      - Divide the scenario in smaller areas by deploying multiple robots
        - Map each area by a different robot
      - Two main variants
        - _Centralized_ method: robots build submaps & transfer local info to
          a central station that performs inference
        - _Decentralized_ method: no central data fusion & agents leverage 
          local communication to reach consensus on a common estimate
- Open problems
  a. Map maintenance
     - Localization against a compressed known map
     - Memory-efficient dense reconstruction
  b. Robust distributed mapping
     - Outliers are ignored in multi-robot SLAM
     - Challenges
       - Robots may not share a common reference frame which makes it harder to
         detect & reject wrong loop closures
       - Robots have to detect outliers from very partial/local information
  c. Learning, forgetting, remembering
     - Question for long-term mapping: determine the followings
       - How often to update the map information
       - How to decide when the information becomes outdated & can be discarded
       - When is it fine to forget
       - What can be forgotten & what is essential to maintain
  d. Resource-constrained platforms
     - How to adapt existing SLAM algorithms for robot platforms with
       severe computational constraints
     - How to guarantee reliable operation for multi robot teams given tight 
       bandwidth constraints & communication dropout


## 5. Representation I: Metric Reasoning
- **_Metric representation_** (metric map): symbolic structure that encodes
  the geometry of the environment
- Geometric modeling in SLAM in 2D case
  - _Landmark-based maps_: model the environment as a sparse set of landmarks
  - _Occupancy grid maps_: discretize the environment in cells and assign a 
    probability of occupation to each cell
- Geometric modeling in SLAM in 3D case
  a. Landmark-based sparse representations
     - Represent scene as a set of sparse 3D landmarks corresponding to 
       discrimitive features (lines/corners/...) in the environment
     - Referred to as landmark-based / feature-based representations
     - Application:
       - Localization & mapping in mobile robotics
       - _Structure from motion_ in computer vision
     - Assumption: landmarks are distinguishable
       - Sensor data measure geometric aspect of a landmark & provide a 
         descriptor for data association between measurement & landmark
     - Features: point (mainly), lines, segments, or arcs
  b. Low-level raw dense representations
     - Provide high-resolution models for 3D geometry
     - Application
       - Obstacles avoidance & path planning in robotics
       - Visualization & rendering
     - Raw representations
       - Large unstructured set of points: **_point clouds_**
         - In conjunction with stereo & RGB-D cameras, and 3D laser scanners
       - Polygons: **_polygon soup_**
       - **_Surfels maps_**: encode the geometry as a set of disks
     - Application in monocular SLAM: in conjunction with the use of 
       _direct methods_
       - Estimate robot trajectory & 3D model directly from image pixel 
         intensity values
     - Pros & cons
       - Pro: visually pleasant
       - Cons: 
         - Cumbersome (require storing of large amount of data)
         - Low-level geometry description which neglects obstacle topology
  c. Boundary and spatial-partitioning dense representations
     - Attempt to explicitly represent surfaces/_boundaries_ and volumes
     - Boundary representations (b-reps): define 3D objects in terms of 
       their surface boundary
       - Plane-based models
       - _Curve-based representations_: tensor product of NURBS or B-splines
       - _Surface mesh models_: connected set of polygons
       - _Implicit surface representations_
         - Specify the surface of a solid as the zero crossing of a function
           defined on \( \mathbb{R}^3 \)
         - Function examples
           - Radial-basis functions
           - Signed-distance function
           - **_Truncated signed-distance function (TSDF)_**
     - Spatial-partitioning representations
       - Spatial-occupancy enumeration: decompose 3D space into identical 
         cubes (voxels) arranged in a regular 3D grid
       - Octree: used for 3D mapping
         - Occupancy grid maps: probabilistic variants of space-partitioning
           representations
       - Polygonal Map octree
       - Binary Space-partitioning tree
       - 2.5D elevation maps
  d. High-level object-based representations
     - Features of object & solid shape representations
       - Model objects as 3D objects rather than 1D (points) & 2D (surfaces)
       - Associate physical notions (volume, mass, ...) to each object
     - Solid representations in CAD & computer graphics (no applications in
       SLAM yet)
       ([intro](http://designer.mech.yzu.edu.tw/articlesystem/article/compressedfile/(2010-12-10)%20Constructive%20solid%20geometry%20and%20sweep%20representation.aspx?ArchID=1616))
       - _Parameterized Primitive Instancing_
         - Define a families of objects (cylinder, sphere, ...)
         - Define a set of parameters (radius, height, ...) to each member 
           of the family
       - _Sweep representations_ (see the intro above)
         - Define a solid as the sweep of a 2D/3D object along a trajectory 
           through space
       - _Constructive solid geometry_ (see the intro above)
     - Feature-based models in CAD
     - Dictionary-based representations
       - Define a solid as a combination of atoms in a dictionary
       - Dictionary source:
         - Learned from data 
         - Existing repositories of object models
     - Affordance-based models
     - Generative & procedural models
     - Scene graphs
- Sparse (feature-based) vs. dense representations/algorithms in visual SLAM
  - Disadvantages of sparse representations
    - Dependence of feature type
    - Reliance on numerous detection & matching thresholds
    - Necessity for robust estimation methods to deal with wrong correspondences
    - Most feature detectors are optimized for speed rather than precision
  - Dense/direct methods
    - Exploit all image information
    - Outperform feature-based methods in scenes with poor texture, defocus &
      motion blur
    - Require high computing power for real-time performance
    - Currently unable to jointly estimate dense structure & motion
  - Alternatives to feature-based methods
    - _Semi-dense_ methods: only exploit pixels with strong gradients
    - _Semi-direct_ methods: leverage both sparse features & direct methods
      - Most efficient alternative
      - Allow joint estimation of structure & motion
- Open problems
  a. High-level, expressive representations in SLAM
     - Point clouds or TSDF are mainly used to model 3D geometry
       - Drawbacks
         - Wasteful for environment modeling
         - No high-level understanding of 3D geometry
     - Advantages of higher-level representations
       - Compact representations \( \to \) compressed map in large-scale mapping
       - Higher-level object description facilitates data association,
         place recognition & semantic understanding
       - Enable interactions with existing modern building construction & 
         management standards
     - Current representations in SLAM are only low-level point clouds, 
       mesh models, surfels models, and TSDFs
  b. Optimal representations
     - Lack of criteria to choose an appropriate representation for specific 
       environment
  c. Automatic & adaptive representations
     - Choice of representation is entrusted to SLAM system designer
       - Drawbacks
         - Representation design is time-consuming & require an expert
         - No flexibility: representation cannot be changed once the 
           system is designed


## 6. Representation II: Semantic Reasoning
- Features of semantic mapping
  - Associate semantic concepts to geometric entities in robot's surroundings
  - Enhance robot's autonomy & robustness
  - Facilitate more complex tasks (e.g. avoid muddy-road while driving)
  - Move from path-planning to task-planning
  - Enable advanced human-robot interaction
  - Typical semantic parsing: formulated as a classification problem
    - Simple mapping between sensory data & semantic concepts
- Semantic vs. topological SLAM
  - Semantic: interested in classifying the places according to semantic labels
  - Topological: do place recognition only for topological mapping (disregard 
    the actual function of a place (kitchen/corridor/...)
- Design of a semantic representation
  - Level/detail of semantic concepts: determined by specific tasks
  - Organization of semantic concepts
    - Flat/hierarchical organization
    - Sharing/not sharing properties
- Semantic mapping in SLAM
  - SLAM helps semantics
    - Segmenting the map built from classical SLAM systems into 
      semantic concepts
  - Semantics helps SLAM
    - Improve map estimation by recognizing semantic concepts in the map
  - Joint SLAM & semantics inference
    - Perform monocular SLAM & map segmentation within a joint formulation
- Open problems
  a. Semantic SLAM lacks cohesive formulation
  b. Semantic mapping is much more than a categorization problem
     - How to represent interrelationships among semantic concepts
  c. Ignorance, awareness & adaption
     - Given prior knowledge, a robot should be able to reason about new 
       concepts & their semantic representations
       - Discover new objects
       - Learn new properties
       - Adapt representations to slow/abrupt changes in the environment
  d. Semantic-based reasoning
     - Robots are currently unable to effectively localize & continuous map
       using semantic concepts in the environment
       - Unable to update & refine map by detecting new semantic concepts


## 7. New Theoretical Tools for SLAM
- Focus
  - Factor graph optimization approaches
  - _Algorithmic properties_
- MAP estimation \( \to \) MLE if no prior is given
  - SLAM inherits all properties of maximum likelihood estimators
  - MLE properties
    - Consistent
    - Asymptotically Gaussian & asymptotically efficient
    - Invariant to transformations in the Euclidean space
  - Estimator will no longer be invariant if priors are given
- SLAM: _nonconvex_ problem
  - Iterative optimization can only guarantee local convergence
  - Local minimum \( \to \) completely wrong estimation & unsuitable for 
    navigation
  - State-of-the-art iterative solvers fail to converge to global minimum
    for relatively small noise levels
- Progress on theoretical analysis of SLAM problem
  - Rotation estimation can be solved uniquely in closed form in 2D
  - Convergence improvement approaches
    - (Convex) dual **_semidefinite programming (SDP)_**:
      under certain conditions often encountered in practice (strong duality),
      ML estimation is unique & pose graph optimization can be solved globally
      via this approach
    - Convex relaxation: avoid local minima convergence
    - Compute a suitable initialization for iterative nonlinear optimization
      - Effective idea: solve for rotations first, and then use the resulting 
        estimate to bootstrap nonlinear iteration
  - Verification techniques to judge whether the quality of given SLAM solution
    is optimal or not
    - Based on Lagrangian duality in SLAM
    - Current progress: perform verification by solving a sparse linear system
      and provide a correct answer as long as strong duality holds
- Open problems
  a. Generality, guarantees, verification
     - Can guaranteed solution & verification techniques in context of pose 
       graph optimization be generalized to arbitrary factor graphs?
  b. Weak or strong duality?
     - SLAM can be solved globally when strong duality holds
     - Given a set of sensors & a factor graph, does the strong duality hold?
  c. Resilience to outliers
     - Design of global techniques & verification techniques that are resilient
       to outliers remains open
    

## 8. Active SLAM
- Active SLAM
  - Control robot's motion to minimize the uncertainty of its map 
    representation & localization
  - Other names: 
    - Simultaneous Planning Localization and Mapping (SPLAM)
    - Autonomous/Active/Integrated Exploration
- General frameworks for decision making in active SLAM
  - Goal: solve for exploration-exploitation decisions
  - Frameworks
    - Theory of Optimal Experimental Design (TOED)
      - Allow selecting future robot motion based on predicted map uncertainty
    - Information theoretic approaches
      - Decision making is guided by _information gain_
    - Control theoretic approaches
      - Use Model Predicative Control
    - Partially Observably Markov Decision Process (POMDP)
      - Computationally intractable
    - Bayesian Optimization
    - Efficient Gaussian beliefs propagation
- Framework based on selecting future robot motion among a 
  finite set of alternatives
  - Step 1 - _selecting vantage points_: 
    robot identifies possible locations (vantage locations) to explore/exploit
    - Only a small subset of locations are selected as evaluation grows 
      exponentially with the search place
    - Related approaches only guarantee local optimal policies
  - Step 2 - _computing the utility of an action_: 
    robot computes the utility of visiting each vantage point and
    select the action with highest utility
    - Computationally intractable for precise information gain of each action
    - Compute approximations: utility defined as linear combination of
      metrics that quantify robot & map uncertainties
      - Drawback: scale of numerical uncertainties not comparable
    - TOED can also be applied
  - Step 3 - _executing actions or terminate exploration_: 
    robot carries out selected action and decides if necessary to 
    continue/terminate the task
    - Open challenge for the decision whether exploration task is done
- Open problems
  a. Fast and accurate predictions of future states
     - Necessary because each robot action should reduce the uncertainty in 
       map & improve localization accuracy in active SLAM
     - Loop closure is important: no efficient methods exist
     - Predicting effects of future actions is computationally expensive
  b. Enough is enough: when to stop active SLAM?
     - Active SLAM is computationally expensive, need to stop and focus 
       resources on other tasks
  c. Performance guarantees
     - Need mathematical guarantees for active SLAM & near-optimal policies
       - Exact solution is computationally intractable
       - Desirable for approximation algorithms with clear performance bounds


## 9. New Sensors for SLAM
- Progress in SLAM are partly triggered by novel sensors
  - 2D laser range finders: enable very robust SLAM algorithms
  - 3D lidars: main thrust behind applications such as autonomous cars
  - Vision sensors: used in augmented reality & vision-based navigation
- Limitation of current vision sensors
  - High latency & temporal discretization
    - Latency: time to capture image & time to process it
    - Temporal discretization: time to wait until the next frame arrives
  - Prevent the advent of high-agility autonomous robot
- **_Event-based cameras_** 
  - May substitute current vision sensors
  - Typical cameras: 
    - Dynamic Vision Sensor (DVS)
    - Asynchronous Time-based Image Sensor (ATIS)
  - Event cameras vs. standard frame-based cameras
    - Standard frame-based cameras: send entire images at fixed frame rates
    - Event cameras: only send local pixel-level changes caused by movement
      in a scene at the time it they occur
      - Event is triggered if relative intensity change exceeds a threshold
  - _Microsecond time_ resolution for event cameras
  - 5 key advantages over standard frame-based cameras
    a. Temporal latency: 1 microsecond
    b. Measurement rate: up to 1MHz
    c. Dynamic range: up to 140 dB (vs. 60-70 dB of standard cameras)
    d. Power consumption: 20 mW (vs. 1.5W of standard cameras)
    e. Bandwidth for data transfer: kilobytes/s scale (vs. Megabytes/s)
  - Disadvantages
    - Traditional frame-based computer-vision algorithms are not applicable
      - Sensor output composes of a sequence of asynchronous events instead
        of image frame
    - Require a _paradigm shift_ from traditional computer vision approaches
- Progress on event camera applications
  - Algorithmic adaptions: 
    - Feature detection & tracking
    - LED tracking
    - Epipolar geometry
    - Image reconstruction
    - Motion estimation
    - Optical flow
    - 3D reconstruction
    - Iterative closest points
  - Event-based visual odometry, localization & SLAM
    - Design goal: each incoming event can asynchronously change 
      estimated system state
    - Adaptions of continuous-time trajectory-estimation methods
      - Pose trajectory can be _approximated_ by a smooth curve in rigid-body 
        motion space using basis functions (cubic splines) & _optimized_
        according to the observed events
- Open problems
  a. Modeling, sensitivity and resolution
     - Events in event-based cameras are highly susceptible to noise
     - Lack of full characterization of sensor noise & sensor non idealities
     - Event pipelines may be failed for scenes with smooth edges as events
       are triggered by brightness changes
     - Low spatial resolution (128x128 pixels)
  b. Event-based or standard camera for SLAM?
     - No criteria for choosing algorithm-sensor pair

\newpage
