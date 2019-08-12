# Notes on SLAM Tutorial Part II [@Bailey2006] ([paper](http://ieeexplore.ieee.org/iel5/100/35300/01678144.pdf)) {#slam2}

## 1. Focus
- Tutorial focus: recursive Bayesian formulation of the SLAM problem where
  probability distributions/estimates of absolute/relative landmark 
  locations & vehicle poses are obtained
- 3 Key areas in SLAM research in this tutorial:
  a. Computational complexity
     - Approaches to reduce the computational complexity
       - Linear-time state augmentation
       - Sparsification in information form
       - Partitioned updates
       - Submapping methods
  b. Data association
     - Methods to correctly associate landmark observations 
       with landmarks in the map
       - Batch-validation methods
       - Appearance-based methods
       - Multihypothesis techniques
  c. Environment representation
     - Related to appearance-based models of landmarks and maps
       - Delayed mapping
       - Use of nongeometric landmarks
       - Trajectory estimation methods
- Use mathematical notations defined in SLAM Part I tutorial


## 2. Computational Complexity
- Exploit the structure to limit computational complexity of SLAM algorithm
  - Process/Motion model \( P(\V{x}_k | \V{x}_{k-1}, \V{u}_k) \):
    only affect vehicle pose states
  - Observation model \( P(\V{z}_k | \V{x}_k, \V{m}) \):
    only make reference to single vehicle-landmark pair
- Category of techniques:
  - **_Optimal_**: _reduce_ required computation and get estimates & 
    covariances that are **_equal to_** the full-form SLAM algorithm
  - **_Conservative_**: computationally more efficient but **_less accurate_**
    - Termed **_inconsistent_** & regarded as **_invalid solutions_** 
      to the SLAM (or any estimation) problem
- Methods:
  - State augmentation:
    - Re-formulating time-update equations
    - Optimal
  - Partitioned updates:
    - Re-formulating observation-update equations
    - Optimal
  - Sparsification in information form
    - Re-formulating standard state-space SLAM representation into 
      information form
    - Usually conservative
  - Submapping methods
    - Provide estimates in global frame
    - Conservative
      
### 2.1 State Augmentation
- SLAM state vector \( \V{x}_k = [\V{x}_{vk}^T, \V{m}^T]^T \) (where 
  \( \V{x}_{vk} \) denotes the vehicle state vector at time \( k \) )
- Covariance computation in a naive implementation of EKF-SLAM:
  \[
      \V{P}_{k|k-1} 
      = \nabla \V{f}_{\V{x}} \V{P}_{k-1|k-1} \nabla \V{f}_{\V{x}}^T + 
        \nabla \V{f}_{\V{u}} \V{U}_k \nabla \V{f}_{\V{u}}^T
  \]
  where \( \nabla \V{f}_{\V{x}} = \partial \V{f} / \partial \V{x}_{k-1} \),
  \( \nabla \V{f}_{\V{u}} = \partial \V{f} / \partial \V{u}_k \), and
  \( \V{U}_k \) is the covariance characterising uncertainty on control vector
- Exploit the fact that motion model only affects vehicle pose states:
  \[
      \V{P}_{k|k-1}
      = \begin{bmatrix}
         \V{P}_{vv}   & \V{P}_{vm} \\
         \V{P}_{vm}^T & \V{P}_{mm} \\
        \end{bmatrix}
      = \begin{bmatrix}
         \nabla \V{f}_{v_\V{x}} \V{P}_{vv} \nabla \V{f}_{v_\V{x}}^T 
         + \nabla \V{f}_{v_\V{u}} \V{U}_k \nabla \V{f}_{v_\V{u}}^T &
         \nabla \V{f}_{v_\V{x}} \V{P}_{vm} \\
         \V{P}_{vm}^T \nabla \V{f}_{v_\V{x}}^T & \V{P}_{mm} \\
        \end{bmatrix}
  \]
  where 
  \( \nabla \V{f}_{v_\V{x}} = \partial \V{f}_v / \partial \V{x}_{v(k-1)} \) and
  \( \nabla \V{f}_{v_\V{u}} = \partial \V{f}_u / \partial \V{u}_{k} \)
- Adding new landmarks to the SLAM state vector at a new state \( k \):
  \[ 
      \V{x}_k^+
      = \begin{bmatrix}
         \V{x}_{vk} \\ 
         \V{m} \\ 
         \V{m}_{new} \\
        \end{bmatrix}
      = \begin{bmatrix}
         \V{x}_{vk} \\ 
         \V{m} \\ 
         g(\V{x}_{vk}, \V{z}_k) \\
        \end{bmatrix}
  \]
  where the new map landmark(s) are initialized by a function of 
  vehicle pose \( \V{x}_{vk} \) and the current observation \( \V{z}_k \)
- Apply state augmentation whenever new states are a function of a subset
  of existing states:
  \[
      \V{x}^+ 
      = \begin{bmatrix}
         \V{x}_1 \\
         \V{x}_2 \\
         f(\V{x}_2, \V{q}) \\
        \end{bmatrix}
  \]
  \[
      \V{P}^+
      = \begin{bmatrix}
         \V{P}_{11}   & \V{P}_{12} & \V{P}_{12} \nabla \V{f}_{\V{x}_2}^T \\
         \V{P}_{12}^T & \V{P}_{22} & \V{P}_{22} \nabla \V{f}_{\V{x}_2}^T \\
         \nabla \V{f}_{\V{x}_2} \V{P}_{12}^T &
         \nabla \V{f}_{\V{x}_2} \V{P}_{22} &
         \nabla \V{f}_{\V{x}_2} \V{P}_{22} \nabla \V{f}_{\V{x}_2}^T
         + \nabla \V{f}_{\V{q}} \V{Q} \nabla \V{f}_{\V{q}}^T
        \end{bmatrix}
  \]
  where the dimensions are:
  \[
      \mathrm{dim}\ \V{x}^+
      = \begin{bmatrix}
         a \times 1 \\
         b \times 1 \\
         (b + c) \times 1 \\
        \end{bmatrix}
  \]
  \[
      \mathrm{dim}\ \V{P}^+
      = \begin{bmatrix}
        a       \times a & a       \times b & a       \times (b + c) \\
        b       \times a & b       \times b & b       \times (b + c) \\
        (b + c) \times a & (b + c) \times b & (b + c) \times (b + c) \\
        \end{bmatrix}
  \]

### 2.2 Partitioned Updates
- Instead of updating **_all vehicle & map states_** every time when a new 
  measurement is made, only update a **_small local region_** of states and
  update the global map only at a **_much slower frequency_**
- 2 basic types:
  - Type 1: operating on a local region of the global map and maintains 
    globally referenced coordinates
    - Key methods:
      - Compressed EKF (CEKF)
      - Postponement algorithm
  - Type 2: generating a short-term submap with its own local coordinate
    frame
    - Key methods:
      - Constrained local/relative submap filter (CLSF/CRSF)
      - Local map sequencing algorithm
    - Simpler & more numerically stable & less affected by 
      linearization errors
- CLSF/CRSF:
  - Maintain 2 independent SLAM estimates at all times:
    \[
        \V{x}_G = \begin{bmatrix} \V{x}_F^G \\ \V{m}_G \end{bmatrix}, \quad
        \V{x}_R = \begin{bmatrix} \V{x}_v^R \\ \V{m}_R \end{bmatrix}
    \]
    - \( \V{x}_G \): map composed of global reference pose of a submap 
      coordinate frame \( \V{x}_F^G \) and a set of global referenced 
      landmarks \( \V{m}_G \)
    - \( \V{x}_R \): local submap with a locally referenced vehicle pose
      \( \V{x}_v^R \) and locally referenced landmarks \( \V{m}_R \)
  - Conventional SLAM updates are performed entirely within the local submap
    - Only those landmarks held in the local submap will be updated
  - Obtain global vehicle pose estimate at any time by **_vector summation_**
    of _locally referenced pose_ and the _global estimate_ of the 
    submap coordinate frame
  - Optimal global estimate is obtained periodically by **_registering 
    the submap with the global map_** and **_applying constraint updates on
    any common features of both maps_**
  - Advantages:
    - At any one instance, the number of landmarks that must be updated 
      is limited to those described in the local submap coordinate frame only
      - Observation-rate update is independent of the total map size
      - Full update can be performed in a background task at a much lower rate
        as long as the observation-rate update can be performed
    - Uncertainty is lower in a locally referenced frame
      - Linearization error is reduced
    - Association robustness can be improved by using _batch-validation 
      gating_ in submap registration 

### 2.3 Sparsification
- Represent Gaussian probability density in canonical/information form 
  rather than using the 1st & 2nd moment \( \V{\hat{x}}_k \) & \( \V{P}_k \)
  - Information matrix \( \V{Y}_k = \V{P}_k^{-1} \)
  - Information vector \( \V{\hat{y}}_k = \V{Y}_k \V{\hat{x}}_k \)
- Many off-diagonal components of the _normalized_ information matrix are
  very close to 0
- An exactly sparse solution can be obtained by augmenting the state vector
  with new vehicle pose estimate at every time step and retaining all past
  robot poses:
  - \( \V{x}_k
       = [\V{x}_{vk}^T, \V{x}_{vk-1}^T, \dots, \V{x}_{v1}^T, \V{m}^T]^T \)
  - Nonzero off-diagonal components: only for poses & landmarks that are 
    directly related by measurement data
  - Control sparsity by utilizing **_marginalization_** to remove past states
- Caveat: necessary to recover mean & covariance of the state at every time step
  - Mean estimate is required to perform linearization of 
    process & observation models
    - Efficient recovery by conjugate gradient method
  - Both mean & covariance are required to compute validation gate for 
    data association
    - Robust batch gating methods require the full recovery of covariance 
      matrix, which has a **_cubic_** complexity in the number of landmarks

### 2.4 Submapping Methods
- Another category of methods to deal with the **_quadratically scaling 
  computation complexity_** with the number of landmarks during measurement
  updates
- 2 categories: globally (a) & locally referenced submaps (b)  
  ![Submapping Methods](images/ch02/01_submapping_methods.jpg){ width=60% }
- Global submaps
  - Estimate global locations of submap coordinate frames relative to 
    a common base frame
  - Cannot alleviate linearization issues arising from large pose uncertainties
- Local/Relative submaps
  - Location of any given submap is recorded only by its neighboring submaps
  - Submaps are connected in a graphical network
  - Obtain global estimate by vector summation along a path in the network
  - Can avoid linearization issues because global-level data fusion is eschewed


## 3. Data Association
- _New measurements_ are associated with _existing map landmarks_ 
  before fusing data into the map
  - The associations **_cannot be revised_** after fusion
- Any single incorrect data association can lead to failure of the 
  SLAM algorithm
  
### 3.1 Batch Validation
- Individual gating
  - Consider each measurement-to-landmark association individually
  - Test whether an observed landmark is close to a predicted location 
  - Extremely unreliable if the vehicle pose is very uncertain
- Batch gating
  - Multiple associations are considered simultaneously
  - Exploit geometric relationship between landmarks

### 3.2 Appearance Signatures
- Extract landmark appearance features as its signature 
- Use **_similarity metrics_** such as an image similarity metric to 
  predict a possible association

### 3.3 Multihypothesis Data Association
- Essential for robust target tracking in cluttered environment
- Generate separate track estimate for each association hypothesis, and create
  an ever-branching tree of tracks over time
- Prune low-likelihood tracks from the hypothesis tree


## 4. Environment Representation
- Assumption in early SLAM works: the world can be reasonably modeled as 
  **_a set of simple discrete landmarks_** described by 
  **_geometric primitives_** such as points, lines or circles
  - Assumption does not hold for more complex & unstructured environments

### 4.1 Partial Observability and Delayed Mapping
- Limitations on sensing modality
  - Sonar: produce accurate range measurement but unusable bearing estimate
  - Vision (camera): provide bearing info without accurate range indication
- Single measurement (range-only / bearing-only sensors) is **_insufficient_**
  to constrain a landmark location
  - Need be observed from multiple vantage points (e.g., the intersection 
    of regions mapped by multiple measurements)
  - Single measurement can only generate a non-Gaussian distribution over
    landmark location
    - Example for a Gaussian landmark estimate: central position & radius
- Delayed mapping: obtaining Gaussian landmark estimate from 
  range-only / bearing-only sensors by **_delaying initialization_**
  & **_accumulating raw measurement data_**
  - Record vehicle pose for each deferred measurement
    - Augment SLAM states with recorded vehicle pose estimates  
      (\( \V{x}_k = [\V{x}_{v_k}^T, \V{x}_{v_{k-1}}, \dots, 
                     \V{x}_{v_{k-n}}^T, \V{m}^T]^T \))
    - Store corresponding measurements in an auxiliary list
      (\( \{\V{z}_k, \dots, \V{z}_{k-n}\} \))
  - Initialize a landmark by a batch update once sufficient 
    information is accumulated
  - Remove recorded poses that do not have any associated measurements
- Delayed fusion
  - General concept for increasing robustness by **_accumulating
    information_** and **_permitting delayed decision making_**
  - Advantages:
    - Reduce linearization errors by performing a batch update (such as
      _bundle adjustment_ (BA)) on accumulated data set 
    - Facilitate batch validation gating & aid reliable data association

### 4.2 Nongeometric Landmarks
- EKF-SLAM is usually applied to geometric landmarks
- Nongeometric landmarks: landmarks with arbitrary shape
  - Described by a shape model with an embedded coordinate frame whose 
    origin defines the landmark origin
- EKF-SLAM with nongeometric landmarks: separate shape parameter estimation
  from landmark locations

### 4.3 3-D SLAM
- 3 essential forms
  a. 2-D SLAM with additional map building ability in 3rd dimension
  b. Direct extension of 2-D SLAM 
  c. SLAM with joint state containing history of past vehicle poses
     - Obtaining 3-D environment scan at each pose

### 4.4 Trajectory-Oriented SLAM
- Estimate **_vehicle trajectory_** instead of pose + landmark
- Type 1:
  - Joint state \( \V{x}_k \): 
    \( [\V{x}_{vk}^T, \V{x}_{v_{k-1}}^T, \dots, \V{x}_{v_1}^T] \)
  - Particularly suited to environments where 
    - Discrete landmarks are not easily discerned
    - Direct alignment of sensed data is easier / more reliable
  - Map is not part of the state but forms an auxiliary data set
    - Map is formed by aligning associated scan of sensed data 
      from each pose estimate
- Type 2 (Consistent Pose Estimation (CPE)):
  - Connect pose in a graphical network instead of forming a joint state vector
- Type 3:
  - Sparse-information-form SLAM with sparse estimation of joint state 
    vector of type 1
- Caveats:
  - Unbounded state-space & quantity of stored measurement data
    - Necessary to coalesce data to bound storage costs

### 4.5 Embedded Auxiliary Information
- Soil salinity, humidity, temperature, terrain characteristics, etc...
- May be used to assist mapping, aid data association or for tasks unrelated 
  to mapping (path planning / data gathering)
- More difficult to be incorporated within traditional SLAM framework

### 4.6 Dynamic Environments
- Dynamic landmarks:
  - Moving objects such as people
  - Temporary structures that appear static but later moved 
    such as chairs / parked cars
- Property: 
  - Landmarks can be removed from the map without loss of consistency
    - Large number of landmarks can be removed without change of 
      convergence rate
- Ideas:
  - Removing landmarks that are obsolete (due to environment changes)
  - Implement auxiliary identification routine to remove dynamic info 
    from a data scan before sending it to SLAM algorithm
  - Tracking both stationary & moving targets
    - Reduce cost by implementing stationary SLAM update followed by 
      separate tracking of moving objects

\newpage
