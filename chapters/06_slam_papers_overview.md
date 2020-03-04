# Overview of Various SLAM-related Papers

## 1. Classic Papers

### 1.1 Direct-Based Systems

#### 1.1.1 SVO - Fast Semi-Direct Monocular Visual Odometry [@Forster2014SVO]

- [Paper](https://infoscience.epfl.ch/record/199740/files/ICRA14_Forster.pdf)

##### 1.1.1.1 Overview

- System diagram:  
  ![System Diagram](images/ch06/forster2014_fig_01_diagram.jpg){ width=50% }

- System overview:

  - Motion estimation thread
    - Pose initialization: through sparse model-based image alignment
      - Find relative pose \( \V{T}_{k,k-1} \)
    - Pose & structure refinement: BA procedure
    
  - Mapping thread
    - Depth-filter
      - Initialized with a large uncertainty in depth
      - Updated in a Bayesian fashion
      - Depth is computed from the point in non-reference frame
        with the highest correlation (the point lying on the epipolar line) 
        with the reference patch
    - Add new 3D point only if the depth-filter is converged (i.e., depth
      uncertainty is small enough)
    
##### 1.1.1.2 Motion Estimation

- Procedures:

  1) Sparse Model-based Image Alignment  
     ![Sparse Alignment](images/ch06/forster2014_fig_02_sparse_img_align.jpg){ width=50% }
     - Goal: compute relative pose \( \V{T}_{k,k-1} \) from frame \( k-1 \) to
       frame \( k \) through minimization of photometric error of image patches
     - Minimization of reprojection error through Gauss-Newton procedure
     - Optimize both camera pose and image patch positions in the current frame
  
  2) Feature Alignment  
     ![Feature Alignment](images/ch06/forster2014_fig_03_feat_align.jpg){ width=50% }
     - Goal: further minimize photometric error of feature patches by estimating
       new feature positions in current frame w.r.t. reference keyframe
     - Solved using inverse compositional Lucas-Kanade algorithm [@Baker2004LK]
       - *Inverse compositional*: apply the delta parameters on the
         template/reference image during an iteration of optimization, instead
         of the current image, and then update the parameters with the inverse 
         of the delta parameters
     - Camera pose is not involved in the minimization scheme
     - Only the feature patch positions are optimized
     - A relaxation scheme: because of the violation of epipolar constraint
       for higher correlation between the feature patches
  
  3) Pose & Structure Refinement  
     ![BA](images/ch06/forster2014_fig_04_ba.jpg){ width=50% }
     - Step 1: motion-only BA: only optimize the camera pose
     - Step 2: structure-only BA: only optimize the 3D points
     - Step 3: (optional) local BA: optimize both local poses and 3D points
     - Minimization of reprojection error through Gauss-Newton procedure

- Advantages of the above procedures combined:

  - Advantage of the 1st procedure
    - The procedure implicitly satisfies epipolar constraint and ensures 
      there's no outliers
    - Without this procedure, the tracking will be slower because
      of the tracking of all features over large distances, whereas this
      procedure only optimizes 6 parameters
  - Advantage of the combination of the 3 procedures:
    - 2nd and 3rd procedures are necessary to reduce drift
      - 3rd procedure: image alignment w.r.t. the map and keyframes
      - 1st procedure only aligns image w.r.t. the previous frame

##### 1.1.1.3 Mapping

- Depth-filter in SVO:  
  ![Depth Filter](images/ch06/forster2014_fig_05_depth_filter.jpg){ width=50% }
  - Modification to the original depth-filter in [@Vogiatzis2011DF]

- Find depth of a image point via depth-filter associated with a keyframe
- Search for a patch on the epipolar line in the new image that has the 
  highest correlation (most probable depth) with the reference patch
- Depth measurement is modeled with a Gaussian + uniform mixture model
  - Gaussian distribution for inlier
  - Uniform distribution for outlier
  - Weighted average of the 2 distributions
    - Weight: inlier ratio \( \pi \)
- Depth-filter update:
  - Depth posterior: product of a **Gaussian** for the *depth* \( Z \) with a 
    **Beta** distribution for the *inlier ratio* \( \pi \) as an *approximation*
    - \( Z \sim \mathcal{N}(\mu, \sigma^2) \), \( \pi \sim Beta(a, b) \)
  - Bayesian update: 
    \( p(Z, \pi | x_n) = \frac {p(x_n | Z, \pi) p(Z, \pi)} {p(x_n)} \)
    - Prior: \( p(Z, \pi) = p(Z) p(\pi) \)
    - Likelihood: \( p(x_n | Z, \pi) \)
    - Posterior \( p(Z, \pi | x_n) \) approximated by a \( Gaussian \times Beta \)
      distribution \( q(Z, \pi | a, b, \mu, \sigma^2) \)
    - Update step: 
      - Match the 1st and 2nd moment of 
        \( p(x | Z, \pi) q(Z, \pi | a, b, \mu, \sigma^2) \)
        and \( q(Z, \pi | a', b', \mu', \sigma'^2 \)
      - Solve 4 equations to get \( a' \), \( b' \), \( \mu' \), and 
        \( \sigma'^2 \), and substitute the 4 old parameters, and also
        get the new measurement \( x \)

##### 1.1.1.4 Implementation Details

- System initialization:
  - Assume a locally planar scene and estimate a homography: compute 
    1st pose via decomposition of \( \V{H} \)

- Sparse image alignment
  - Coarse-to-fine scheme: utilize a image pyramid of 5 levels with scale 
    factor of 2
  - Optimization on the coarsest level for an initial solution
  - Continue optimization on finer level and stop after convergence on the 
    3rd level

- Mapping
  - Fixed number of keyframes
    - Queue: new keyframe added will cause the farthest keyframe to be removed
  - Keyframe selection scheme: Euclidean distance of the new frame relative to
    all keyframes exceeds 12% of the average scene depth
  - Feature extraction
    - Divide the image in cells of fixed size (\( 30 \times 30 \) pixels)
    - Extract FAST corners with the highest Shi-Tomasi score in each image cell
      at *all levels* of the pyramid
      - Skip this step if the cell already has a 2D-to-3D correspondence

#### 1.1.2 LSD-SLAM: Large-scale Direct Monocular SLAM [@Engel2014LSD]

- [Paper](https://vision.in.tum.de/_media/spezial/bib/engel14eccv.pdf)

##### 1.1.2.1 Overview

- System diagram:  
  ![Overview](images/ch06/engel2014_fig_03_lsd_slam.jpg)

- Features
  - Direct method to align 2 keyframes on \( \mathfrak{sim}(3) \)
    - Incorporate and detect scale-drift in a novel scale-aware image alignment
      algorithm
    - Detect loop closures
  - Probabilistic approach to incorporate noise on the estimated depth maps
    into tracking
  - A semi-dense map represented as point clouds
  - Tracking in environment with large variations in scale and large rotation

##### 1.1.2.2 Complete Method

- 3 major components
  - **Tracking**
    - Estimates pose \( \BG{\xi} \in \mathfrak{se}(3) \) w.r.t. the current
      keyframe, using the previous frame as initialization
  - **Depth Map Estimation**
    - Uses tracked frames to either *refine* or *replace* the current keyframe
  - **Map Optimization**
    - Detect loop closures and scale-drift by estimating a similarity transform
      \( \BG{\xi} \in \mathfrak{sim}(3) \) to close-by existing keyframes
      in a scale-aware & direct \( \mathfrak{sim}(3) \)-image alignment scheme

- Initialization scheme
  - Initialize a first keyframe with a *random* depth map and large variance

##### 1.1.2.3 Map Representation

- Global map as pose graph
- Vertices: keyframes \( \mathcal{K}_i \)
  - Camera image \( I_i: \Omega \to \F{R} \)
  - Inverse depth map \( D_i: \Omega_{D_i} \to \F{R}^+ \) where 
    \( \Omega_{D_i} \subset \Omega_i \)
  - Variance of inverse depth \( V_i: \Omega_{D_i} \to \F{R}^+ \)
- Edges \( \mathcal{E}_{ji} \)
  - Relative alignment \( \BG{\xi}_{ji} \in \mathfrak{sim}(3) \)
  - Corresponding covariance matrix \( \BG{\Sigma}_{ji} \)

##### 1.1.2.4 Tracking: Direct \( \mathfrak{se}(3) \) Image Alignment

- Minimize the variance-normalized photometric error
  \[
     E_p(\BG{\xi}_{ji}) 
     = \sum_{p \in \Omega_{D_i}} 
       \| \frac {r_p^2(\V{p}, \BG{\xi}_{ji})} 
                {\sigma_{r_p(\V{p}, \BG{\xi}_{ji})}^2}
       \|_\delta
  \]
  where
  \[
     r_p(\V{p}, \BG{\xi}_{ji}) 
     = I_i(\V{p}) - I_j(\omega(\V{p}, D_i(\V{p}), \BG{\xi}_{ji}))
  \]
  being the photometric error/residual with \( \omega(\cdot) \) as the mapping of
  pixel \( \V{p} \) in reference image \( I_i \) to the current image \( I_j \),
  given the relative pose \( \BG{\xi}_{ji} \in \mathfrak{se}(3) \), and
  \[
     \omega_{r_p(\V{p}, \BG{\xi}_{ji})}^2
     = 2\omega_I^2 
       + (\frac {\partial r_p(\V{p}, \BG{\xi}_{ji})} 
                {\partial D_i(\V{p})})^2 V_i(\V{p})
  \]
  being the variance of the residual term taking into account image noise 
  \( \omega_I^2 \) (assumed to bew Gaussian) and varying noise on 
  depth estimates, and \( \| \cdot \|_\delta \) being the Huber norm

##### 1.1.2.5 Depth Map Estimation

- Depth map creation for new keyframe
  - Project points from the previous keyframe to the new one
  - Perform spatial regularization and outlier removal as described in 
    [@Engel2013VO]
  - Scale the depth map to have a mean inverse depth of one
    - The scaling factor is incorporated into the \( \mathfrak{sim}(3) \) pose
  - Replace the previous keyframe and use it for tracking subsequent new frames
- Depth map refinement [@Engel2013VO]
  - Filter over many per-pixel, small baseline stereo comparisons coupled with 
    interleaved spatial regularization
    - For each pixel, find a previous reference frame as *old* as possible
    - Given known relative pose and the pixel, compute the epipolar line on 
      the reference frame
    - Stereo matching: perform an exhaustive search alone the epipolar line
      using *SSD* error (over 5 equidistant points on the line)
    - Identify 3 major factors which determine the accuracy of such a stereo 
      observation, and determine which pixel is worth updating the depth 
      estimation

##### 1.1.2.6 Constraint Acquisition: Direct \( \mathfrak{sim}(3) \) Image Alignment

- Perform *direct, scale-drift aware image alignment* on \( \mathfrak{sim}(3) \)
  to align 2 differently scaled keyframes
  - Add a depth redisual \( r_d \) to the error function defined for tracking
  \[
     E(\BG{\xi}_{ji}) 
     = \sum_{p \in \Omega_{D_i}} 
       \| \frac {r_p^2(\V{p}, \BG{\xi}_{ji})} 
                {\sigma_{r_p(\V{p}, \BG{\xi}_{ji})}^2}
          + \frac {r_d^2(\V{p}, \BG{\xi}_{ji})} 
                  {\sigma_{r_d(\V{p}, \BG{\xi}_{ji})}^2}
       \|_\delta
  \]
  where
  \[ r_d(\V{p}, \BG{\xi}_{ji}) = [\V{p}']_3 - D_j([\V{p}']_{1,2}) \]
  with \( [\cdot]_k \) meaning the \( k \)th row of a matrix

- Constraint search (loop closure detection)
  - Use the closest 10 keyframes and a suitable candidate keyframe proposed
    by an appearance-based mapping algorithm [@Glover2011OpenFABMAP] 
    to detect loop closures
  - Perform a **reciprocal tracking check** for each candidate keyframe 
    \( \mathcal{K}_{j_k} \):
    - Independently track \( \BG{\xi}_{j_k i} \) and \( \BG{\xi}_{i j_k} \)
    - Only if the 2 estimates are statistically similar are they added to the
      global map
      
- Improve convergence radius (ability to track large motion) for 
  \( \mathfrak{sim}(3) \) tracking (direct \( \mathfrak{sim}(3) \) image 
  alignment)
  - Use a small number of keypoints to compute a better initialization 
  - Use **Efficient Second Order Minimization (ESM)** [@Benhimane2004ESM]
  - Use coarse-to-fine search (image pyramids) starting at a very low 
    resolution (such as \( 20 \times 15 \))

##### 1.1.2.7 Map Optimization

- Map: keyframes + tracked \( \mathfrak{sim}(3) \) constraints
- Continuously optimize the map in the background using pose graph optimization
  (implemented using *g2o*)
  - Vertices: pose \( \BG{\xi}_{Wi} \in \mathfrak{sim}(3) \) in the world 
    coordinate frame for keyframe \( \mathcal{K}_i \)
    - \( W \): world coordinate frame
  - Edges \( \mathcal{E} \): 
    relative pose \( \BG{\xi}_{ji} \)  from keyframe \( \mathcal{K}_i \)
    to \( \mathcal{K}_j \)
  - Error function:
    \[
       E(\BG{\xi}_{W1}, \dots, \BG{\xi}_{Wn})
       = \sum_{(\BG{\xi}_{ji}, \BG{\Sigma}_{ji}) \in \mathcal{E}}
         (\BG{\xi}_{ji} \circ \BG{\xi}_{Wi}^{-1} \circ \BG{\xi}_{Wj})^T 
         \BG{\Sigma}_{ji}^{-1} 
         (\BG{\xi}_{ji} \circ \BG{\xi}_{Wi}^{-1} \circ \BG{\xi}_{Wj})
    \]
    
      
### 1.2 Feature-based Systems


### 1.3 Systems in Kalman Filter Framework

#### 1.3.1 A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation [@Mourikis2007MSCKF]

- [Paper](https://ieeexplore.ieee.org/document/4209642)
- Related materials:
  - [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508) [@Sola2017Quaternion]: 
    include related concepts/formulations on quaternions/rotational matrices,
    and their applications in Kalman filter framework
  - [Why and How to Avoid the Flipped Quaternion Multiplication](https://arxiv.org/abs/1801.07478):
    details on different conventions for quaternions

##### 1.3.1.1 Overview

- A visual inertial navigation system (VINS), **MSCKF** in short
  - Camera + inertial measurement unit (IMU)

- Features
  - An EKF-based algorithm in real time for VIN
  - Main contribution:
    - Proposed a **measurement model** for the expression of geometric 
      constraints when observing a landmark from multiple camera poses
      - The geometric constraints involve all related camera poses that observe
        the landmark
    - The measurement model **does not require including 3D feature position** 
      in the state vector of EKF, resulting in computational complexity only
      **linear** in the number of features
    - The measurement model is **optimal** and **up to linearization errors**

- Terms
  - \( \left\{ I \right\} \): IMU-affixed frame
  - \( \left\{ G \right\} \): global frame
    - Chosen as *Earth-Centered, Earth-Fixed* (ECEF) frame:
      - \( (X, Y, Z) \) coordinates
      - \( (0, 0, 0) \) is defined as the center mass of Earth
      - Z-axis: extends through true north
      - X-axis: intersects the sphere of the earth at \( 0^{\circ} \) latitude
        (the equator) and \( 0^{\circ} \) longitude (prime meridian in 
        Greenwich)
      - Right-handed coordinate system

- Proposed algorithm  
  ![MSCKF](images/ch06/mourikis2007_alg_01_msckf.jpg){ width=50% }
  
- EKF state vector at any time instant
  - Evolving IMU state \( \V{X}_{IMU} \)
  - History of up to \( N_{max} \) past camera poses

##### 1.3.1.2 Structure of EKF State Vector

- Some references:
  - Error-state Kalman Filter (ESKF): chapter 5 in [@Sola2017Quaternion]

- Evolving IMU state
  \[ 
     \V{X}_{IMU} = \begin{bmatrix}
       {}_{G}^{I}\V{q}^T &
       \V{b}_g^T &
       {}^{G}\V{v}_I^T &
       \V{b}_a^T &
       {}^{G}\V{p}_I^T
     \end{bmatrix}^T
  \]
  - \( {}_{G}^{I}\V{q}^T \): unit **q**uaternion of rotation from
    global frame \( \left\{ G \right\} \) to IMU frame \( \left\{ I \right\} \)
    - ***WARNING***: this paper uses the **JPL convention** 
      (\( ij = -ji = -k \)) for quaternions instead of the
      ***Hamilton convention*** (\( ij = -ji = k \))
  - \( {}^{G}\V{v}_I^T \) & \( {}^{G}\V{p}_I^T \): 
    **v**elocity and **p**osition of IMU w.r.t. \( \left\{ G \right\} \)
  - \( \V{b}_g^T \) & \( \V{b}_a^T \): **b**iases which affect the 
    **g**yroscope and **a**ccelerometer measurements
    - They are modeled as random walk processes, driven by white Gaussian noise
      vectors \( \V{n}_{wg} \) and \( \V{n}_{wa} \)
- IMU error-state
  \[
     \tilde{\V{X}}_{IMU} = \begin{bmatrix}
       \delta\BG{\theta}_I^T &
       \tilde{\V{b}}_g^T &
       {}^{G}\tilde{\V{v}}_I^T &
       \tilde{\V{b}}_g^T &
       {}^{G}\tilde{\V{p}}_I^T
     \end{bmatrix}^T
  \]
  - Error definitions:
    - Standard additive errors for *position*, *velocity*, and *biases*
      - \( \tilde{x} = x - \hat{x} \) (error = real - estimate)
    - Error for quaternion \( \V{q} \): *error quaternion* 
      \( \delta\V{q} \)
      - \( \V{q} = \delta\V{q} \otimes \hat{\V{q}} \) where
        \( \otimes \) denotes quaternion multiplication

- EKF state vector at time-step \( k \) with \( N \) camera poses included:
  \[
     \hat{\V{X}}_k = \begin{bmatrix}
       \hat{\V{X}}_{{IMU}_k}^T &
       {}_G^{C_1}\hat{\V{q}}^T &
       {}^G\hat{\V{p}}_{C_1}^T &
       \dots &
       {}_G^{C_N}\hat{\V{q}}^T &
       {}^G\hat{\V{p}}_{C_N}^T
     \end{bmatrix}^T
  \]
  - \( {}_G^{C_i}\hat{\V{q}}^T \) & \( {}^G\hat{\V{p}}_{C_i}^T \):
    \( i \)th estimate of the camera attitude/pose and position

- EKF error-state vector at time-step \( k \) with \( N \) camera poses 
  included:
  \[
     \tilde{\V{X}}_k = \begin{bmatrix}
       \tilde{\V{X}}_{{IMU}_k}^T &
       \BG{\delta\theta}_{C_1}^T &
       {}^G\tilde{\V{p}}_{C_1}^T &
       \dots &
       \BG{\delta\theta}_{C_N}^T &
       {}^G\tilde{\V{p}}_{C_N}^T
     \end{bmatrix}^T
  \]
  
##### 1.3.1.3 Propagation

###### 1.3.1.3.1 Continuous-time System Modeling

- Time evolution of IMU state (derivative w.r.t. time / increment over period 
  \( dt \)):
  \[
     \begin{bmatrix}
       {}_G^I\V{\dot{q}}(t) \\
       \V{\dot{b}}_g(t) \\
       {}^G\V{\dot{v}}_I(t) \\
       \V{\dot{b}}_a(t) \\ 
       {}^G\V{\dot{p}}_I(t)
     \end{bmatrix}
     = \begin{bmatrix}
         \frac{1}{2} \BG{\Omega}(\BG{\omega}(t)) {}_G^I\V{q}(t) \\
         \V{n}_{wg}(t) \\
         {}^G\V{a}(t) \\
         \V{n}_{wa}(t) \\
         {}^G\V{v}_I(t)
       \end{bmatrix}
     \tag{MSCKF-6} \label{eq-MSCKF-6}
  \]
  where
  \[
     \BG{\omega} 
     = \begin{bmatrix} \omega_x & \omega_y & \omega_z \end{bmatrix}^T, \quad
     \BG{\Omega}(\BG{\omega})
     = \begin{bmatrix}
         -[\BG{\omega}]_{\times} & \BG{\omega} \\
         -\BG{\omega}^T & 0
       \end{bmatrix}
  \]
  - Ordinary differential equation (ODE) for \( {}_G^I\dot{\V{q}}(t) \):
    \begin{align*}
      {}_G^I\dot{\V{q}}(t) 
      &= \frac{1}{2} {}_G^I\V{q}(t) 
         \otimes \begin{bmatrix} \BG{\omega}^T & 0 \end{bmatrix}^T \\
      &= \frac{1}{2} (0 \cdot \V{I}_{4 \times 4} + \BG{\Omega}(\BG{\omega}))
         {}_G^I\V{q}(t) \\
      &= \frac{1}{2} \BG{\Omega}(\BG{\omega}) {}_G^I\V{q}(t)
    \end{align*}

- Gyroscope and accelerometer measurement \( \BG{\omega}_m \) and 
  \( \V{a}_m \) incorporating the effects of the planet's rotation 
  \( \BG{\omega}_G \):
  \[
     \BG{\omega}_m 
     = \BG{\omega} + \V{C}({}_G^I\V{q}) \BG{\omega}_G + \V{b}_g + \V{n}_g
  \]
  \[
     \V{a}_m
     = \V{C}({}_G^I\V{q})(
       {}^G\V{a} - {}^G\V{g} + 2 [\BG{\omega}_G]_{\times} {}^G\V{v}_I
       + [\BG{\omega}_G]_{\times}^2 {}^G\V{p}_I) + \V{b}_a + \V{n}_a
  \]
  where \( \V{C}(\cdot) \) converts an input into a rotational matrix
  
- *Estimates* of the evolving IMU state by applying expectation operator
  to the equation of time evolution of IMU state in \eqref{eq-MSCKF-6}
  \[
     \begin{bmatrix}
       {}_G^I\V{\dot{\hat{q}}} \\
       \V{\dot{\hat{b}}}_g \\
       {}^G\V{\dot{\hat{v}}}_I \\
       \V{\dot{\hat{b}}}_a \\ 
       {}^G\V{\dot{\hat{p}}}_I
     \end{bmatrix}
     = \begin{bmatrix}
         \frac{1}{2} \BG{\Omega}(\BG{\hat{\omega}}) {}_G^I\V{\hat{q}} \\
         \V{0}_{3 \times 1} \\
         \V{C}_{\V{\hat{q}}}^T \V{\hat{a}}
         - 2 [\BG{\omega}]_{\times} {}^G\V{\hat{v}}_I 
         - [\BG{\omega}]_{\times}^2 {}^G \V{\hat{p}}_I + {}^G \V{g} \\
         \V{0}_{3 \times 1} \\
         {}^G\V{\hat{v}}_I
       \end{bmatrix}
     \tag{MSCKF-9} \label{eq-MSCKF-9}
  \]
  where 
  \[ 
     \V{C}_{\V{\hat{q}}} = \V{C}({}_G^I\V{\hat{q}}), \quad
     \BG{\hat{\omega}} = \BG{\omega}_m - \V{\hat{b}}_g 
                         - \V{C}_{\V{\hat{q}}} \BG{\omega}_G, \quad
     \V{\hat{a}} = \V{a}_m - \V{\hat{b}}_a
  \]

- *Linearized* continuous-time model for IMU error-state:
  \[
     \V{\dot{\tilde{X}}}_{IMU} = \V{F} \V{\tilde{X}}_{IMU} + \V{G} \V{n}_{IMU}
  \]
  where 
  \( \V{n}_{IMU} = \begin{bmatrix} 
                     \V{n}_g^T &
                     \V{n}_{wg}^T & 
                     \V{n}_a^T &
                     \V{n}_{wa}^T
                   \end{bmatrix}_{12 \times 1}^T 
  \) is the system noise, and 
  \[
     \V{F} 
     = \begin{bmatrix}
         -[\hat{\BG{\omega}}]_\times & -\V{I}_{3 \times 3} & 
         \V{0} & \V{0} & \V{0} \\
         \V{0} & \V{0} & \V{0} & \V{0} & \V{0} \\
         -\V{C}_{\hat{\V{q}}}^T [\hat{\V{a}}]_\times & \V{0} & 
         -2[\BG{\omega}_G]_\times & -\V{C}_{\hat{\V{q}}}^T & 
         -[\BG{\omega}_G]_\times^2 \\
         \V{0} & \V{0} & \V{0} & \V{0} & \V{0} \\
         \V{0} & \V{0} & \V{I}_{3 \times 3} & \V{0} & \V{0}
       \end{bmatrix}_{15 \times 15}
  \] and 
  \[
     \V{G}
     = \begin{bmatrix}
         -\V{I}_{3 \times 3} & \V{0} & \V{0} & \V{0} \\
         \V{0} & \V{I}_{3 \times 3} & \V{0} & \V{0} \\
         \V{0} & \V{0} & -\V{C}_{\hat{q}}^T & \V{0} \\
         \V{0} & \V{0} & \V{0} & \V{I}_{3 \times 3} \\
         \V{0} & \V{0} & \V{0} & \V{0}
       \end{bmatrix}_{15 \times 12}
  \]

###### 1.3.1.3.2 Discrete-time Implementation

- Time-step: \( T \)
- Method for the propagation of the IMU state / EKF covariance matrix 
  (i.e., to solve the ODE using numerical integration): 
  5th order Runge-Kutta method
  - Propagation: can be seen as the value of a function \( f(t) \) at time 
    \( t_{base} + \delta t \) based on its value at time \( t_{base} \)
    after one/multiple time-step(s) \( \delta t \) elapsed
    - In this paper, propagation is to compute \( f(t_k + T) \) given
      \( f(t_k) \) and time-step \( T \)
- Propagation of IMU state (**not** error-state): 
  numerical integration on the derivatives in \eqref{eq-MSCKF-9}
- Propagation of EKF covariance matrix
  - EKF covariance matrix \( \V{P}_{k|k} \)
    \[
       \V{P}_{k|k}
       = \begin{bmatrix}
           \V{P}_{II_{k|k}} & \V{P}_{IC_{k|k}} \\
           \V{P}_{IC_{k|k}}^T & \V{P}_{CC_{k|k}}
         \end{bmatrix}
    \]
  - where
    - \( \V{P}_{II_{k|k}} \) is the \( 15 \times 15 \) covariance matrix
      of the evolving IMU error-state
    - \( \V{P}_{CC_{k|k}} \) is the \( 6N \times 6N \) covariance matrix
      of the camera pose estimates
    - \( \V{P}_{IC_{k|k}} \) is the correlation between IMU error-state
      and camera pose estimates


## 2. New Papers (2019)

### 2.1 IEEE International Conference on Robotics and Automation 2019 (ICRA2019)

#### 2.1.1 Loosely-Coupled Semi-Direct Monocular SLAM [@SHLee2019]

- [Paper](https://ieeexplore.ieee.org/abstract/document/8584894)

System diagram:  
![System Diagram](images/ch06/shlee2019_fig_02.jpg){ width=100% }

- Built upon direct sparse odometry (DSO) & ORB-SLAM

Features:

- Combine the complementary strength of direct and feature-based methods
- 2 modules running in parallel: 
  1) Direct odometry module: uses direct method to track new frames based on a 
     local semi-dense map
  2) Feature-based module: builds a sparse feature-based map based on tracked 
     map points and poses
- 3 levels of parallel optimizations
  1) Photometric bundle adjustment (BA): 
     jointly optimizes local structure and motion
  2) Geometric BA: refines keyframe poses and associated feature map points
  3) Pose graph optimization: achieve global map consistency in case of
     loop closures
- Achieve real-time by limiting feature-based operations to marginalized 
  keyframes from the direct odometry module

#### 2.1.2 Sparse2Dense: From Direct Sparse Odometry to Dense 3-D Reconstruction [@Tang2019]

- [Paper](https://ieeexplore.ieee.org/document/8605349)

System diagram:  
![System Diagram](images/ch06/tang2019_fig_02.jpg){ width=100% }

- 4 major stages:
  1) Depth/normal generation using CNN
  2) Visual tracking using direct alignment (direct sparse odometry, DSO)
  3) Geometrical sparse to dense reconstruction
  4) Fusion-based mapping
- Contribution on stage 1 & 3

Features:

- A *deep* dense monocular SLAM system
- Construct a dense 3D model via a sparse to dense mapping using 
  learned surface normals  
  ![Sparse2Dense](images/ch06/tang2019_fig_03.jpg){ width=100% }
- Depth estimations are learned in single view as prior for monocular 
  visual odometry (VO)
- VO with the prior above can obtain both accurate positioning and high-quality
  depth reconstruction
- Depth and normals are predicted by a single convoluted neural network (CNN) 
  trained in a tightly coupled manner
  - Depth: used in the projective geometric optimization for accurate 
    pose estimation
  - Normal: used for a dense geometrical reconstruction with intermediate
    sparse point clouds
  - CNN structure  
    ![FCRN vs. FCDRN](images/ch06/tang2019_fig_04.jpg){ width=50% }
    - Left: original *fully convolutional residual network* (FCRN) 
      with an encoder-decoder structure based on *ResNet-50*
    - right: FCDRN with *Resnet-50* replaced by the *Dilated Residual Network* 
      (DRN)

#### 2.1.3 Tightly-Coupled Visual-Inertial Localization and 3-D Rigid-Body Target Tracking [@Eckenhoff2019]

- [Paper](https://ieeexplore.ieee.org/document/8629939)

Features:

- Tightly-coupled estimator for visual-inertial localization and target tracking
  (VILTT)
  - Buiit upon visual-inertial navigation system (VINS) which is based on 
    multi-state constraint Kalman filter (MSCKF) framework
  - Generalize the VINS to incorporate a 6DOF rigid-body target tracking
    of a 3D moving object
- Key contribution: tightly-coupling of target motion estimation within VINS
- Represent the target object as a 3D rigid-body model rather than a point with
  the following data:
  - 6 DOF target pose
  - Features attached
- Introduced 3 stochastic target motion models for a broad range of realistic
  target tracking scenarios

#### 2.1.4 Dense-ArthroSLAM: Dense Intra-Articular 3-D Reconstruction With Robust Localization Prior for Arthroscopy [@Marmol2019]

- [Paper](https://ieeexplore.ieee.org/document/8606964)

Features:

- Dense-ArthroSLAM: a SLAM system based on *ArthroSLAM* for arthroscopy 

- *ArthroSLAM*
  - A *sparse* SLAM system
  - 2 visual sensors are attached to the final link of a robotic arm
    - Arthroscopic camera for intra-articular tissues
    - External camera for the exterior of the patient anatomy
  - Raw camera measurements are fused with robot's odometry in an extended
    Kalman filter (EKF) framework
  - Role: as a robust localization prior for reliable camera pose information

- Improvement over original *ArthroSLAM*
  - Added a robust localizer with a keyframe selection strategy
    - A mix of 2 strategies:
      1) The amount of translational and rotational variation
      2) Certain number of frames
  - Added a batch multiview stereo (MVS) for 3D reconstruction
    - Re-detect SIFT features in the keyframes and do brute-force matching
    - Triangulation: 2 random keyframes having at least 5 degrees between 
      viewing rays
    - 3D points and camera poses are refined using BA for every keyframe added
      to the optimization
    - Outlier removal scheme
    - Use a patch-match for per-pixel intensity similarity, and jointly
      optimize depth and normal of point clouds
    - Valid pixel observations are fused into a dense scene reconstruction of
      the tissues
      - Resulting point cloud with associated normals can be used to infer a
        surface mesh and approximate a reconstructed area

- Outperform state-of-the-art SLAM methods (*ORB-SLAM* for feature-based,
  and *LSD-SLAM* for direct-based) both in reliability and estimation accuracy

System diagram:  
![System Diagram](images/ch06/marmol2019_fig_01.jpg){ width=50% }

- Art.: Arthroscopic
- Ext.: External
- New components are in red

#### 2.1.5 Modeling Perceptual Aliasing in SLAM via Discrete–Continuous Graphical Models [@Lajoie2019DCGM]

- [Paper](https://ieeexplore.ieee.org/document/8624393)

Features:

- A unified framework to model perceptual aliasing (wrong data association) in 
  SLAM 
  - Discrete-continuous graphical model (*DC-GM*)  
    ![DC-GM](images/ch06/lajoie2019_fig_01_dcgm.jpg){ width=60% }
    - Continuous portion: describes the standard SLAM formulation, i.e., 
      a pose graph
    - Discrete portion: describes outlier selection and models correlation
      - A Markov random field
- Practical algorithms to cope with outliers without relying on initial guess
  for optimization
  - Semidefinite relaxation to perform inference in *DC-GM* and returns 
    estimates with provable sub-optimality guarantees
    
Limitations:

- MATLAB implementation: slow, and only tested for relatively small 
  2D SLAM problems
  
#### 2.1.6 FMD Stereo SLAM: Fusing MVG and Direct Formulation Towards Accurate and Fast Stereo SLAM [@Tang2019FMD]

- [Paper](https://ieeexplore.ieee.org/document/8793664)

System diagram:  
![System Diagram](images/ch06/tang2019_fig_01_fmd.jpg){ width=80% }

Features:

- Stereo visual SLAM system
- **F**usion of key-feature-based **M**ultiple view geometry (MVG) and 
  **D**irect formulation (FMD)
  - Make full use of advantages from *SVO* and *ORB-SLAM*
- Front-end: make the system faster
  - Use *direct formulation* and *constant motion model* to predict an 
    initial pose
  - Reproject local map for 3D-to-2D correspondences
  - Refine the pose by *reprojection error minimization*
- Back-end: make the system more accurate
  - Use MVG to estimate 3D structure
    - Perform triangulation for new map points when a new keyframe is inserted
    - Remove outliers in the map and maintain a global map by BA

#### 2.1.7 GEN-SLAM: Generative Modeling for Monocular Simultaneous Localization and Mapping [@Chakravarty2019GENSLAM]

- [Paper](https://ieeexplore.ieee.org/document/8793530)

System diagram:  
![System Diagram](images/ch06/chakravarty2019_fig_04_gen_slam.jpg){ width=70% }

Features:

- Deep learning based visual SLAM system
- Use a *CNN* to localize in a topological map
- Generative modeling: encoder-decoder architecture with shared latent space
  for *variational auto encoder* (VAE)
  - 2 encoders and 2 decoders, and one each for RGB and depth respectively
  - Use a *conditional variational auto encoder* (VAE) for depth map,
    conditioned on the CNN-estimated pose
  - Employ 4 reconstruction losses to train the network
    - RGB -> RGB
    - Depth -> Depth
    - RGB -> Depth
    - Depth -> RGB
- RGB VAE structure:  
  ![RGB VAE](images/ch06/chakravarty2019_fig_02_rgb_vae.jpg){ width=75% }
  
#### 2.1.8 RESLAM: A Real-time Robust Edge-based SLAM System [@Schenk2019RESLAM]

- [Paper](https://ieeexplore.ieee.org/document/8794462)

System diagram:  
![System Diagram](images/ch06/schenk2019_fig_02_reslam.jpg){ width=70% }

- 3 components
  - Visual odometry module: for relative camera motion estimation
  - Local mapper: manages keyframes and optimizes a local window
  - Global mapper: stores a global map and performs loop closure and 
    relocalization

Features:

- Edge-based SLAM system for RGBD sensors
  - Canny edges are detected for each frame
- Utilize edges in all the subsequent steps in the proposed SLAM pipeline:
  - Camera pose estimation
  - Local sliding window optimization for depth refinement
  - Loop closure
  - Relocalization
- Initial depth refinement in a sliding window
- Edge-based loop closure verification
- Fast system that runs in real-time on a CPU
- Open-source

#### 2.1.9 Fast and Robust Initialization for Visual-Inertial SLAM [@Campos2019FastInitVISLAM]

- [Paper](https://ieeexplore.ieee.org/document/8793718)

Features:

- A fast method for joint initialization of monocular-inertial SLAM
  built on the method proposed by [@Martinelli2014ClosedVISfM] and 
  extended by [@Kaiser2017InitVIN] (the original method in the paper)
- The original method is improved as follows:
  - More general: allows incomplete feature tracks (features not seen by 
    all cameras)
  - More computationally efficient: uses the IMU preintegration method of 
    [@Forster2015IMUPreint]
  - Improved accuracy: uses 2 visual-inertial BA steps
  - Improved robustness: uses novel observability and consensus tests to 
    detect bad initializations
- Scale errors decreases from up to 156% of the original method (215ms) to 
  around 5% (1-2 seconds), or to less than 1% after performing visual-inertial
  BA after 10 seconds
  
#### 2.1.10 Visual SLAM: Why Bundle Adjust? [@Bustos2019WhyBA]

- [Paper](https://ieeexplore.ieee.org/document/8793749)

Algorithm for the proposed system:  
![L-Infinity SLAM](images/ch06/bustos2019_alg_02_linf.jpg){ width=60% }

Features:

- A SLAM system called L-infinity SLAM that is not based on BA
- Solution to problems of BA-based SLAM systems
  - Problems:
    - The need to maintain an accurate map and camera motions at keyframe rate
    - Cannot deal with slow motion and pure rotational motion because of
      the requirement on sufficient baselines for BA
  - Solution:
    - Conduct rotation averaging to optimize *camera orientations only*
      instead of BA
    - Given the orientations, camera positions and 3D points are estimated
      via a quasi-convex optimization that can be *globally optimally* solved 
- Efficient loop closure module
  - Incorporation of relative translation directions to solve camera drifts
- Advantage of L-Infinity SLAM
  - Many tasks of map maintenance (feature/map point selection, point culling,
    map updating and aggregation) can be done in a low priority thread or even
    offline
  - The system can handle pure rotation motions

#### 2.1.11 Illumination Robust Monocular Direct Visual Odometry for Outdoor Environment Mapping [@Wu2019MonoVO]

- [Paper](https://ieeexplore.ieee.org/document/8793607)

Features:

- An illumination-robust direct monocular VO system focusing on modeling 
  outdoor scenery
- Evaluation on state-of-the-art illumination invariant cost functions
  in the context of monocular joint optimization framework
- An illumination-robust cost is proposed by combining intensity- and 
  gradient-based costs with an adaptive weight
- Illustration that sun glares can be modeled as local illumination changes,
  and its adverse effect on motion estimation can be alleviated by dealing
  with these local changes
- Computational cost of the proposed system is higher than other methods

### 2.2 Robotics: Science and Systems 2019 (RSS2019)

#### 2.2.1 Continuous Direct Sparse Visual Odometry from RGB-D Images [@Ghaffari2019RGBDVO]

- [Paper](http://www.roboticsproceedings.org/rss15/p44.html)

Features:

- A direct and continuous energy-based formulation and solution for the
  RGB-D visual odometry problem (sensor registration problem)
  - Direct: relates to direct-based method
  - Continuous: the formulation of the image registration problem, given
    discrete cloud measurements, is in a continuous form (based on continuous
    camera motion)
    - Operates from 3D space to an abstract information space such as 
      intensity surface
    - Continuity is achieved via functional treatment of the problem,
      and representing the functions in a reproducing kernel Hilbert space
- Do not rely on the association between 2 measurements and the same number
  of measurements within each set because of the continuous formulation
- The developed framework is not limited to specific image resolution
- The framework is fully analytical and the gradient has a complete closed-form
  derivation
  - It is opposed to the current direct energy formulation:
    a) Computation of numerical image intensity gradient
    b) The use of the gradient with the Jacobian of the pose via the chain rule
- The proposed method can be an alternative to the core module of many modern
  visual odometry and tracking systems

#### 2.2.2 PoseRBPF: A Rao-Blackwellized Particle Filter for 6D Object Pose Estimation [@Deng2019PoseRBPF]

- [Paper](http://www.roboticsproceedings.org/rss15/p49.html)

System diagram:  
![System Diagram](images/ch06/deng2019_fig_05_diagram.jpg){ width=100% }

- Propagation: estimate current pose based on previous pose using motion prior

Rotation computation:  
![Rotation Computation](images/ch06/deng2019_fig_03_rot.jpg){ width=100% }

Features:

- PoseRBPF: 6D object pose tracking based on Rao-Blackwellized particle 
  filter (RBPF) framework
- Pose is factorized into translation and rotation parts
  - Rotation
    - Rotation space is discretized at 5 degree resolution
      (\( 72 \times 37 \times 72 = 191808 \) bins for elevation only ranges
      from -90 to 90 degree)
    - An auto-encoder network is trained as an observation model to construct
      a codebook of feature embeddings for the discretized rotations
- Procedures: for each particle,
  1) Determine the center and the size of the object bounding box in the 
     image via 3D translation data
  2) Determine embedding of the bounding box
  3) Update rotation distribution by comparing the embedding value with the
     pre-computed entries in the codebook using cosine distance
  4) Compute the weight of the particle from the normalization factor of the
     rotation distribution
  5) Update motion by sampling from a motion model over poses and a 
     convolution over rotations
     
#### 2.2.3 VIMO: Simultaneous Visual Inertial Model-Based Odometry and Force Estimation [@Nisar2019VIMO]

Features:

- VIMO: visual inertial model-based odometry with an added motion constraint
  - Motion constraint is derived from the dynamic model including 
    *external forces*
  - Model dynamics and external force are combined in a preintegrated residual  
    ![Factor Graph](images/ch06/nisar2019_fig_01_graph.jpg){ width=60% }
    - The residual makes current visual inertial odometry (VIO) framework 
      jointly estimate this force in addition to the robot state and IMU bias
  - Built upon a state-of-the-art VIO system the VINS-mono system
  - Tightly-coupled, sliding-window visual inertial estimator
  - Input: visual-inertial measurements & the collective thrust
  - Estimation strategy: nonlinear optimization (BA)
- (1st approach to) Exploit model dynamics by jointly estimating motion 
  and external force

\newpage
