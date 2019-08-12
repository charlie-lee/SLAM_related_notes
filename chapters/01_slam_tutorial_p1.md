# Notes on SLAM Tutorial Part I [@Durrant-Whyte2006] ([paper](http://everobotics.org/pdf/SLAMTutorial.pdf)) {#slam1}
- Problem: 
  - Whether a mobile robot placed in an unknown location of an unknown 
    environment can incrementally **_build a consistent map_** of this 
    environment while simultaneously **_determining its location_** 
    within this map
    
## 1. History
- As a mobile robot moves through an unknown environment taking relative 
  observations of landmarks, the estimates of these landmarks are all 
  necessarily correlated with each other because of the common error in 
  estimated vehicle location
  - Implication: consistent full solution of SLAM problem would require **_a
    joint state_** composed of the **_vehicle pose_** and **_every landmark
    position_**, to be updated following **_each landmark observation_**
    - Pose: combination of position & orientation
- Once SLAM is formulated as a single **_estimation problem_**, the estimated 
  map errors are **_actually convergent_**
- The more these correlations between landmarks grow, the better the solution


## 2. Formulation & Structure of the SLAM Problem
- SLAM: a process by which a mobile robot can **_build a map of an environment
  _** and at the same time **_use this map to deduce its location_**
- Both the **_trajectory of the platform_** and the **_location of all 
  landmarks_** can be estimated online without any **_a priori knowledge 
  of location_**
  
### 2.1 Preliminaries
- SLAM problem illustration:  
  ![SLAM problem](images/ch01/01_slam_problem.jpg){ width=60% }
- \( \V{x}_k \): 
  the state vector indicating vehicle location & orientation at time \( k \)
- \( \V{u}_k \): 
  the control vector, applied at time \( k-1 \) to drive the vehicle to 
  state vector \( \V{x}_k \) at time \( k \)
- \( \V{m}_i \): 
  a vector indicating the location of \( i \)th landmark whose true location 
  is assumed to be time invariant
- \( \V{z}_{ik} \): 
  an observation of \( i \)th landmark taken from the vehicle at time \( k \)
  - May be abbreviated as \( \V{z}_k \) indicating multiple observations 
    at time \( k \) or if the specific landmark is not revelant to the 
    discussion
- \( 
      \V{X}_{0:k} = \{\V{x}_0, \V{x}_1, \dots, \V{x}_k\} = 
      \{\V{X}_{0:k-1}, \V{x}_k\} 
  \): the history of previous vehicle locations
- \( 
      \V{U}_{0:k} = \{\V{u}_1, \V{u}_2, \dots, \V{u}_k\} = 
      \{\V{U}_{0:k-1}, \V{u}_k\} 
  \): the history of control inputs
- \( \V{m} = \{\V{m}_1, \V{m}_2, \dots, \V{m}_n\} \): the set of all landmarks
- \( 
      \V{Z}_{0:k} = \{\V{z}_1, \V{z}_2, \dots, \V{z}_k\} = 
      \{\V{Z}_{0:k-1}, \V{z}_k\}
  \): the set of all landmark observations

### 2.2 Probabilistic SLAM
- SLAM problem requires the computation of 
  \( P(\V{x}_k, \V{m} | \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0) \) 
  at all times \( k \)
  - Joint posterior density of the _vehicle state_ and the 
    _landmark locations_ (at time \( k \))
  - Recursive computation: compute joint posterior density at time 
    \( k \) by 
    \( P(\V{x}_{k-1}, \V{m} | \V{Z}_{0:k-1}, \V{U}_{0:k-1}, \V{x}_0) \) 
    following control \( \V{u}_k \) 
    and observation \( \V{z}_k \)
  - Requirement: the definition of a **_state transition model_** and an 
    **_observation model_** describing the control input and observation 
    respectively
    - **_Observation model_**: \( P(\V{z}_k | \V{x}_k, \V{m}) \)
      - The probability of making an observation at time \( k \) given the 
        _vehicle location_ and _landmark locations_
      - The observations are **_conditionally independent_** of current vehicle 
        state (\( \V{x}_k \)) and the map (\( \V{m} \)) once the vehicle 
        location and map are defined
    - **_Motion model_**: \( P(\V{x}_k | \V{x}_{k-1}, \V{u}_k) \)
      - Probability distribution on state transitions
      - Assume state transition is a **_Markov process_**: 
        - The next state \( \V{x}_k \) depends only on the previous state
          \( \V{x}_{k-1} \) and the applied control \( \V{u}_k \)
        - State transition is independent of both the observations and the map
- SLAM Algorithm: two-step recursive (_sequential_) prediction 
  (_time-update_) correction (_measurement-update_) form
  - Step 1: Time-update
    \[ 
        P(\V{x}_k, \V{m} | \V{Z}_{0:k-1}, \V{U}_{0:k}, \V{x}_0) = 
        \int P(\V{x}_k | \V{x}_{k-1}, \V{u}_k) 
             P(\V{x}_{k-1}, \V{m} | \V{Z}_{0:k-1}, \V{U}_{0:k-1}, \V{x}_0) 
             \mathrm{d}\V{x}_{k-1} 
    \]
    - Notes:
      - Only vehicle location \( \V{x}_k \) is updated? Map data 
        \( \V{m} \) should be of time \( k-1 \)
      - Law of total probability: 
        \( P(A) = \displaystyle\sum_{n} P(A|B_n) P(B_n) \)
        - Event A: get prediction of current vehicle location & measurement
          of _?previous?_ landmark locations 
        - Event B: get measurement of previous vehicle location & 
          landmark locations
  - Step 2: Measurement-update
    \[ 
        P(\V{x}_k, \V{m} | \V{Z}_{0:k}, \V{U}_{0:k}, 
        \V{x}_0) = 
        \frac
        { 
          P(\V{z}_k | \V{x}_k, \V{m}) 
          P(\V{x}_k, \V{m} | \V{Z}_{0:k-1}, \V{U}_{0:k}, 
          \V{x}_0) 
        } 
        { 
          P(\V{z}_k | \V{Z}_{0:k-1}, \V{U}_{0:k}) 
        }
    \]
    - Notes:
      - Bayes' theorem: \( P(A|B, C) = \frac {P(B|A, C) P(A|C)} {P(B|C)} \)
        - Event A: get current vehicle location & landmark locations
          - \( \V{x}_k, \V{m} \)
        - Event B: get current observation 
          - \( \V{z}_k \)
        - Event C: get historical observations & control inputs
          - \( \V{Z}_{0:k-1}, \V{U}_{0:k-1}, \V{x}_0 \)
      - Current observation is only related to current vehicle state and map
        - \( P(B|A, C)
             = P(\V{z}_k | (\V{x}_k, \V{m}), 
                           (\V{Z}_{0:k-1}, \V{U}_{0:k-1}, \V{x}_0))
             = P(\V{z}_k | \V{x}_k, \V{m}) = P(B|A) \)
    
### 2.3 Structure of Probabilistic SLAM
- Observations are **_dependent_** on both vehicle & landmark locations
  from the _observation model_ \( P(\V{z}_k | \V{x}_k, \V{m}) \), 
  therefore:
  \[
      P(\V{x}_k, \V{m} | \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0) \neq
      P(\V{x}_k | \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0)
      P(\V{m} |  \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0)
  \]
- Errors in landmark locations are highly correlated
  - Relative location between any 2 landmarks \( \V{m}_i \), 
    \( \V{m}_j \), may be **_known with high accuracy_**, even when 
    the absolute location of a landmark \( \V{m}_i \) is quite uncertain
    - Meaning in probabilistic form:
      joint probability density for the pair of landmarks
      \( P(\V{m}_i, \V{m}_j) \) is highly peaked even when the 
      marginal densities \( P(\V{m}_i) \) may be quite dispersed
- Correlations between landmark estimates increase monotonically as more and 
  more observations are made
  - Knowledge of relative landmark locations always improves and never 
    diverges, regardless of robot motion
    - \( P(\V{m}) \) becomes monotonically peaked as more observations 
      are made
  - All known relative landmark locations are updated at each time segment
    even though some of the landmarks are not seen by the vehicle at that time


## 3. Solutions to the SLAM Problem
- Solutions involve _finding an appropriate representation_ for both the 
  **_observation model_** and the **_motion model_**
  - Representation 1: a state-space model with additive Gaussian noise
    - Use **_extended Kalman filter_** (EKF) to solve the SLAM problem 
      (**_EKF-SLAM_**)
  - Representation 2: describe vehicle motion model 
    \( P(\V{x}_k | \V{x}_{k-1}, \V{u}_k) \) as a set of
    samples of a more general non-Gaussian probability distribution
    - Use **_Rao-Blackwellized particle filter_** or **_FastSLAM_** algorithm 
      to solve the SLAM problem
      
### 3.1 Prerequisites: Kalman Filter & EKF
- Tutorials on Kalman filter & extended Kalman filter:
  - Kalman Filter:
    [#1](http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies) |
    [**_#2_**](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
    & [extra](https://stats.stackexchange.com/questions/230596/why-do-the-probability-distributions-multiply-here) | 
    [#3 (Faragher 2012)](https://courses.engr.illinois.edu/ece420/sp2017/UnderstandingKalmanFilter.pdf) |
    [#4 (Wiki)](https://en.wikipedia.org/wiki/Kalman_filter)
  - Extend Kalman Filter: 
    [#1](https://home.wlu.edu/~levys/kalman_tutorial/) |
    [#2](https://towardsdatascience.com/extended-kalman-filter-43e52b16757d) |
    [#3 (Wiki)](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
    
#### 3.1.1 Kalman Filter
- Models  
  ![Kalman Filter](images/ch01/02_kalman_filter.jpg){ width=60% }
  - **_State model_** (\( P(\V{x}_k | \V{x}_{k-1}, \V{u}_k) \)):
    \[ \V{x}_k = \V{F}_k \V{x}_{k-1} + \V{B}_k \V{u}_k + \V{w}_k \]
    - \( \V{x}_k \): the state vector containing the parameters for 
      the system at time \( k \)
    - \( \V{F}_k \): state-transition model to transfer the state vector 
      \( \V{x}_{k-1} \) (such as the position & velocity of a robot) at 
      time \( k-1 \) to the next state \( \V{x}_k \) at time \( k \)
    - \( \V{B}_k \): control-input model - control matrix applying 
      on the control vector \( \V{u}_k \) (external force to lead 
      the robot to the next state) at time \( k \)
    - \( \V{w}_k \): process noise at time \( k \) which assumes to 
      have a multivariate Gaussian distribution with zero mean and 
      a covariance of \( \V{Q}_k \)
      - \( \V{w}_k \sim \mathcal{N}(0, \V{Q}_k) \)
  - **_Observation model_** (\( P(\V{z}_k | \V{x}_k) \)): 
    \[ \V{z}_k = \V{H}_k \V{x}_k + \V{v}_k \]
    - \( \V{z}_k \): observation/measurement at time \( k \) made by 
      true state \( \V{x}_k \) (not an estimation)
    - \( \V{H}_k \): observation model (transformation matrix) which 
      maps the state space/domain into the observation/measurement 
      space/domain
    - \( \V{v}_k \): observation noise which assumes to have a 
      multivariate Gaussian distribution of zero mean and a 
      covariance of \( \V{R}_k \)
      - \( \V{v}_k \sim \mathcal{N}(0, \V{R}_k) \)
- 2 steps: prediction & update
  - Step 1: Kalman filter produces estimates of the _current state 
    variables_, along with their uncertainties
    \[
        \V{\hat{x}}_{k|k-1} = \V{F}_k \V{\hat{x}}_{k-1|k-1} + 
                              \V{B}_k \V{u}_k
    \]
    \[
        \V{P}_{k|k-1} = \V{F}_k \V{P}_{k-1|k-1} \V{F}_k^T + \V{Q}_k
    \]
    - General notation \( \V{X}_{a|b} \): \( \V{X} \) at time \( a \)
      given observations up to and including at time \( b \leq a \)
    - State prediction:
      - \( \V{\hat{x}}_{k|k-1} \): the **_estimate_** of state vector 
        \( \V{x} \) at time \( k \) given observations up to and 
        including at time \( k-1 \)
    - Covariance prediction:
      - \( \V{P}_{k|k-1} \): covariance matrix at time \( k \) (describes
        the correlations between any two parameters inside the state 
        vector \( \V{x} \))
      - \( \V{Q}_k \): covariance matrix of the process noise 
        (additive Gaussian noise) at time \( k \)
      - Derivation (notice the uncorrelation between state estimation 
        errors and process noise):
        \begin{align*}
            \V{P}_{k|k-1} 
            &= \mathrm{E}[ (\V{x}_{k} - \V{\hat{x}}_{k|k-1})
                           (\V{x}_{k} - \V{\hat{x}}_{k|k-1})^T ] \\
            &= \mathrm{E}
               [ (\V{F}_k (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1}) + \V{w}_k)
                 (\V{F}_k (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1}) + 
                 \V{w}_k)^T ] \\
            &= \mathrm{E} [ (\V{F}_k (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1}) 
                            (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1})^T 
                            \V{F}_k^T) ] +
               \mathrm{E} [ \V{F}_k 
                            (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1}) 
                            \V{w}_k^T ] \\
            &\quad + \mathrm{E} [ \V{w}_k 
                                  (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1})^T 
                                  \V{F}_k^T ] +
                     \mathrm{E} [ \V{w}_k \V{w}_k^T ] \\
            &= \V{F}_k 
               \mathrm{E} [ (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1}) 
                            (\V{x}_{k-1} - \V{\hat{x}}_{k|k-1})^T ]
               \V{F}_k^T + 0 + 0 + \mathrm{E} [ \V{w}_k \V{w}_k^T ] \\
            &= \V{F}_k \V{P}_{k-1|k-1} \V{F}_k^T + \V{Q}_k
        \end{align*}
  - Step 2: estimates are updated using a _weighted average_ once the 
    outcome of the _next measurement_ is observed
    \[
        \V{\hat{x}}_{k|k} = \V{\hat{x}}_{k|k-1} + \V{K}_k \V{\tilde{y}}_k
    \]
    \[
        \V{P}_{k|k} = (\V{I} - \V{K}_k \V{H}_k) \V{P}_{k|k-1}
    \]
    where
    \[
        \V{K}_k = \V{P}_{k|k-1} \V{H}_k^T \V{S}_k^{-1}
    \]
    \[
        \V{\tilde{y}}_k = \V{z}_k - \V{H}_k \V{\hat{x}}_{k|k-1}
    \]
    \[
        \V{S}_k = \V{H}_k \V{P}_{k|k-1} \V{H}_k^T + \V{R}_k 
    \]
    - \( \V{\tilde{y}}_k \) & \( \V{S}_k \): innovation / measurement 
      residual and its covariance at time \( k \)
    - Both _estimated observation_ 
      (\( \V{z}_{k, \text{estimated}} = \V{H}_k \V{\hat{x}}_{k|k-1} \)) 
      and _actual observation_ (\( \V{z}_k \)) have uncertainties 
      (a range of possible observation data (assuming Gaussian 
      distributions))
    - \( 
          \V{z}_{k, \text{estimated}} \sim \mathcal{N} 
          ( \V{H}_k \V{\hat{x}}_{k|k-1}, 
            \V{H}_k \V{P}_{k|k-1} \V{H}_k^T ) =
          \mathcal{N} (\V{H}_k \BG{\mu}_0, \V{H}_k \BG{\Sigma}_0 \V{H}_k^T)
      \)
    - \( 
          \V{z}_{k, \text{actual}} \sim \mathcal{N} (\V{z}_k, \V{R}_k) =
          \mathcal{N} (\BG{\mu}_1, \BG{\Sigma}_1)
      \)
    - Choose the **_overlapped_** region of the 2 ranges as the best 
      guess of the real observation:
      - Also a Gaussian distribution (multiplying the 2 densities):
        \( \V{z}_{k, \text{fused}} \sim \mathcal{N} 
           (\BG{\mu}_{\text{fused}}, \BG{\Sigma}_{\text{fused}}) \)
      - \( \BG{\mu}_{\text{fused}} = \BG{\mu}_0 + \V{K}_k 
                                       (\BG{\mu}_1 - \V{H}_k \BG{\mu}_0) \)
      - \( \BG{\Sigma}_{\text{fused}} = \BG{\Sigma}_0 - 
                                         \V{K}_k \V{H}_k \BG{\Sigma}_0 \)
      - \( \V{K}_k = \BG{\Sigma}_0 \V{H}_k^T 
                     (\V{H}_k \BG{\Sigma}_0 \V{H}_k^T + \BG{\Sigma}_1)^{-1} \)
        (\( \V{K}_k \): **_Kalman gain_** at time \( k \))
        
#### 3.1.2 Extended Kalman Filter (EKF)
> Notes:  mathematical notations are almost the same with those in KF

- KF limitations: 
  - Linear function/model/system
  - Gaussian distribution for observed variables
- EKF: Nonlinear version of KF
  - Use **_estimated_** mean & covariance of observed variables for filter 
    response computation
  - Mean & covariance is estimated using linear approximations through 
    Taylor Series (multivariate Taylor Series)
    - Linear because only the first order (partial) derivatives are 
      computed (using the Jacobian matrix)
- Models: use **_differentiable_** functions (\( f \) & \( h \)) 
  instead of linear functions (i.e., matrices \( \V{F}_k \), 
  \( \V{B}_k \), and \( \V{H}_k \))
  - **_State model_**: 
    \[ \V{x}_k = f(\V{x}_{k-1}, \V{u}_k) + \V{w}_k \]
  - **_Observation model_**: 
    \[ \V{z}_k = h(\V{x}_k) + \V{v}_k \]
- 2 steps: prediction & update
  - Step 1: prediction
    \[
        \V{\hat{x}}_{k|k-1} = f( \V{\hat{x}}_{k-1|k-1}, \V{u}_k )
    \]
    \[
        \V{P}_{k|k-1} = \V{F}_k \V{P}_{k-1|k-1} \V{F}_k^T + \V{Q}_k
    \] 
    - \( \V{F}_k \): Jacobian matrix whose elements are the linear 
      approximations (partial derivatives) of state transition 
      function \( f \) defined as follows (assume state vector has
      a dimension of \( n \times 1 \))
      \[
          \V{F}_{k, n \times n} = \frac {\partial f_{n \times 1}} 
                                        {\partial \V{x}_{n \times 1}}
                                  ( \V{\hat{x}}_{k-1|k-1}, \V{u}_k )
      \]
      
  - Step 2: update
    \[
        \V{\hat{x}}_{k|k} = \V{\hat{x}}_{k|k-1} + \V{K}_k \V{\tilde{y}}_k
    \]
    \[
        \V{P}_{k|k} = (\V{I} - \V{K}_k \V{H}_k) \V{P}_{k|k-1}
    \]
    where
    \[
        \V{K}_k = \V{P}_{k|k-1} \V{H}_k^T \V{S}_k^{-1}
    \]
    \[
        \V{\tilde{y}}_k = \V{z}_k - h( \V{\hat{x}}_{k|k-1} )
    \]
    \[
        \V{S}_k = \V{H}_k \V{P}_{k|k-1} \V{H}_k^T + \V{R}_k
    \]
    - \( \V{H}_k \): Jacobian matrix whose elements are the linear
      approximations (partial derivatives) of measurement-to-state mapping 
      function \( h \) defined as follows (assume state vector has a dimension 
      of \( n \times 1 \) and observation vector has a dimension 
      of \( m \times 1 \))
      \[
          \V{H}_{k, m \times n} = \frac {\partial h_{m \times 1}} 
                                        {\partial \V{x}_{n \times 1}} 
                                  ( \V{\hat{x}}_{k|k-1} )
      \]

### 3.2 EKF-SLAM
- Models
  - Motion model: 
    \[ 
        P(\V{x}_k | \V{x}_{k-1}, \V{u}_k) \iff 
        \V{x}_k = f(\V{x}_{k-1}, \V{u}_k) + \V{w}_k
    \]
    - \( f(\cdot) \): models the vehicle kinematics
    - \( \V{w}_k \): _additive_, _uncorrelated_ Gaussian motion disturbances,
      and \( \V{w}_k \sim \mathcal{N}(0, \V{Q}_k) \)
  - Observation model:
    \[
        P(\V{z}_k | \V{x}_k, \V{m}) \iff
        \V{z}_k = h(\V{x}_k, \V{m}) + \V{v}_k
    \]
    - \( h(\cdot) \): models the geometry of the observation
    - \( \V{v}_k \): _additive_, _uncorrelated_ Gaussian observation errors,
      and \( \V{v}_k \sim \mathcal{N}(0, \V{R}_k) \)
- Applying EKF to SLAM problem by modeling joint posterior distribution
  \( P(\V{x}_k, \V{m} | \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0) \) as 
  \( \V{z}_{k, \text{fused}} \)
  \[ 
    \V{z}_{k, \text{fused}} \sim 
    \mathcal{N}(\BG{\mu}_{\text{fused}}, \BG{\Sigma}_{\text{fused}})
  \]
  \[
    \BG{\mu}_{\text{fused}} = 
     \begin{bmatrix}
      \V{\hat{x}}_{k|k} \\
      \V{\hat{m}}_k \\
     \end{bmatrix} = \mathrm{E} 
      \left[
       \begin{array} {c|}
        \V{x}_k \\ 
        \V{m} \\
       \end{array} \V{Z}_{0:k}
      \right] \\
  \]
  \[
    \BG{\Sigma}_{\text{fused}} = 
     \begin{bmatrix}
      \V{P}_{xx}   & \V{P}_{xm} \\
      \V{P}_{xm}^T & \V{P}_{mm} \\
     \end{bmatrix}_{k|k} = 
     \mathrm{E}
      \left[
       \begin{pmatrix}
        \V{x}_k - \V{\hat{x}}_{k|k} \\
        \V{m} - \V{\hat{m}}_k \\
       \end{pmatrix}
       \begin{pmatrix}
        \V{x}_k - \V{\hat{x}}_{k|k} \\
        \V{m} - \V{\hat{m}}_k \\
       \end{pmatrix}^T
      \right] = 
     \mathrm{E}
      \left[
       \begin{array} {c|}
        \begin{pmatrix}
         \V{x}_k - \V{\hat{x}}_k \\
         \V{m} - \V{\hat{m}}_k \\
        \end{pmatrix}
        \begin{pmatrix}
         \V{x}_k - \V{\hat{x}}_k \\
         \V{m} - \V{\hat{m}}_k \\
        \end{pmatrix}^T
       \end{array} \V{Z}_{0:k}
      \right]
  \]
  - EKF applying: **_time-update_** (prediction step) & 
    **_observation-update_** (update step)
    - Time-update is only for state vectors (except when landmarks can move)
    - Observation-update: 
      \( \V{\hat{x}}_{k|k} \to
         \begin{bmatrix} \V{\hat{x}}_{k|k} \\ \V{\hat{m}}_k \end{bmatrix} \)
    - \( \V{F}_k = \nabla{f} = 
         \frac {\partial f} {\partial \V{x}}
         ( \V{\hat{x}}_{k-1|k-1}, \V{u}_k ) \)
    - \( \V{H}_k = \nabla{h} = 
         \frac {\partial h} {\partial \V{x}}
         ( \V{\hat{x}}_{k|k-1}, \V{\hat{m}}_{k-1} ) \)
    - \( \V{W}_k \): Kalman gain (\( \V{K}_k \))
- Map convergence: map convergence matrix \( \V{P}_{mm, k} \) and 
  all landmark pair submatrices are monotonically convergent toward zero
- Computation grows **_quadratically_** with **_the number of landmarks_** 
  because all landmarks and joint covariance matrix will be updated 
  every time an observation is made
- Standard formulation of EKF-SLAM solution is fragile to incorrect association
  of observations to landmarks (**_loop-closure_** problem)
- Linearization of nonlinear motion & observation models (**_E_**KF) leads to 
  inconsistency in solutions

### 3.3 Rao-Blackwellized Filter (FastSLAM)
- Basis: recursive Monte Carlo sampling / particle filtering
- Applying Rao-Blackwellization (R-B) on sample-space (state-space)
  - Partition joint state according to the product rule: 
    \( P(\V{x}_1, \V{x}_2) = P(\V{x}_2 | \V{x}_1) P(\V{x}_1) \)
    - Only \( P(\V{x}_1) \) needs be sampled 
      (\( \V{x}_1^{(i)} \sim P(\V{x}_1) \)) if \( P(\V{x}_2 | \V{x}_1) \)
      can be represented analytically
  - Joint distribution is represented by the set
    \( \{ \V{x}_1^{(i)}, P(\V{x}_2 | \V{x}_1^{(i)}) \} \)
  - Statistics can be obtained by sampling over the joint space
    - E.g., the marginal:
      \[ P(\V{x}_2) \approx \frac{1}{N} \sum_i^{N} P(\V{x}_2 | \V{x}_1^{(i)}) \]
- Joint SLAM state may be factored into a **_vehicle component_** and a 
  **_conditional map component_**:
  \begin{align*}
      P(\V{X}_{0:k}, \V{m} | \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0) 
      &= P(\V{X}_{0:k} | \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0) 
         P(\V{m} | \V{X}_{0:k}, \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0) \\
      &= P(\V{X}_{0:k} | \V{Z}_{0:k}, \V{U}_{0:k}, \V{x}_0) 
         P(\V{m} | \V{X}_{0:k}, \V{Z}_{0:k})
  \end{align*}
  - Note: \( P(A, B | C) = P(A | C) P(B | A, C) \)
  - Map landmarks **_become independent_** if the distribution is conditioned
    on the trajectory \( \V{X}_{0:k} \) rather than a single pose \( \V{x}_k \)
    \[
        P(\V{m} | \V{X}_{0:k}, \V{Z}_{0:k})
        = \prod_{j=1}^{n} P(\V{m}_j | \V{X}_{0:k}, \V{Z}_{0:k})
    \]
    - Conditional independence of landmarks: 
      - As landmark status is dependent with history of observations & 
        robot poses, if all the history info are known, any landmark status
        in the landmark set can be extracted using the same history info
        - Thus there's no dependence between any two of the landmarks given
          the history info (observation history is manifestly known)
      - \( 
            P(\V{m}_i | \V{m}_j, \V{X}_{0:k}, \V{Z}_{0:k})
            = P(\V{m}_i | \V{X}_{0:k}, \V{Z}_{0:k})
        \)
    - The map is represented as a set of **_independent Gaussians_** in 
      FastSLAM with linear complexity rather than a joint map of covariance 
      with quardratic complexity (\( n^2 \) elements in the covariance
      matrix at any given time \( k \))
- Rao-Blackwellized joint distribution (\( N \)-sample set) at time \( k \):
  \( 
      \{ 
          w_k^{(i)}, \V{X}_{0:k}^{(i)}, 
          P(\V{m} | \V{X}_{0:k}^{(i)}, \V{Z}_{0:k}) 
      \}_i^N
  \)
  - \( w_k^{(i)} \): weight for trajectory particle/sample
    \( \V{X}_{0:k}^{(i)} \)
  - Rao-Blackwellized conditional map component for each particle 
    in the above set
    \(
        P(\V{m} | \V{X}_{0:k}^{(i)}, \V{Z}_{0:k})
        = \prod_{j=1}^{n} P(\V{m}_j | \V{X}_{0:k}^{(i)}, \V{Z}_{0:k})
    \)
- Particle filter theory is derived from a recursive form of sampling known as
  **_sequential importance sampling_** (SIS)
  - Sampling recursively via the product rule rather than directly sampling 
    the joint state
    \[
        P(\V{x}_0, \V{x}_1, \ldots, \V{x}_T | \V{Z}_{0:T})
        = P(\V{x}_0 | \V{Z}_{0:T}) P(\V{x}_1 | \V{x}_0, \V{Z}_{0:T})
          \ldots P(\V{x}_T | \V{X}_{0:T-1}, \V{Z}_{0:T})
    \]
- General form of R-B particle filter for SLAM
  - Assume the joint state is represented as 
    \( \{ w_{k-1}^{(i)}, \V{X}_{0:k-1}^{(i)}, 
          P(\V{m} | \V{X}_{0:k-1}^{(i)}, \V{Z}_{0:k-1}) \}_i^N \)
    at time \( k-1 \)
  - Step 1: draw a sample from a proposal distribution conditioned on 
    specific particle history
    \[
        \V{x}_k^{(i)} \sim 
        \pi(\V{x}_k | \V{X}_{0:k-1}^{(i)}, \V{Z}_{0:k}, \V{u}_k)
    \]
    - Implicitly update the particle history:
      \( \V{X}_{0:k}^{(i)} \triangleq \{\V{X}_{0:k-1}^{(i)}, \V{x}_k^{(i)}\} \)
  - Step 2: compute sample weight
    \[
        w_k^{(i)} 
        = w_{k-1}^{(i)} \frac
          { P(\V{z}_k | \V{X}_{0:k}^{(i)}, \V{Z}_{0:k-1}) 
            P(\V{x}_k^{(i)} | \V{x}_{k-1}^{(i)}, \V{u}_k) } 
          { \pi(\V{x}_k^{(i)} | \V{X}_{0:k}^{(i)}, \V{Z}_{0:k}^{(i)}) }
    \]
    - Numerator terms: **_observation model_** (map info being marginalized) &
      **_motion model_**
      - Map marginalization for _observation model_:
        \[
            P(\V{z}_k | \V{X}_{0:k}, \V{Z}_{0:k-1}) 
            = \int P(\V{z}_k | \V{x}_k, \V{m}) 
                   P(\V{m} | \V{X}_{0:k-1} \V{Z}_{0:k-1}) \mathrm{d}\V{m}
        \]
  - Step 3: perform resampling if necessary
    - Select particles with replacement from the set 
      \( \{ \V{X}_{0:k}^{(i)} \}_i^N \) and set uniform weight 
      \( w_k^{(i)} = 1/N \)
  - Step 4: update observed landmarks using EKF as a simple mapping function 
    with known vehicle pose

\newpage
