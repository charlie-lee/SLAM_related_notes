# Notes on State Estimation for Robotics Book [@Barfoot2017] ([link](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf))

## Part I - Estimation Machinery

### 2. Primer on Probability Theory

#### 2.1 Probability Density Functions

##### 2.1.2 Bayes' Rule and Inference
- Factorization of joint probability
  \[ 
     p(\V{x}, \V{y}) = p(\V{x}|\V{y}) p(\V{y}) = p(\V{y}|\V{x}) p(\V{x}) 
  \]
  - \( \V{x} = (x_1, ..., x_N) \), \( \V{y} = (y_1, ..., y_M) \)
- Rearranging gives _Bayes' rule_:
  \begin{align*}
    p(\V{x}|\V{y}) &= \frac {p(\V{y}|\V{x}) p(\V{x})} {p(\V{y})} \\
                   &= \frac {p(\V{y}|\V{x}) p(\V{x})} 
                            {\int p(\V{y}|\V{x}) p(\V{x}) \mathrm{d}\V{x}}
  \end{align*}
  - \( p(\V{x}|\V{y}) \): (_inferred_) _posterior_/likelihood of the state
    given the measurements
    - _Posterior_ density
  - \( p(\V{x}) \): _prior_ probability density function (PDF) over the state
    - _Prior_ density
  - \( p(\V{y}|\V{x}) \): sensor model

##### 2.1.6 Normalized Product
- Normalized product \( p(\V{x}) \) of 2 PDFs \( p_1(\V{x}) \) 
  & \( p_2(\V{x}) \) for \( \V{x} \)
  \[
     p(\V{x}) = \eta p_1(\V{x}) p_2(\V{x})
  \]
  - \( \eta \): normalization constant
    \[
       \eta = (\int p_1(\V{x}) p_2(\V{x}) \mathrm{d}\V{x})^{-1}
    \]


## Part II - Three Dimensional Machinery

### 7. Matrix Lie Groups
- Lie group: a group that is also a **differentiable manifold**
  - Property: group operations are smooth

#### 7.1 Geometry
- Two specific Lie groups:
  (1) \( SO(3) \): special orthogonal group, which can represent rotations
  (2) \( SE(3) \): special Euclidean group, which can represent poses

##### 7.1.1 Special Orthogonal and Special Euclidean Groups
- Special orthogonal group: set of valid rotation matrices
  \[
     SO(3) = 
     \left\{ 
       \V{C} \in \F{R}^{3 \times 3} 
             \,\middle|\, \V{C} \V{C}^T = \V{I}, \mathrm{det} \V{C} = 1 
     \right\}
  \]
- Special Euclidean group: set of valid transformation matrices
  \[
     SE(3) = 
     \left\{ 
       \V{T} = \begin{bmatrix} \V{C} & \V{r} \\ \V{0}^T & 1 \end{bmatrix}
             \in \F{R}^{4 \times 4} 
             \,\middle|\, \V{C} \in SO(3), \V{r} \in \F{R}^3 
     \right\}
  \]
- Lie group properties for \( SO(3) \) and \( SE(3) \):  
  ![07_tab_01](images/ch05/07_tab_01.jpg){ width=75% }

##### 7.1.2 Lie Algebras
- Lie algebra
  - Associated with every matrix Lie group
  - Consists of a vector space \( \mathfrak{g} \), over some field \( \F{F} \),
    together with a binary operation \( [\cdot\,,\,\cdot] \) called 
    *Lie bracket* 
    - Lie bracket: non-associative, alternating bilinear map
      \( \mathfrak{g} \times \mathfrak{g} \to \mathfrak{g}; \, 
         (x, y) \mapsto [x, y] \)
    - Visualization of Lie bracket: [link](http://visuallietheory.blogspot.com/2012/07/infinitesimal-rotations.html)
  - Properties of Lie bracket for all \( \V{X}, \V{Y}, \V{Z} \in \mathfrak{g} \)
    and \( a, b \in \F{F} \)
    a) Closure: \( [\V{X}, \V{Y}] \in \mathfrak{g} \)
    b) Bilinearity: 
       - \( [a \V{X} + b \V{Y}, \V{Z}] = a[\V{X}, \V{Z}] + b[\V{Y}, \V{Z}] \)
       - \( [\V{Z}, a \V{X} + b \V{Y}] = a[\V{Z}, \V{X}] + b[\V{Z}, \V{Y}] \)
    c) Alternating: \( [\V{X}, \V{X}] = \V{0} \)
    d) Jacobi identity: 
       \( [\V{X}, [\V{Y}, \V{Z}]] + 
          [\V{Y}, [\V{Z}, \V{X}]] + 
          [\V{Z}, [\V{X}, \V{Y}]] 
          = \V{0}\)
- Lie algebra associated with \( SO(3) \) (rotations)
  - Definition:
    - Vector space: 
      \( \mathfrak{so}(3) = 
         \left\{ \BG{\Phi} = \BG{\phi}^{\wedge} \in \F{R}^{3 \times 3} 
                 \,\middle|\, \BG{\phi} \in \F{R}^3 \right\} \)  
      where \( \BG{\phi}^{\wedge} \) is a [skew-symmetric matrix](https://en.wikipedia.org/wiki/Skew-symmetric_matrix) generated from vector \( \BG{\phi} \):
      \[ \BG{\phi}^{\wedge}
         = \begin{bmatrix} \phi_1 \\ \phi_2 \\ \phi_3 \end{bmatrix}^{\wedge}
         = \begin{bmatrix} 
                   0 & -\phi_3 &  \phi_2 \\
              \phi_3 &       0 & -\phi_1 \\
             -\phi_2 &  \phi_1 &       0
           \end{bmatrix}
         \in \F{R}^{3 \times 3}, \, \BG{\phi} \in \F{R}^3
      \]
    - Field: \( \F{R} \)
    - Lie bracket: 
      \( [\BG{\Phi}_1, \BG{\Phi}_2] 
         = \BG{\Phi}_1 \BG{\Phi}_2 - \BG{\Phi}_2 \BG{\Phi}_1 \)
  - \( \BG{\Phi} = \BG{\phi}^{\wedge} \), \( \BG{\phi} = \BG{\Phi}^{\vee} \)
- Lie algebra associated with \( SE(3) \) (poses)
  - Definition:
    - Vector space:
      \( \mathfrak{se}(3) = 
         \left\{ \BG{\Xi} = \BG{\xi}^{\wedge} \in \F{R}^{4 \times 4}
         \,\middle|\, \BG{\xi} \in \F{R}^6 \right\} \)  
      where
      \[ \BG{\xi}^{\wedge}
         = \begin{bmatrix} \BG{\rho} \\ \BG{\phi} \end{bmatrix}^{\wedge}
         = \begin{bmatrix} 
             \BG{\phi}^{\wedge} & \BG{\rho} \\
                        \V{0}^T &         0
           \end{bmatrix}
         \in \F{R}^{4 \times 4}, \, \BG{\rho}, \BG{\phi} \in \F{R}^3
      \]
      - \( \BG{\rho} \): translation component of pose
      - \( \BG{\phi} \): rotation component of pose
    - Field: \( \F{R} \)
    - Lie bracket:
        \( [\BG{\Xi}_1, \BG{\Xi}_2] 
           = \BG{\Xi}_1 \BG{\Xi}_2 - \BG{\Xi}_2 \BG{\Xi}_1 \)
  - \( \BG{\Xi} = \BG{\xi}^{\wedge} \), \( \BG{\xi} = \BG{\Xi}^{\vee} \)
- Lie algebra: **tangent space** of the corresponding Lie group 
  (normally, at the *identity* \( \V{I} \) of a Lie group)
  - Why *identity*: for Lie group \( G \), the tangent spaces at all points 
    of \( G \) are *isomorphic*. So we pick the identity as \( \V{I} \in G \)
    is guaranteed for every Lie group \( G \)
  - Intuitive explanation:  
    Think of tangent space of a surface at a point as
    the space of all "directions" one can move from that point 
    (with all velocities) *while staying on that surface*.  
    
    The paths through the point is not on the surface, but a very small portion
    of the path will be very likely on it.  
    
    Start from the identity (in this example, \( \V{I} \in SO(n) \)):
    \[
       (\V{I} + \alpha\V{t})^T (\V{I} + \alpha\V{t})
       = \V{I} + \alpha (\V{t} + \V{t}^T) + O(\alpha^2)
    \]
    where \( \V{t} \in \F{R}^{3 \times 3} \) and \( \alpha \rightarrow 0 \).  
    
    If we ignore the \( \alpha^2 \) term, \( \V{I} + \alpha\V{t} \) will be
    in the Lie group if \( \V{t} + \V{t}^T = \V{0} \) for small \( \alpha \).
    That is, the skew-symmetric matrices \( \V{t} \) form the Lie algebra 
    \( \mathfrak{so}(n) \).
  - More generally, the Lie algebra gives the set of possible ways 
    (directions) in which you can change the group elements while staying in 
    the group

  
##### 7.1.3 Exponential Map
- Matrix exponential
  \[
     \mathrm{exp}(\V{A}) 
     = \V{I} + \V{A} + \frac{1}{2!} \V{A}^2 + \frac{1}{3!} \V{A}^3 + \cdots
     = \displaystyle \sum_{n=0}^{\infty} \frac{1}{n!} \V{A}^n
  \]
- Matrix logarithm
  \[
     \mathrm{ln}(\V{A})
     = \displaystyle \sum_{n=1}^{\infty} \frac{(-1)^{n-1}}{n} (\V{A} - \V{I})^n
  \]
- Exponential map: \( \mathfrak{so}(3) \leftrightarrow SO(3) \)
  - \( \mathfrak{so}(3) \to SO(3) \):
    \[
       \V{C} 
       = \mathrm{exp}(\BG{\phi}^{\wedge})
       = \displaystyle \sum_{n=0}^{\infty} \frac{1}{n!} (\BG{\phi}^{\wedge})^n
    \]
    where \( \V{C} \in SO(3) \), and \( \BG{\phi} \in \F{R}^3 \) 
    (\( \BG{\phi}^{\wedge} \in \mathfrak{so}(3) \))
    - Let \( \BG{\phi} = \phi \V{a} \) where \( \phi = |\BG{\phi}| \) 
      is the rotation angle, and \( \V{a} \) is a unit-vector representing 
      rotation axis (angle-axis representation)
      \[
         \V{C}
         = \mathrm{exp}(\BG{\phi}^{\wedge})
         = \mathrm{exp}(\phi \V{a}^{\wedge})
         = \cos \phi \V{I} + \sin \phi \V{a}^{\wedge} 
           + (1 - \cos \phi) \V{a} \V{a}^T
      \]
      (*Rodrigues' formula*)
  - \( SO(3) \to \mathfrak{so}(3) \) (not uniquely):
    \[
       \BG{\phi} = \mathrm{ln}(\V{C})^{\vee} = (\BG{\phi}^{\wedge})^{\vee}
    \]
- Exponential map: \( \mathfrak{se}(3) \leftrightarrow SE(3) \)
  - \( \mathfrak{se}(3) \to SE(3) \):
    \begin{align*}
      \V{T}
      &= \mathrm{exp}(\BG{\xi}^{\wedge}) \\
      &= \displaystyle \sum_{n=0}^{\infty} \frac{1}{n!} (\BG{\xi}^{\wedge})^n \\
      &= \displaystyle \sum_{n=0}^{\infty} \frac{1}{n!} 
         \left(
           \begin{bmatrix} \BG{\rho} \\ \BG{\xi} \end{bmatrix}^{\wedge}
         \right)^n \\
      &= \displaystyle \sum_{n=0}^{\infty} \frac{1}{n!} 
         \begin{bmatrix} 
           \BG{\phi}^{\wedge} & \BG{\rho} \\
                      \V{0}^T &         0
         \end{bmatrix}^n \\
      &= \begin{bmatrix} 
           \sum_{n=0}^{\infty} \frac{1}{n!} (\BG{\phi}^{\wedge})^n &
           \left(\sum_{n=0}^{\infty} \frac{1}{(n+1)!} 
            (\BG{\phi}^{\wedge})^n \right) \BG{\rho} \\
           \V{0}^T & 1 
         \end{bmatrix} \\
      &= \begin{bmatrix} \V{C} & \V{r} \\ \V{0}^T & 1 \end{bmatrix} \in SE(3)
    \end{align*}
    where \( \BG{\xi} \in \F{R}^6 \), \( \V{r} = \V{J} \BG{\rho} \in \F{R}^3 \),
    and \( \V{J} = \sum_{n=0}^{\infty} \frac{1}{(n+1)!} (\BG{\phi}^\wedge)^n \)
    (\( \BG{\xi}^{\wedge} \in \mathfrak{se}(3) \))
  - \( SE(3) \to \mathfrak{se}(3) \) (not uniquely):
    \[
       \BG{\xi} = \mathrm{ln}(\V{T})^{\vee} = (\BG{\xi}^{\wedge})^{\vee}
    \]
    - From \( \V{T} \in SE(3) \) to \( \BG{\xi} \in \F{R}^6 \):
      a) Compute \( \BG{\phi} \in \F{R}^3 \) from \( \V{C} \in SO(3) \)
      b) Compute \( \V{J} = \sum_{n=0}^{\infty} \frac{1}{(n+1)!} 
         (\BG{\phi}^{\wedge})^n \)
      c) Compute \( \BG{\rho} = \V{J}^{-1} \V{r} \)
      d) Composite \( \BG{\xi} \) using computed 
         \( \BG{\phi} \) and \( \BG{\rho} \)
- Jacobian \( \V{J} \): 
  \( \displaystyle \sum_{n=0}^{\infty} \frac{1}{(n+1)!} (\BG{\phi}^\wedge)^n \)
  - Close-form expressions for \( \V{J} \) and \( \V{J}^{-1} \):
    \[
       \V{J} 
       = \frac{\sin \phi}{\phi} \V{I} 
         + (1 - \frac{\sin \phi}{\phi}) \V{a} \V{a}^T
         + \frac{1 - \cos \phi}{\phi} \V{a}^{\wedge}
    \]
    \[
       \V{J}^{-1}
       = \frac{\phi}{2} \cot \frac{\phi}{2} \V{I}
         + (1 - \frac{\phi}{2} \cot \frac{\phi}{2}) \V{a} \V{a}^T
         - \frac{\phi}{2} \V{a}^{\wedge}
    \]
    where 
    \( \phi = |\BG{\phi}| \) is the rotation angle and 
    \( \V{a} = \BG{\phi} / \phi \) is the unit-length rotation axis
  - \( \V{J} \V{J}^T \) and its inverse:
    \[
       \V{J} \V{J}^T = \gamma \V{I} + (1 - \gamma) \V{a} \V{a}^T
    \]
    \[
       (\V{J} \V{J}^T)^{-1} 
       = \frac{1}{\gamma} \V{I} + (1 - \frac{1}{\gamma}) \V{a} \V{a}^T
    \]
    where \( \gamma = 2 \frac{1 - \cos \phi}{\phi^2} \)
    - \( \V{J} \V{J}^T \) is positive-definite
  - \( \V{J} \) represented by rotation matrix \( \V{C} \):
    \[
       \V{J} = \int_{0}^{1} \V{C}^{\alpha} \mathrm{d} \alpha
    \]
  - Rotation matrix \( \V{C} \) represented by Jacobian \( \V{J} \):
    \[
       \V{C} = \V{I} + \BG{\phi}^{\wedge} \V{J}
    \]
    - Impossible to solve \( \V{J} \) because \( \BG{\phi} \) is invertible
      in the foregoing expression
- Direct series expression for \( \V{T} \in SE(3) \)
  \[
     \V{T} 
     = \V{I} + \BG{\xi}^{\wedge} 
       + (\frac{1 - \cos \phi}{\phi^2}) (\BG{\xi}^{\wedge})^2
       + (\frac{\phi - \sin \phi}{\phi^3}) (\BG{\xi}^{\wedge})^3
  \]
  - Based on the identity: 
    \(
       (\BG{\xi}^{\wedge})^4 + \phi^2 (\BG{\xi}^{\wedge})^2 \equiv \V{0}
    \)
    
##### 7.1.4 Adjoints
- Adjoint of an element of \( SE(3) \):
  \[
     \mathbcal{T}_{6 \times 6} 
     = \mathrm{Ad}(\V{T}_{4 \times 4})
     = \mathrm{Ad}
       \left(\begin{bmatrix} \V{C} & \V{r} \\ \V{0}^T & 1 \end{bmatrix}\right)
     = \begin{bmatrix} \V{C} & \V{r}^\wedge \V{C} \\ \V{0} & \V{C} \end{bmatrix}
  \]
  where
  \[
     \V{r}^{\wedge} 
     = \begin{bmatrix} r_1 \\ r_2 \\ r_3 \end{bmatrix}^{\wedge}
     = \begin{bmatrix} 
            0 & -r_3 &  r_2 \\ 
          r_3 &    0 & -r_1 \\ 
         -r_2 &  r_1 &    0 
       \end{bmatrix}
  \]
- Adjoints of all elements of \( SE(3) \):
  \[
     \mathrm{Ad}(SE(3)) 
     = \left\{ \mathbcal{T} = \mathrm{Ad}(\V{T}) 
               \,\middle|\, \V{T} \in SE(3) \right\}
  \]
- \( \mathrm{Ad}(SE(3)) \) is a Lie group:
  - \( \mathbcal{T}_1 \mathbcal{T}_2 \in \mathrm{Ad}(SE(3)) \)
  - \( \mathbcal{T}^{-1} \in \mathrm{Ad}(SE(3)) \)
- Adjoint of an element \( \BG{\Xi} = \BG{\xi}^{\wedge} \in \mathfrak{se}(3) \):
  \[
     \mathrm{ad}(\BG{\Xi}) 
     = \mathrm{ad}(\BG{\xi}^{\wedge}) = \BG{\xi}^{\curlywedge}
  \]
  where
  \[
     \BG{\xi}^{\curlywedge}
     = \begin{bmatrix} \BG{\rho} \\ \BG{\phi} \end{bmatrix}^{\curlywedge}
     = \begin{bmatrix} 
         \BG{\phi}^{\wedge} & \BG{\rho}^{\wedge} \\ 
                      \V{0} & \BG{\phi}^{\wedge}
       \end{bmatrix}
     \in \F{R}^{6 \times 6}, \qquad \BG{\rho}, \BG{\phi} \in \F{R}^3
  \]
- The Lie algebra associated with \( \mathrm{Ad}(SE(3)) \):
  - Vector space: 
    \( \mathrm{ad}(\mathfrak{se}(3))
       = \left\{ \BG{\Psi} = \mathrm{ad}(\BG{\Xi}) \in \F{R}^{6 \times 6}
                 \,\middle|\, \BG{\Xi} \in \mathfrak{se}(3)
         \right\}
    \)
  - Field: \( \F{R} \)
  - Lie bracket: 
    \( [\BG{\Psi}_1, \BG{\Psi}_2] 
       = \BG{\Psi}_1 \BG{\Psi}_2 - \BG{\Psi}_2 \BG{\Psi}_1 \)
- Exponential map: 
  \( \mathrm{ad}(\mathfrak{se}(3)) \leftrightarrow \mathrm{Ad}(SE(3)) \)
  - \( \mathrm{ad}(\mathfrak{se}(3)) \to \mathrm{Ad}(SE(3)) \):
    \[
       \mathbcal{T} 
       = \mathrm{exp}(\mathrm{ad}(\BG{\xi}^{\wedge}))
       = \displaystyle \sum_{n=0}^{\infty} 
         \frac{1}{n!} (\BG{\xi}^{\curlywedge})^n
    \]
    where \( \mathbcal{T} \in \mathrm{Ad}(SE(3)) \) and 
    \( \BG{\xi} \in \F{R}^6 \)
    (\( \BG{\xi}^{\curlywedge} \in \mathrm{ad}(\mathfrak{se}(3)) \))
  - \( \mathrm{Ad}(SE(3)) \to \mathrm{ad}(\mathfrak{se}(3)) \) (not uniquely):
    \[
       \BG{\xi} 
       = \mathrm{ln}(\mathbcal{T})^{\curlyvee} 
       = (\BG{\xi}^{\curlywedge})^{\curlyvee}
    \]
  - Relationship between various Lie groups and algebras associated with poses  
    ![07_eq_056](images/ch05/07_eq_056.jpg){ width=50% }
- Direct series expression for \( \mathbcal{T} \in \mathrm{Ad}(SE(3)) \)
  \begin{align*}
    \mathbcal{T}
    = &\,\V{I}
      + (\frac{3\sin\phi - \phi \cos\phi}{2\phi}) \BG{\xi}^{\curlywedge}
      + (\frac{4 - \phi \sin\phi - 4\cos\phi}{2\phi^2}) 
        (\BG{\xi}^{\curlywedge})^2 \\
      &+ (\frac{\sin\phi - \phi \cos\phi}{2\phi^3}) 
        (\BG{\xi}^{\curlywedge})^3
      + (\frac{2 - \phi \sin\phi - 2\cos\phi}{2\phi^4}) 
        (\BG{\xi}^{\curlywedge})^4
  \end{align*}
  - Based on the identity: 
    \(
       (\BG{\xi}^{\curlywedge})^5 
       + 2 \phi^2 (\BG{\xi}^{\curlywedge})^3 
       + \phi^4 \BG{\xi}^{\curlywedge} 
       \equiv \V{0}
    \)

##### 7.1.5 Baker-Campbell-Hausdorff
- BCH formula: solution to \( Z \) in equation \( e^X e^Y = e^Z \) for possibly 
  noncommunicative \( X \) and \( Y \) in the Lie algebra of a Lie group
- General combinatorial formula by Eugene Dynkin (1947):
  \[
     \mathrm{ln}(\mathrm{exp}(\V{A}) \mathrm{exp}(\V{B}))
     = \displaystyle \sum_{n=1}^{\infty} \frac{(-1)^{n-1}}{n}
       \displaystyle \sum_{\substack{r_i + s_i > 0, \\ 1 \leq i \leq n}} 
       \frac {(\displaystyle \sum_{i=1}^n (r_i + s_i))^{-1}} 
             {\displaystyle \prod_{i=1}^n r_i! s_i!}
       \left[ \V{A}^{r_1} \V{B}^{s_1} \V{A}^{r_2} \V{B}^{s_2} \cdots 
              \V{A}^{r_n} \V{B}^{s_n} \right]
  \]
  where
  \[
     \left[ \V{A}^{r_1} \V{B}^{s_1} \V{A}^{r_2} \V{B}^{s_2} \cdots 
            \V{A}^{r_n} \V{B}^{s_n} \right]
     = \underbrace{[\V{A}, \dots [\V{A},}_{r_1}
       \underbrace{[\V{B}, \dots [\V{B},}_{s_1}
       \dots
       \underbrace{[\V{A}, \dots [\V{A},}_{r_n}
       \underbrace{[\V{B}, \dots [\V{B}, \V{B}], \dots]]}_{s_n}
       \dots] \dots] \dots]] \dots]
  \]
- Lie product formula:
  \[
     \mathrm{exp}(\V{A} + \V{B}) 
     = \displaystyle \lim_{\alpha \to \infty}
       (\mathrm{exp}(\V{A} / \alpha) \mathrm{exp}(\V{B} / \alpha))^{\alpha}
  \]
- BCH in \( SO(3) \) case
  - General case
    \[
       \mathrm{ln}(\V{C}_1 \V{C}_2)^{\vee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\phi}_1^{\wedge})
                     \mathrm{exp}(\BG{\phi}_2^{\wedge}))^{\vee} \\
       = \BG{\phi}_1 + \BG{\phi}_2 + \frac{1}{2} \BG{\phi}_1^\wedge \BG{\phi}_2
         + \frac{1}{12} \BG{\phi}_1^{\wedge} \BG{\phi}_1^{\wedge} \BG{\phi}_2
         + \frac{1}{12} \BG{\phi}_2^{\wedge} \BG{\phi}_2^{\wedge} \BG{\phi}_1
         + \cdots
    \]
  - Cases where either \( \BG{\phi}_1 \) or \( \BG{\phi}_2 \) is small
    \[
       \mathrm{ln}(\V{C}_1 \V{C}_2)^{\vee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\phi}_1^{\wedge})
                     \mathrm{exp}(\BG{\phi}_2^{\wedge}))^{\vee} \\
       \approx
         \begin{cases}
           \V{J}_l(\BG{\phi}_2)^{-1} \BG{\phi}_1 + \BG{\phi}_2 
           & \quad \text{if } \BG{\phi}_1 \text{ is small} \\
           \BG{\phi}_1 + \V{J}_r(\BG{\phi}_1)^{-1} \BG{\phi}_2
           & \quad \text{if } \BG{\phi}_2 \text{ is small}
         \end{cases}
    \]
    where
    \[
       \V{J}_r(\BG{\phi})^{-1}
       = \displaystyle\sum_{n=0}^{\infty} \frac{B_n}{n!} (-\BG{\phi}^\wedge)^n
       = \frac{\phi}{2} \cot\frac{\phi}{2} \V{I}
         + (1 - \frac{\phi}{2} \cot\frac{\phi}{2}) \V{a} \V{a}^T
         + \frac{\phi}{2} \V{a}^{\wedge}
    \]
    \[
       \V{J}_l(\BG{\phi})^{-1}
       = \displaystyle\sum_{n=0}^{\infty} \frac{B_n}{n!} (\BG{\phi}^\wedge)^n
       = \frac{\phi}{2} \cot\frac{\phi}{2} \V{I}
         + (1 - \frac{\phi}{2} \cot\frac{\phi}{2}) \V{a} \V{a}^T
         - \frac{\phi}{2} \V{a}^{\wedge}
    \]
    are *right* and *left* *Jacobians* of \( SO(3) \) respectively.
    The \( B_n \) are the [*Bernoulli numbers*](https://en.wikipedia.org/wiki/Bernoulli_number). \( \BG{\phi} = \phi \V{a} \) where \( \phi = |\BG{\phi}| \) and 
    \( \V{a} = \BG{\phi} / \phi \). 
    \( \V{C} = \mathrm{exp}(\BG{\phi}^{\wedge}) \).
- More about left & right Jacobians of \( SO(3) \)
  - Expressions for the Jacobians
    \[
      \V{J}_r(\BG{\phi})
      = \displaystyle\sum_{n=0}^{\infty} \frac{1}{(n+1)!} (-\BG{\phi}^\wedge)^n
      = \displaystyle\int_0^1 \V{C}^{-\alpha} \mathrm{d} \alpha \\
      = \frac{\sin\phi}{\phi} \V{I}
        + (1 - \frac{\sin\phi}{\phi}) \V{a} \V{a}^T
        - \frac{1 - \cos\phi}{\phi} \V{a}^{\wedge}
    \]
    \[
      \V{J}_l(\BG{\phi})
      = \displaystyle\sum_{n=0}^{\infty} \frac{1}{(n+1)!} (\BG{\phi}^\wedge)^n
      = \displaystyle\int_0^1 \V{C}^{\alpha} \mathrm{d} \alpha \\
      = \frac{\sin\phi}{\phi} \V{I}
        + (1 - \frac{\sin\phi}{\phi}) \V{a} \V{a}^T
        + \frac{1 - \cos\phi}{\phi} \V{a}^{\wedge}
    \]
  - Relationship between the 2 Jacobians
    \[
       \V{J}_l(\BG{\phi}) = \V{C} \V{J}_r(\BG{\phi})
    \]
    \[
       \V{J}_l(-\BG{\phi}) = \V{J}_r(\BG{\phi})
    \]
- BCH in \( SE(3) \) and \( \mathrm{Ad}(SE(3)) \) cases
  - General case
    \[
       \mathrm{ln}(\V{T}_1 \V{T}_2)^{\vee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\xi}_1^{\wedge})
                     \mathrm{exp}(\BG{\xi}_2^{\wedge}))^{\vee} \\
       = \BG{\xi}_1 + \BG{\xi}_2 + \frac{1}{2} \BG{\xi}_1^\curlywedge \BG{\xi}_2
         + \frac{1}{12} \BG{\xi}_1^\curlywedge \BG{\xi}_1^\curlywedge \BG{\xi}_2
         + \frac{1}{12} \BG{\xi}_2^\curlywedge \BG{\xi}_2^\curlywedge \BG{\xi}_1
         + \cdots
    \]
    \[
       \mathrm{ln}(\mathbcal{T}_1 \mathbcal{T}_2)^{\curlyvee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\xi}_1^{\curlywedge})
                     \mathrm{exp}(\BG{\xi}_2^{\curlywedge}))^{\curlyvee} \\
       = \BG{\xi}_1 + \BG{\xi}_2 + \frac{1}{2} \BG{\xi}_1^\curlywedge \BG{\xi}_2
         + \frac{1}{12} \BG{\xi}_1^\curlywedge \BG{\xi}_1^\curlywedge \BG{\xi}_2
         + \frac{1}{12} \BG{\xi}_2^\curlywedge \BG{\xi}_2^\curlywedge \BG{\xi}_1
         + \cdots
    \]
  - Cases where either \( \BG{\xi}_1 \) or \( \BG{\xi}_2 \) is small
    \[
       \mathrm{ln}(\V{T}_1 \V{T}_2)^{\vee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\xi}_1^{\wedge})
                     \mathrm{exp}(\BG{\xi}_2^{\wedge}))^{\vee} \\
       \approx
         \begin{cases}
           \mathbcal{J}_l(\BG{\xi}_2)^{-1} \BG{\xi}_1 + \BG{\xi}_2 
           & \quad \text{if } \BG{\xi}_1 \text{ is small} \\
           \BG{\xi}_1 + \mathbcal{J}_r(\BG{\xi}_1)^{-1} \BG{\xi}_2
           & \quad \text{if } \BG{\xi}_2 \text{ is small}
         \end{cases}
    \]
    \[
       \mathrm{ln}(\mathbcal{T}_1 \mathbcal{T}_2)^{\curlyvee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\xi}_1^{\curlywedge})
                     \mathrm{exp}(\BG{\xi}_2^{\curlywedge}))^{\curlyvee} \\
       \approx
         \begin{cases}
           \mathbcal{J}_l(\BG{\xi}_2)^{-1} \BG{\xi}_1 + \BG{\xi}_2 
           & \quad \text{if } \BG{\xi}_1 \text{ is small} \\
           \BG{\xi}_1 + \mathbcal{J}_r(\BG{\xi}_1)^{-1} \BG{\xi}_2
           & \quad \text{if } \BG{\xi}_2 \text{ is small}
         \end{cases}
    \]
    where
    \[
       \mathbcal{J}_r(\BG{\phi})^{-1}
       = \displaystyle\sum_{n=0}^{\infty} \frac{B_n}{n!} 
         (-\BG{\xi}^\curlywedge)^n
    \]
    \[
       \mathbcal{J}_l(\BG{\phi})^{-1}
       = \displaystyle\sum_{n=0}^{\infty} \frac{B_n}{n!} 
         (\BG{\xi}^\curlywedge)^n
    \]
    are *right* and *left* *Jacobians* of \( SE(3) \) respectively.
- More about left & right Jacobians of \( SE(3) \)
  - Expressions for the Jacobians
    \[
      \mathbcal{J}_r(\BG{\xi})
      = \displaystyle\sum_{n=0}^{\infty} \frac{1}{(n+1)!} 
        (-\BG{\xi}^\curlywedge)^n
      = \displaystyle\int_0^1 \mathbcal{T}^{-\alpha} \mathrm{d} \alpha \\
    \]
    \[
      \mathbcal{J}_l(\BG{\xi})
      = \displaystyle\sum_{n=0}^{\infty} \frac{1}{(n+1)!} 
        (\BG{\xi}^\curlywedge)^n
      = \displaystyle\int_0^1 \mathbcal{T}^{\alpha} \mathrm{d} \alpha \\
    \]
  - Relationship between the 2 Jacobians
    \[
       \mathbcal{J}_l(\BG{\xi}) = \mathbcal{T} \mathbcal{J}_r(\BG{\xi})
    \]
    \[
       \mathbcal{J}_l(-\BG{\xi}) = \mathbcal{J}_r(\BG{\xi})
    \]
- Rewrite BCH approximations using *left Jacobian* 
  (\( \V{J} = \V{J}_l \) and \( \mathbcal{J} = \mathbcal{J}_l \))
  - \( SO(3) \) case:
    \[
       \mathrm{ln}(\V{C}_1 \V{C}_2)^{\vee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\phi}_1^{\wedge})
                     \mathrm{exp}(\BG{\phi}_2^{\wedge}))^{\vee} \\
       \approx
         \begin{cases}
           \V{J}(\BG{\phi}_2)^{-1} \BG{\phi}_1 + \BG{\phi}_2 
           & \quad \text{if } \BG{\phi}_1 \text{ is small} \\
           \BG{\phi}_1 + \V{J}(-\BG{\phi}_1)^{-1} \BG{\phi}_2
           & \quad \text{if } \BG{\phi}_2 \text{ is small}
         \end{cases}
    \]
  - \( SE(3) \) case:
    \[
       \mathrm{ln}(\V{T}_1 \V{T}_2)^{\vee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\xi}_1^{\wedge})
                     \mathrm{exp}(\BG{\xi}_2^{\wedge}))^{\vee} \\
       \approx
         \begin{cases}
           \mathbcal{J}(\BG{\xi}_2)^{-1} \BG{\xi}_1 + \BG{\xi}_2 
           & \quad \text{if } \BG{\xi}_1 \text{ is small} \\
           \BG{\xi}_1 + \mathbcal{J}(-\BG{\xi}_1)^{-1} \BG{\xi}_2
           & \quad \text{if } \BG{\xi}_2 \text{ is small}
         \end{cases}
    \]
    \[
       \mathrm{ln}(\mathbcal{T}_1 \mathbcal{T}_2)^{\curlyvee} \\
       = \mathrm{ln}(\mathrm{exp}(\BG{\xi}_1^{\curlywedge})
                     \mathrm{exp}(\BG{\xi}_2^{\curlywedge}))^{\curlyvee} \\
       \approx
         \begin{cases}
           \mathbcal{J}(\BG{\xi}_2)^{-1} \BG{\xi}_1 + \BG{\xi}_2 
           & \quad \text{if } \BG{\xi}_1 \text{ is small} \\
           \BG{\xi}_1 + \mathbcal{J}(-\BG{\xi}_1)^{-1} \BG{\xi}_2
           & \quad \text{if } \BG{\xi}_2 \text{ is small}
         \end{cases}
    \]
  
##### 7.1.6 Distance, Volume, Integration
- Difference of 2 rotations: right difference \( \BG{\phi}_{12} \) 
  and left difference \( \BG{\phi}_{21} \)
  \[
     \BG{\phi}_{12} = \mathrm{ln}(\V{C}_1^T \V{C}_2)^{\vee}
  \]
  \[
     \BG{\phi}_{21} = \mathrm{ln}(\V{C}_2 \V{C}_1^T)^{\vee}
  \]
  where \( \V{C}_1, \V{C}_2 \in SO(3) \)
- Inner product for \( \mathfrak{so}(3) \):
  \[
     \langle \BG{\phi}_1^{\wedge}, \BG{\phi}_2^{\wedge} \rangle
     = \frac{1}{2} \mathrm{tr} 
       \left( \BG{\phi}_1^{\wedge} \BG{\phi}_2^{{\wedge}^T} \right)
     = \BG{\phi}_1^T \BG{\phi}_2
  \]
- The metric *distance* between 2 rotations: (i) the square root of the
  inner product of difference with itself, or (ii) the Euclidean norm of 
  the difference, or (iii) the magnitude of the angle of the rotation
  difference
  \[
     \phi_{12} 
     = \sqrt{\langle \BG{\phi}_{12}, \BG{\phi}_{12} \rangle}
     = \sqrt{\BG{\phi}_{12}^T \BG{\phi}_{12}}
     = |\BG{\phi}_{12}|
  \]
  \[
     \phi_{21} 
     = \sqrt{\langle \BG{\phi}_{21}, \BG{\phi}_{21} \rangle}
     = \sqrt{\BG{\phi}_{21}^T \BG{\phi}_{21}}
     = |\BG{\phi}_{21}|
  \]
- Integration on rotations
  - Let \( \V{C} = \mathrm{exp}(\BG{\phi}^{\wedge}) \in SO(3) \)
  - Let \( \V{C}' = \mathrm{exp}((\BG{\phi} + \delta\BG{\phi})^{\wedge}) \) 
    by purturbing \( \V{C} \) a bit
  - Right & left differences (relative to \( \V{C} \)):
    \[
       \mathrm{ln}(\delta \V{C}_r)^{\vee}
       = \mathrm{ln}(\V{C}^T \V{C}')^{\vee}
       \approx \mathrm{ln}(
         \V{C}^T \V{C}
         \,\mathrm{exp}((\V{J}_r \delta\BG{\phi})^{\wedge})
       )^{\vee}
       = \V{J}_r \delta\BG{\phi}
    \]
    \[
       \mathrm{ln}(\delta \V{C}_l)^{\vee}
       = \mathrm{ln}(\V{C}' \V{C}^T)^{\vee}
       \approx \mathrm{ln}(
         \mathrm{exp}((\V{J}_l \delta\BG{\phi})^{\wedge})
         \V{C} \V{C}^T
       )^{\vee}
       = \V{J}_l \delta\BG{\phi}
    \]
    where \( \V{J}_r \) and \( \V{J}_l \) are evaluated at \( \BG{\phi} \)
    - BCH approximation (assume \( \V{J}_r(\BG{\phi}) \delta\BG{\phi} \) 
      is small enough):
      \begin{align*}
        \V{C}' 
        &= \mathrm{exp}((\BG{\phi} + \delta\BG{\phi})^{\wedge}) \\
        &= \mathrm{exp}(
             (\BG{\phi} + 
              \V{J}_r(\BG{\phi})^{-1} 
              (\V{J}_r(\BG{\phi}) \delta\BG{\phi})
           )^{\wedge}) \\
        &\approx \mathrm{exp}(\BG{\phi}^{\wedge})
                 \mathrm{exp}((\V{J}_r(\BG{\phi}) \delta\BG{\phi})^{\wedge}) \\
        &= \V{C} \, \mathrm{exp}((\V{J}_r(\BG{\phi}) \delta\BG{\phi})^{\wedge})
      \end{align*}
  - Infinitestimal volume element
    \begin{align*}
      \mathrm{d}\V{C} 
      &= |\mathrm{det}\,\V{J}| \mathrm{d}\BG{\phi} \\
      &= \mathrm{d}\V{C}_r = |\mathrm{det}\,\V{J}_r| \mathrm{d}\BG{\phi} \\
      &= \mathrm{d}\V{C}_l = |\mathrm{det}\,\V{J}_l| \mathrm{d}\BG{\phi}
    \end{align*}
    - \( |\mathrm{det}(\V{J})| = 2 \frac{1 - \cos\phi}{\phi^2} \)
  - Integrating functions of rotations
    \[
       \int_{SO(3)} f(\V{C}) \mathrm{d}\V{C}
       \to
       \int_{|\phi|<\pi} f(\BG{\phi}) |\mathrm{det}\,\V{J}| \mathrm{d}\BG{\phi}
    \]
- Similar for poses (\( SE(3) \) and \( \mathrm{Ad}(SE(3)) \))
  - Inner product defined for \( \mathfrak{se}(3) \) (\( 4 \times 4 \)) and 
    \( \mathrm{ad}(\mathfrak{se}(3)) \) (\( 6 \times 6\))
    \[
       \langle \BG{\xi}_1^{\wedge}, \BG{\xi}_2^{\wedge} \rangle
       = - \mathrm{tr}
         \left(
           \BG{\xi}_1^{\wedge}
           \begin{bmatrix} \V{I}/2 & \V{0} \\ \V{0}^T & \V{I} \end{bmatrix}
           \BG{\xi}_2^{\wedge^T}
         \right)
       = \BG{\xi}_1^T \BG{\xi}_2
    \]
    \[
       \langle \BG{\xi}_1^{\curlywedge}, \BG{\xi}_2^{\curlywedge} \rangle
       = - \mathrm{tr}
         \left(
           \BG{\xi}_1^{\curlywedge}
           \begin{bmatrix} \V{I}/4 & \V{0} \\ \V{0}^T & \V{I}/2 \end{bmatrix}
           \BG{\xi}_2^{\curlywedge^T}
         \right)
       = \BG{\xi}_1^T \BG{\xi}_2
    \]

##### 7.1.7 Interpolation
- From typical linear interpolation scheme to interpolation of 2 rotations
  \[
     x = (1 - \alpha) x_1 + \alpha x_2 = \alpha(x_2 - x_1) + x_1
  \]
  \[
     \Downarrow 
  \]
  \[
     \V{C} = (\V{C}_2 \V{C}_1^T)^{\alpha} \V{C}_1
  \]
  where \( \alpha \in [0, 1] \), \( \V{C}, \V{C}_1, \V{C}_2 \in SO(3) \)
  - Let \( \V{C} = \mathrm{exp}(\BG{\phi}^{\wedge}) \),
    \( \V{C}_1 = \mathrm{exp}(\BG{\phi_1}^{\wedge}) \),
    \( \V{C}_2 = \mathrm{exp}(\BG{\phi_2}^{\wedge}) \in SO(3) \),
    with \( \BG{\phi} \), \( \BG{\phi}_1 \), \( \BG{\phi}_2 \in \F{R}^3 \)
    \begin{align*}
      \BG{\phi}
      &= \mathrm{ln}(\V{C})^{\vee}
       = \mathrm{ln}\left( (\V{C}_2 \V{C}_1^T)^{\alpha} \V{C}_1\right)^{\vee} \\
      &= \mathrm{ln}(\mathrm{exp}(\alpha \BG{\phi}_{21}^{\wedge}) 
                     \mathrm{exp}(\BG{\phi}_1^{\wedge}))^{\vee}
       \approx \alpha \V{J}(\BG{\phi}_1)^{-1} \BG{\phi}_{21} + \BG{\phi}_1
    \end{align*}
- Purturbed rotations (\( SO(3) \))
  - Let \( \V{C}' \), \( \V{C}_1' \), \( \V{C}_2' \in SO(3) \) be the purturbed
    rotation matrices with left differences given as
    \[
       \delta\BG{\phi} = \mathrm{ln}(\V{C}' \V{C}^T)^{\vee}, \quad
       \delta\BG{\phi}_1 = \mathrm{ln}(\V{C}_1' \V{C}_1^T)^{\vee}, \quad
       \delta\BG{\phi}_2 = \mathrm{ln}(\V{C}_2' \V{C}_2^T)^{\vee}
    \]
  - Interpolation scheme must hold for the purturbed rotation matrices
    \[
       \V{C}' = (\V{C}_2' \V{C}_1'^T)^{\alpha} \V{C}_1', \quad \alpha \in [0, 1]
    \]
    - Special case: 
      \( \V{C} = \V{C}_2^{\alpha} \), \( \BG{\phi} = \alpha \BG{\phi}_2 \) 
      when \( \V{C}_1 = \V{I} \)
  - Perturbation quantity:
    \[
       \delta\BG{\phi} 
       = (\V{I} - \V{A}(\alpha, \BG{\phi}_{21})) \delta\BG{\phi}_1
         + \V{A}(\alpha, \BG{\phi}_{21}) \delta\BG{\phi}_2
    \]
    where
    \[
       \V{A}(\alpha, \BG{\phi}_{21})
       = \alpha \V{J}(\alpha \BG{\phi}) \V{J}(\BG{\phi})^{-1}
    \]
    \[
       \BG{\phi}_{21} = \mathrm{ln}(\V{C}_2 \V{C}_1^T)^{\vee}
    \]
    - \( \V{A}(\alpha, \BG{\phi}) \approx \alpha \V{I} \) when 
      \( \BG{\phi} \) is small
    - \( \V{A}(\alpha, \BG{\phi}) \) has a series expression which can be
      used to compute it (only use the first several terms if \( \BG{\phi} \)
      is small
- Similar for purturbed poses (\( SE(3) \))

##### 7.1.8 Homogeneous Points
- Represent points in \( \F{R}^3 \) via \( 4 \times 1 \) 
  *homogeneous coordinates*:
  \[
     \V{p} 
     = \begin{bmatrix} sx \\ sy \\ sz \\ s \end{bmatrix}
     = \begin{bmatrix} \BG{\varepsilon} \\ \eta \end{bmatrix}
  \]
  where \( \BG{\varepsilon} \in \F{R}^3 \), \( \eta \) is a scalar
- Define 2 operators for manipulating homogeneous points:
  \[
     \begin{bmatrix} \BG{\varepsilon} \\ \eta \end{bmatrix}^{\odot}
     = \begin{bmatrix} 
         \eta \V{I}_{3 \times 3} & -\BG{\varepsilon}_{3 \times 3}^{\wedge} \\
         \V{0}_{1 \times 3}^T & \V{0}_{1 \times 3}^T
       \end{bmatrix}_{4 \times 6}
     , \quad
     \begin{bmatrix} \BG{\varepsilon} \\ \eta \end{bmatrix}^{\circledcirc}
     = \begin{bmatrix} 
         \V{0}_{3 \times 1} & \BG{\varepsilon}_{3 \times 1}  \\
         -\BG{\varepsilon}_{3 \times 3}^{\wedge} & \V{0}_{3 \times 1}
       \end{bmatrix}_{6 \times 4}
  \]
- Useful identities:
  - \( \BG{\xi}^{\wedge} \V{p} \equiv \V{p}^{\odot} \BG{\xi} \),
    \( \V{p}^T \BG{\xi}^{\wedge} \equiv \BG{\xi}^T \V{p}^{\circledcirc} \)
    where \( \BG{\xi} \in \F{R}^6 \) and \( \V{p} \in \F{R}^4 \)
  - \( (\V{T} \V{p})^{\odot} \equiv \V{T} \V{p}^{\odot} \mathbcal{T}^{-1} \)

##### 7.1.9 Calculus and Optimization
- Rotations
  - Directly taking Jacobian of a rotated point \( \V{C} \V{v} \) 
    w.r.t. the Lie algebra vector \( \BG{\phi} \) representing 
    the rotation \( \V{C} \):
    \[
       \frac{\partial (\V{C} \V{v})}{\partial \BG{\phi}}
       = - (\V{C} \V{v})^{\wedge} \V{J}
    \]
    where \( \V{C} = \mathrm{exp}(\BG{\phi}^{\wedge}) \in SO(3) \),
    \( \V{v} \in \F{R}^3 \) is some arbitrary three-dimensional point,
    and \( \V{J} = \V{J}_l(\BG{\phi}) \)
  - For scalar function \( u(\V{x}) \) with \( \V{x} = \V{C} \V{v} \),
    applying chain rule of differentiation
    \[
       \frac{\partial u}{\partial \BG{\phi}}
       = \frac{\partial u}{\partial \V{x}} 
         \frac{\partial \V{x}}{\partial \BG{\phi}}
       = - \frac{\partial u}{\partial \V{x}} (\V{C} \V{v})^{\wedge} \V{J}
    \]
    The result is the transpose of the gradient \( u \) w.r.t. \( \BG{\phi} \).
    - \( \mathrm{dim}(
           \partial u_{\V{1} \times 1} / 
           \partial \BG{\phi}_{\V{3} \times 1})
         = 1 \times 3 
      \)
  - Gradient descent on function \( u(\cdot) \): update in the direction of 
    negative gradient, evaluated at linearization point, 
    \( \V{C}_{op} = \mathrm{exp}(\BG{\phi}_{op}^{\wedge}) \)
    \begin{align*}
      \BG{\phi} 
      &= \BG{\phi}_{op} 
         - \alpha \left( \frac{\partial u}{\partial \BG{\phi}} \right)^T \\
      &= \BG{\phi}_{op}
         - \alpha 
           \left( 
             - \left.\frac{\partial u}{\partial \V{x}} 
               \right|_{\V{x} = \V{C}_{op} \V{v}}
             (\V{C}_{op} \V{v})^{\wedge} \V{J}
           \right)^T \\
      &= \BG{\phi}_{op}
         + \alpha \V{J}^T (-(\V{C}_{op} \V{v})^{\wedge})
           \left. \frac{\partial u}{\partial \V{x}} 
           \right|_{\V{x} = \V{C}_{op} \V{v}}^T \\
      &= \BG{\phi}_{op} - \alpha \V{J}^T 
         \underbrace{(\V{C}_{op} \V{v})^{\wedge}
           \left. \frac{\partial u}{\partial \V{x}} 
           \right|_{\V{x} = \V{C}_{op} \V{v}}^T}_{\BG{\delta}}
    \end{align*}
    where \( \alpha > 0 \) is the step size
  - Cleaner way to carry out optimization: find an update step for \( \V{C} \)
    in the form of small rotation on the left rather than directly on the 
    Lie algebra rotation vector \( \BG{\phi} \)
    \begin{align*}
      \V{C} 
      &= \mathrm{exp}(\BG{\phi}^{\wedge})
       = \mathrm{exp}
           \left(
             \BG{\phi}_{op} - \alpha \V{J}^T \BG{\delta})^{\wedge}
           \right) \\
      &\approx 
         \mathrm{exp}
           \left(
             -\alpha (\V{J} \V{J}^T \BG{\delta})^{\wedge}
           \right)
         \mathrm{exp}(\V{C}_{op}) \\
      &= \mathrm{exp}(\BG{\psi}^{\wedge}) \mathrm{exp}(\V{C}_{op})
    \end{align*}
    - Let \( \BG{\psi} = -\alpha \V{J} \V{J}^T \BG{\delta} \)
    - To avoid computing Jacobian, drop \( \V{J} \V{J}^T > 0 \) and use
      \( \BG{\psi} = -\alpha \BG{\delta} \) instead (slightly different
      direction for gradient descent)
  - Compute Jacobian w.r.t \( \BG{\psi} \), where the purturbation is applied
    on the left
    \[
       \frac {\partial (\V{C} \V{v})} {\partial \BG{\phi}}
       \approx \frac {\partial (\mathrm{exp}(\BG{\psi}^\wedge) \V{C} \V{v})}
                     {\partial \BG{\psi}}
       = - (\V{C} \V{v})^{\wedge}
    \]
  - Think optimization in terms of purturbation:
    - Purturbation scheme
      \begin{align*}
        \V{C} \V{v} 
        &= \mathrm{exp}(\BG{\psi}^\wedge) 
           \mathrm{exp}(\BG{\phi}_{op}^\wedge) \V{v} \\
        &= f(\BG{\phi}_{op} \oplus \BG{\psi}) \\
        &\approx f(\BG{\phi}_{op}) 
                 + \frac{\partial f(\BG{\phi}_{op})}{\partial \BG{\psi}} 
                   \BG{\psi} \\
        &= \mathrm{exp}(\BG{\phi}_{op}^\wedge) \V{v}
           - (\mathrm{exp}(\BG{\phi}_{op}^\wedge) \V{v})^\wedge \BG{\psi} \\
        &= \V{C}_{op} \V{v} - (\V{C}_{op} \V{v})^\wedge \BG{\psi}
      \end{align*}
      where \( \oplus \) denotes the purturbation operation, and 1st-order
      Taylor series is used for the approximation
    - Insert the scheme into the function to be optimized
      \begin{align*}
        u(\V{C} \V{v})
        &= u(\mathrm{exp}(\BG{\psi}^\wedge) \V{C}_{op} \V{v}) \\
        &\approx 
          u(\V{C}_{op} \V{v} - (\V{C}_{op} \V{v})^\wedge \BG{\psi}) \\
        &\approx 
          u(\V{C}_{op} \V{v})
          \underbrace{
            - \left. \frac{\partial u}{\partial \V{x}} 
              \right|_{\V{x} = \V{C}_{op} \V{v}} (\V{C}_{op} \V{v})^\wedge
          }_{\BG{\delta}^T}
          \BG{\psi} \\
        &= u(\V{C}_{op} \V{v}) + \BG{\delta}^T \BG{\psi}
      \end{align*}
    - An example perturbation vector for gradient descent:
      \[
         \BG{\psi} = - \alpha \V{D} \BG{\delta}
      \]
      where \( \alpha > 0 \) a small step size and \( \V{D} \) is any 
      positive-definite matrix
    - Update step:
      \[
         \V{C}_{op} 
         \leftarrow \mathrm{exp}(- \alpha \V{D} \BG{\delta}^\wedge) \V{C}_{op}
      \]
      where \( \V{C}_{op} \in SO(3) \) is guaranteed at each iteration
  - Purturbation applied to Gauss-Newton optimization method
    - An example general nonlinear, quadratic cost function of rotation
      \[
         J(\V{C}) = \frac{1}{2} \displaystyle\sum_m (u_m(\V{C} \V{v}_m))^2
      \]
      where \( u_m(\cdot) \) are scalar nonlinear functions
      and \( \V{v}_m \in \F{R}^3 \) are three-dimensional points
    - Purturb on an initial guess for the optimal rotation 
      \( \V{C}_{op} \in SO(3) \):
      \[
         \V{C} = \mathrm{exp}(\BG{\psi}^\wedge) \V{C}_{op}
      \]
      where \( \BG{\psi} \) is the purturbation
    - Apply purturbation scheme inside each \( u_m(\cdot) \) to get linearized
      version of \( u_m(\cdot) \) in terms of purturbation \( \BG{\psi} \)
      \[
         u_m(\V{C} \V{v}_m) 
         \approx 
           \underbrace{u_m(\V{C}_{op} \V{v}_m)}_{\beta_m}
           \underbrace{
             \left. \frac{\partial u_m}{\partial \V{x}} 
             \right|_{\V{x} = \V{C}_{op} \V{v}_m}
             (\V{C}_{op} \V{v}_m)^\wedge
           }_{\BG{\delta}_m^T}
           \BG{\psi}
      \]
      and then
      \[
         J(\V{C}) \approx 
         \frac{1}{2} \displaystyle\sum_m (\BG{\delta}_m^T \BG{\psi} + \beta_m)^2
      \]
    - Take the derivative of \( J \) w.r.t \( \BG{\psi} \)
      \[
         \frac{\partial J}{\partial \BG{\psi}^T} =
         \displaystyle\sum_m \BG{\delta}_m (\BG{\delta}_m^T \BG{\psi} + \beta_m)
      \]
    - Set derivative to 0 to find the optimal purturbation \( \BG{\psi}^* \)
      that minimizes \( J \)
      \[
         \left( \displaystyle\sum_m \BG{\delta}_m \BG{\delta}_m^T
         \right) \BG{\psi}^*
         = - \displaystyle\sum_m \beta_m \BG{\delta}_m
      \]
    - Apply the optimal purturbation for each iteration
      \[
         \V{C}_{op} 
         \leftarrow \mathrm{exp} \left( \BG{\psi}^{*^\wedge} \right) \V{C}_{op}
      \]
      where \( \V{C}_{op} \in SO(3) \) is guaranteed for each iteration
- Poses
  - The Jacobian of a transformed point w.r.t the Lie algebra vector 
    \( \BG{\xi} \) representing the transformation:
    \[
       \frac{\partial (\V{T} \V{p})}{\partial \BG{\xi}}
       = (\V{T} \V{p})^{\odot} \mathbcal{J}
    \]
  - The Jacobian w.r.t. the purturbation \( \BG{\epsilon} \) where it is applied
    on the left of the transformation (i.e. the (left) Lie derivative)
    \[
       \frac{\partial (\V{T} \V{p})}{\partial \BG{\epsilon}} 
       = (\V{T} \V{p})^{\odot}
       \quad \text{if} \quad 
       \V{T} \leftarrow \mathrm{exp}(\BG{\epsilon}^\wedge) \V{T}
    \]
    where the need to compute \( \mathbcal{J} \) matrix is removed
  - Gauss-Newton optimization is similar to rotations by using the 
    above purturbation scheme

\newpage
