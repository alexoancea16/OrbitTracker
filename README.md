# OrbitTracker - Documentation
The project produces graphs that visualize satellite trajectories and integration errors between the RK4 and Simulink methods. It also analyzes uncertainties in the initial positions and the probability that they exceed a predetermined threshold. <br>
Project made by Alexandru Oancea <br>
Date: January 2025 <br>

## Model description
Let the position vector $\mathbf{r}(t) = [x(t) \quad y(t) \quad z(t)]^T$. Then the motion of a satellite orbiting the Earth can be approximated by the solution of the system of differential equations:

```math
\ddot{{r}}(t) =
- \frac{GM_\oplus}{\|{r}\|_2^2} \frac{{r}}{\|{r}\|_2}
- \frac{3}{2} J_2 GM_\oplus \frac{R_\oplus^2}{\|{r}\|_2^{5}}
\left[
\begin{array}{c}
    \frac{x - 5xz^2}{\|{r}\|_2^2} \\[10pt]
    \frac{y - 5yz^2}{\|{r}\|_2^2} \\[10pt]
    \frac{3z - 5z^3}{\|{r}\|_2^2}
\end{array}
\right]
+ \omega_\oplus^2 \left[
\begin{array}{c}
   x \\ 
    y \\
    0
\end{array} \right]
+ 2\omega_\oplus 
\left[
\begin{array}{c}
    \dot{y} \\ 
    -\dot{x} \\ 
    0
\end{array} \right] = g(t, {r}(t), \dot{{r}}(t))

```

```math
{r}(t_0) ={r_0}; \dot{{r}}(t_0) = \dot{{r_0}}

```
To avoid the need for explicit transformations of the reference system, the equation of motion is
expressed in the fixed and rotating Earth frame of reference. Consequently, the centrifugal
and Coriolis terms are taken into account in the modeled acceleration. The constants have the values ​​in the table below.
| Simbol  | Semnificație                             | Valoare                 | Unitate          |
|---------|-----------------------------------------|-------------------------|------------------|
| *G*     | Constanta gravitațională universală    | 6.674 · 10⁻¹¹           | Nm²kg⁻²         |
| *Mₑ*    | Masa Pământului                        | 5.972 · 10²⁴            | kg              |
| *Rₑ*    | Raza Pământului                        | 6,371,000               | m               |
| *J₂*    | Coeficient gravitațional               | 1.08262668 · 10⁻³       | -               |
| *ωₑ*    | Viteza unghiulară a Pământului         | 7.2921 · 10⁻⁵           | rad·s⁻¹         |

### Reference
Teunissen, P.J. and Montenbruck, O. eds., 2017. Springer handbook of global navigation satellite
systems (Vol. 10, pp. 978-3). Cham, Switzerland: Springer International Publishing.

### Model
The model is represented by two satellites orbiting the Earth, with position vectors ${r_1}(t)$ and ${r_2}(t)$, being described by the system of equations below:
```math
$$
\ddot{{r_1}}(t) = g(t, {{r_1}}(t), \dot{{r_1}}(t)) + u(t)
$$

$$
\ddot{{r_2}}(t) = g(t, {{r_2}}(t), \dot{{r_2}}(t))
$$

$$
\dot{\varepsilon}(t) = \frac{\| {{r_1}(t) - {r_2}(t) } \|_2}{\| {r_2}(t)\|_2}
$$

$$
y(t) = \varepsilon(t)
$$
```
The exogenous signal $u(t) = k \cdot 10^{-3} \cdot 1(t)$ represents a perturbing acceleration with a constant value $k$, chosen. <br>
The initial conditions are:
```math
$$
{r_1}(t_0) = 10^6 \cdot 
\begin{bmatrix} 
-3.111567646661099 \\ 
2.420733547442338 \\ 
-5.626803092595423 
\end{bmatrix}
$$

$$
{r_2}(t_0) = 10^6 \cdot 
\begin{bmatrix} 
-3.422732421327209 \\ 
2.662806902186572 \\ 
-6.18948308151366 
\end{bmatrix}
$$

$$
\dot{{r_1}}(t_0) = 10^3 \cdot 
\begin{bmatrix} 
4.953572247000772 \\ 
-3.787243278806948 \\ 
-4.362500902062312 
\end{bmatrix}
$$

$$
\dot{{r_2}}(t_0) = 10^3 \cdot 
\begin{bmatrix} 
5.448929471700850 \\ 
-4.165967606687643 \\ 
-4.798750992268544 
\end{bmatrix}
$$
```
## Analysis and Interpretation
### Stage 1
The first requirement for carrying out the project is to choose an initial condition.
```math
\varepsilon_0 = 0.05; 
```
### Stage 2
Creating the model in Simulink using Integrator and Matlab function blocks. We considered a Matlab function block to simulate the system of equations that describe the model. For the input $u(t)$ we initially considered a Step block with the associated value $u(t)=2 \cdot 10^{-3} \cdot 1(t)$. Also, for each of the Integrator blocks we added the initial conditions. The model in the _SimulinkFileModel.slx_ file is represented in the figure below.

### Stage 3
### Stage 4
### Stage 5

