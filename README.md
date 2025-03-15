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

