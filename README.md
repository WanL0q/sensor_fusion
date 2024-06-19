# sensor_fusion
## EKF
![image](https://github.com/WanL0q/sensor_fusion/assets/134664967/8243e034-99c1-4908-82e4-ecafa5d9c8bd)
## Motion model
$$
\mathbf{x_{k+1}}
\=\
\begin{bmatrix}
x_{k+1} \\
y_{k+1} \\
\theta_{k+1} \\
v_{k+1} \\
\end{bmatrix}
\=\
\begin{bmatrix}
1 & 0 & 0 & cos(\theta_{k}).dt \\
0 & 1 & 0 & sin(\theta_{k}).dt \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
\.\
\begin{bmatrix}
x_{k} \\
y_{k} \\
\theta_{k} \\
v_{k} \\
\end{bmatrix}
\+\
\begin{bmatrix}
0  & 0  \\
0  & 0  \\
0  & dt \\
dt & 0  \\
\end{bmatrix}
\.\
\begin{bmatrix}
a_{k} \\
w_{k} \\
\end{bmatrix}
\=\
A_k \cdot \mathbf{x_k} + B_k \cdot \mathbf{u_k}
$$

$$
$$

$$
$$

$$
\mathbf{z_k} 
\=\
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
\cdot
\begin{bmatrix}
x_k \\
y_k \\
v_k \\
\end{bmatrix}
\=\
H \cdot \mathbf{x_k}
$$

$$
$$

$$
$$

$$
\mathbf{Q_k} 
\=\
\begin{bmatrix}
0.1 & 0  \\
0 & (\pi/180)\^2  \\
\end{bmatrix}
$$

$$
$$

$$
$$

$$
\mathbf{R_k} 
\=\
\begin{bmatrix}
1000 & 0 & 0 \\
0 & 1000 & 0  \\
0 & 0 & 1000  \\
\end{bmatrix}
$$

# Value
![Plot1](https://github.com/WanL0q/sensor_fusion/assets/134664967/03b16707-3d49-4247-8a56-084a938504fd)
