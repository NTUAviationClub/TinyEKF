# TinyEKF
EKF for Arduino with 6/9 axis IMU

## Usage
Please see ./example

## State Space Equation
### ekf(10)
#### Transition Model
![equation](https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;\omega_x\\&space;\omega_y\\&space;\omega_z\\&space;b_x\\&space;b_y\\&space;b_z\\&space;q_w\\&space;q_x\\&space;q_y\\&space;q_z\end{bmatrix}_{k&plus;1}&space;=&space;I_{10*10}&space;&plus;&space;\Delta&space;T&space;\begin{bmatrix}&space;0\\&space;0\\&space;0\\&space;0\\&space;0\\&space;0\\&space;0.5(-q_x\omega_x-q_y\omega_y-q_z\omega_z)\\&space;0.5(q_w\omega_x-q_y\omega_z&plus;q_z\omega_y)\\&space;0.5(q_w\omega_y&plus;q_x\omega_z-q_z\omega_x)\\&space;0.5(q_w\omega_z-q_x\omega_y&plus;q_y\omega_x)\end{bmatrix})
#### Observation Model
![equation](https://latex.codecogs.com/svg.image?y=\begin{bmatrix}&space;\omega_x\\&space;\omega_y\\&space;\omega_z\\&space;q_w\\&space;q_x\\&space;q_y\\&space;q_z\end{bmatrix})

### ekf7
#### Transition Model
![equation](https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;b_x\\&space;b_y\\&space;b_z\\&space;q_w\\&space;q_x\\&space;q_y\\&space;q_z\end{bmatrix}_{k&plus;1}&space;=&space;I_{10*10}&space;&plus;&space;\Delta&space;T&space;\begin{bmatrix}&space;0\\&space;0\\&space;0\\&space;0.5(-q_x(\omega_x-b_x)-q_y(\omega_y-b_y)-q_z(\omega_z-b_z))\\&space;0.5(q_w(\omega_x-b_x)-q_y(\omega_z-b_z)&plus;q_z(\omega_y-b_y))\\&space;0.5(q_w(\omega_y-b_y)&plus;q_x(\omega_z-b_z)-q_z(\omega_x-b_x))\\&space;0.5(q_w(\omega_z-b_z)-q_x(\omega_y-b_y)&plus;q_y(\omega_x-b_x))\end{bmatrix})
#### Observation Model
![equation](https://latex.codecogs.com/svg.image?y=\begin{bmatrix}&space;q_w\\&space;q_x\\&space;q_y\\&space;q_z\end{bmatrix})

## References
[1] "An Extended Kalman Filter for Quaternion-Based Orientation Estimation Using MARG Sensors", JoZo Luis Marins et.el., 2001

## License
MIT License

Copyright (c) 2022 TieneSabor

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
