# Simple Pendulum Animation
This project was an investigation both into the dynamics of a single pendulum but also the inefficiencies of numerical methods to solve the equation of motion.

Written by Andrew Dawson
## Contents
1. The Physics of the Pendulum
2.  Expanation of the Code
3.  Euler Method vs the Euler-Cromer Method

## The Physics of the Pendulum
A simple pendulum consists of a mass attatched to a massless rod at one end and the rod to be attatched to some fixed pivot point at the other end. The pendulum is to given an intial displacement and then left alone to swing freely. The diagram below shows the arrangement of a simple pendulum.


The Equation of Motion of a simple pendulum is given by this second order differential equation:
$$
\frac{d^2\theta}{dt^2} = -\frac{g}{l}sin(\theta)
$$

The ODE in this form is not solvable analytical. As a result, has to be solved numerically using a specific numerical technique. Note: If a restriction is added the amplitute of the oscillations the ODE becomes solvable analytically but will only express the dynamics of the pendulum for small oscillations.^[1]

For this project, the Euler-Cromer numerical method will be used to find an approximate solution for the ODE. Details on how this method works is out of the scope of this project but I will redirect you to another project of mine which covers the finer details. To apply the Euler-Cromer method to the simple pendulum ODE we have to split the ODE into to two first order differential equations. These are given as:
$$
\frac{d\omega}{dt} = -\frac{g}{l}sin(\theta) 
\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:
\frac{d\theta}{dt} = \omega
$$

Hence, we can now use the Euler-Cromer method, which results in the following two equations^[2]:
$$
\omega_{i+1} = \omega_{i}-\frac{g}{l}sin(\theta_{i})\Delta t
\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:\: \: \:
\theta_{i+1} = \theta_{i}+\omega_{i+1}\Delta t
$$

## Explanation of the Code


Firstly, the required packages need to be imported. PIL.Image and PIL.ImageDraw packages will be used to create the final animation of the pendulums. 
```python
from PIL import Image,ImageDraw
import matplotlib.pyplot as plt
import numpy as np
```
The next step, is to assign values to all the constants associated with the pendulum's dynamics such as the length of the string, the mass of the bob and also the number of iterations and dt which will be used in the Euler-Cromer method for calculting the actual trajectory of the pendulum.

```python
dt = 0.05
m = 3
l = 1
g = 9.8
width = 100
height = 100

number_Of_Iterations = 1000
```
Futhermore, we need to assign some initial conditions to the pendulum. Hence, we are going to assume the pendulum starts at an initial angle of pi/3 and has an initial angular velcocity of zero.
```python
theta_initial = (np.pi/3)
theta_deriv_initial = 0
```

## Euler Method vs Euler-Cromer Method
This section will briefly cover why the Euler-Cromer Method was used to solve the equation of motion for the pendulum rather than using the Euler Method. The details of each method is out of the scope of this project, however, if you are wanting to learn more about each numerical method, a link to some great learning resources will be linked at the bottom of this page.

The following code is an example of a method calculating the trajectory of the pendulum using Euler's Method
```python
def calcuateTrajectoryEulerMethod(theta_initial,theta_deriv_initial,m,l,g,dt,number_Of_Iterations):
    trajectory = np.ones([number_Of_Iterations,2])
    # Set the initial conditions
    
    trajectory[0,0] = theta_initial
    trajectory[0,1] = theta_deriv_initial  
    
    for i in range(0,number_Of_Iterations-1):
        # Calculation of the theta points
        trajectory[i+1,0] = trajectory[i,0] + trajectory[i,1]*dt
        # Calculation of the angluar velocity
        trajectory[i+1,1] = trajectory[i,1] - (g/l)*np.sin(trajectory[i,0])*dt                      
    return trajectory
```

Now if we plot the the angle against iteration number we get the following output:

[INSERT DIAGRAM]

As the iteration number increases the amplitude of the pendulum begins to increase. It is obvious to see that this isn't the actual dynamics of a pendulum but must be an error in the numerical method.  Decreasing the time-step (dt) in the code would improve the solution however, no matter how small the time step becomes there will always be an increasing amplitude as the number of iterations increases.

The graph below shows how varying the time steps can improve our solution. However, for the orange graph the amplitudes are still slowly increasing still even though the time step has been halved.

[INSERT DIAGRAM]

A quick reason for this increase of amplitude is due to the error in the computed solution which is given by O(dt). As the time step increases the error in the solution will increase proportionally. Hence if you wanted your solution to be 200 times more accurate you would have to take 200 more time steps to achieve this. e.g instead of taking 5 steps per second you would have to take 1000 steps per second.

As a result, the Euler-Cromer method is ideal for this type of problem as it is much more accurate. This method conserves the energy in our problem which as a result means we maintain our constant amplitudes. The code and graph below calculates and displays the total energy of pendulum against the iteration number.

[Insert Diagram]

For a real pendulum the total energy should be constant with iteration number. As you can see the Euler Method shows the energy increasing with iteration number is is nonsense. So again, the Euler-Cromer Method is a much better numerical method to use in this simulation.

## References
[1] - https://en.wikipedia.org/wiki/Pendulum_(mathematics)
[2] - http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node2.htm
