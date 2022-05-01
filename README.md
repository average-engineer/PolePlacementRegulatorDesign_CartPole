# PolePlacementRegulatorDesign_CartPole
Pole Placement Regulator Design of Cart Pole System

**Type of Regulator**: Full State Regulator

![sys](https://github.com/average-engineer/PolePlacementRegulatorDesign_CartPole/blob/main/Images/BothActuated.PNG)

The cart translates with respect to the inertial frame in one dimension freely and no external force is applied on the cart. The cart is assumed to be a simple lumped mass block. The pole is a uniformly distributed stick with its centre of mass at its geometric centre. It rotates with respect to the translating cart with the help of a revolute joint. The system is a lumped parameter system which can be represented by ordinary differential equations.

<h2>Dynamics of Unforced Open Loop System</h2>

Since we don't expect the pole to remain at its unstable position, we can't use small angle approximation to linearlize the system.

`[t,w] = ode45(@(t,w)OpenLoopStateSpaceCartPole(t,w,m,M,L,g),t_span,w_0)`

![sys](https://github.com/average-engineer/PolePlacementRegulatorDesign_CartPole/blob/main/Images/UnactuatedCartPole_PoleAngle.png)

![sys](https://github.com/average-engineer/PolePlacementRegulatorDesign_CartPole/blob/main/Images/UnactuatedCartPole_CartPosition.png)

*Various cases of actuation are also discussed. The influence of actuation on the controllability of the plant is also discussed. Please refer to the MATLAB Livescript file for the detailed comparitive analysis*

Based on the response time requirement, a set of desired poles are set.

`V = [-1 - 1*1i;
    -1 + 1*1i;
    -5 - 5*1i;
    -5 + 5*1i]`

The cotroller gain matrix is determined using the `place` command for the *Control System Toolbox (CST)*. The command basically computes the required controller gain matrix using the *Ackerman's Formula* and the controllability test matrix for the system based on *Controller Companion Form.*


**Comparing results obtained from MATLAB CST Toolbox and `ode45` function**:

The closed loop system is simulated using the `initial` command from CST toolbox. Then the results are compared with the simulation carried out in the traditional way (developing the state space model and then using `ode45`).

`[y,t,X] = initial(sys_CL,w_0,t_span);`

`[t,w] = ode45(@(t,w)ClosedLoopStateSpace(t,w,A,B,K),t_span,w_0);`

![sys](https://github.com/average-engineer/PolePlacementRegulatorDesign_CartPole/blob/main/Images/CSTResuls.jpg)

![sys](https://github.com/average-engineer/PolePlacementRegulatorDesign_CartPole/blob/main/Images/ode45Results.jpg)




