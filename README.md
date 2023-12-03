# Tracker Arm Robot
To run the script, make sure you have installed the following dependencies:
```
pip install numpy
pip install matplotlib
pip install pygame
```

## Short Introduction
The dynamics of an armature linkage can be written generally as

$$\tau = M(\theta)\ddot{\theta} + C(\theta, \dot{\theta})\dot{\theta} + g(\theta)$$

We can rewrite the equation as follows to simulate the dynamics.

$$\ddot{\theta} = M^{-1}(\theta)\left( \tau - C(\theta, \dot{\theta})\dot{\theta} - g(\theta) \right)$$

To make the dynamics less ideal, let us add some viscous frictions on each joint with a positive semi-definite matrix $b$ such that:

$$\ddot{\theta} = M^{-1}(\theta)\left( \tau - \left(C(\theta, \dot{\theta}) + b\right)\dot{\theta} - g(\theta) \right)$$

The control law used for $\tau$ is

$$F = \ddot{x}_d(t) + K_px_e(t) + K_i\int_0^tx_e(t)dt$$

$$\tau = \tilde{M}(\theta)\left(J^TF + K_d\left(J^{-1}\dot{x}_d - \dot{\theta} \right) \right) + \tilde{h}(\theta, \dot{\theta})$$

You can tune $K_p$, $K_i$, and $K_d$ values in tar_main.py.