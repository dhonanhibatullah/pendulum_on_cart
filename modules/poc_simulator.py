import numpy as np



class PocSimulator:



    def __init__(self, pole_length:float, pole_mass:float, pole_inert:float, cart_mass:float, state_init:np.ndarray, timestep:float=0.016667) -> None:
        
        # Take arguments
        self.pole_length    = pole_length
        self.pole_mass      = pole_mass
        self.pole_inert     = pole_inert
        self.cart_mass      = cart_mass
        self.state          = state_init
        self.dt             = timestep
        self.time           = 0.0


        # Other parameters
        self.g  = 9.8
        self.k1 = (self.pole_mass*self.pole_length)/(self.cart_mass + self.pole_mass)
        self.k2 = (self.pole_mass*self.pole_length)/(self.pole_mass*(self.pole_length**2.0) + self.pole_inert)
        self.bm = 0.05 + 0.1*np.random.rand()
        self.bp = 0.05 + 0.1*np.random.rand()
        # self.bm = 0
        # self.bp = 0



    def getSimulationTime(self) -> float:
        return self.time
    


    def getCurrentState(self) -> np.ndarray:
        return self.state



    def stateEquation(self, state:np.ndarray, u:float) -> np.ndarray:
        
        # Take states
        x1  = state[0].item()
        x2  = state[1].item()
        x3  = state[2].item()
        x4  = state[3].item()
        k1  = self.k1
        k2  = self.k2
        g   = self.g
        mp  = self.pole_mass
        l   = self.pole_length
        bm  = self.bm
        bp  = self.bp


        # Return state equation
        x1_dot  = x2
        x2_dot  = (k1/(1.0 - k1*k2*(np.cos(x3)**2.0)))*(0.5*k2*g*np.sin(2.0*x3) + np.sin(x3)*(x4**2.0) - u/(mp*l) - bm*x2/(mp*l))
        x3_dot  = x4
        x4_dot  = (k2/(1.0 - k1*k2*(np.cos(x3)**2.0)))*(-0.5*k1*np.sin(2.0*x3)*(x4**2.0) - g*np.sin(x3) + (k1*np.cos(x3)*u)/(mp*l) - bp*x4)
        return np.array([
            [x1_dot],
            [x2_dot],
            [x3_dot],
            [x4_dot]
        ])



    def stepSimulation(self, input:float) -> None:
        
        # Calculate next iteration with RK4
        rk1         = self.stateEquation(self.state, input)
        rk2         = self.stateEquation(self.state + rk1*self.dt/2.0, input)
        rk3         = self.stateEquation(self.state + rk2*self.dt/2.0, input)
        rk4         = self.stateEquation(self.state + rk3*self.dt, input)
        self.state  = self.state + self.dt*(rk1 + 2.0*rk2 + 2.0*rk3 + rk4)/6.0
        self.time   += self.dt