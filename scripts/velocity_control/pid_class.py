class PID:
    def __init__(self,kp,ki,kd,tau,maxDot,conditionalIntegratorThreshold):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.tau = tau
        self.maxDot = maxDot
        self.conditionalIntegratorThreshold = conditionalIntegratorThreshold

        self.integrator = 0.0
        self.derivative = 0.0

        self.prev_state = 0.0
        self.prev_error = 0.0
        self.command = 0.0

    def update_control(self,state,desired,dt,integrators_on):
        if dt < 0.0001:
            return
        error = desired-state
        if self.ki != 0.0 and integrators_on == True:
            self.update_integrator(error,dt)
        if self.kd != 0.0:
            self.update_derivative(state,dt)
        self.update_command(error)
        self.prev_state = state

    def update_integrator(self,error,dt):
        eDot = (error - self.prev_error)/dt
        if error > 0.0 and eDot > -self.conditionalIntegratorThreshold:
            self.integrator = self.integrator+error*dt
        elif error < 0.0 and eDot < self.conditionalIntegratorThreshold:
            self.integrator = self.integrator+error*dt
        self.prev_error = error

    def update_derivative(self,state,dt):
        dState = state - self.prev_state
        self.derivative = (2.0*self.tau-dt)*self.derivative + (2.0*dState)/(2.0*self.tau + dt)

    def update_command(self,error):
        pTerm = self.kp*error
        iTerm = self.ki*self.integrator
        dTerm = self.kd*self.derivative
        u = pTerm+iTerm-dTerm
        u_sat = self.saturate(u)
        self.compute_anti_windup(u,u_sat,pTerm,iTerm,dTerm)
        self.command = u_sat

    def saturate(self,u):
        if u > self.maxDot:
            u_sat = self.maxDot
        elif u < -self.maxDot:
            u_sat = -self.maxDot
        else:
            u_sat = u
        return u_sat

    def compute_anti_windup(self,u,u_sat,pTerm,iTerm,dTerm):
        if (u != u_sat and abs(iTerm)>abs(u_sat-pTerm+dTerm)):
            self.integrator = (u_sat-pTerm+dTerm)/self.ki
