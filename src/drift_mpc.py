import casadi as ca
import numpy as np

class DriftMPC:
    def __init__(self):
        # 1. DriftBot's hardware specs
        self.params = {
            'm' :4.5, # mass in kg
            'Iz':  0.070, # yaw inertia in kg*m^2
            'Lf' : 0.13, # CoM to front axle
            'Lr' : 0.13, # CoM to rear axle
            'mu_f' : 0.6, # front friction coeff.
            'mu_r' : 0.4, # rear friction coeff.
            'g' : 9.81 # gravity
        }
        # Tyre stiffness(cornering stiffness)
        # Generated tyre force per degreee of slip equation
        self.params['Cf'] =  self.params['mu_f'] * self.params['m'] * self.params['g'] * (self.params['Lr'] / (self.params['Lf'] + self.params['Lr'])) * 10
        self.params['Cr'] =  self.params['mu_r'] * self.params['m'] * self.params['g'] * (self.params['Lf'] / (self.params['Lf'] + self.params['Lr'])) * 10

        # 2. MPC horizon parameters
        self.N = 20 # prediction horizon(20 steps to the future)
        self.dt = 0.05 # Time step is 50 milliseconds.

        # Therefore,
                   # total look-ahead time is N*dt = 1.0 seconds
        
        # Initialize the symbolic variables and physics model
        self._setup_symbolics()
        self._build_phsics_model()

    def _setup_symbolics(self):
        """ mathemmatical symbols that the optimizer would be allowed to change"""

        # state variables, x
        self.X = ca.SX.sym('X')   # global X POSITION
        self.Y = ca.SX.sym('Y')   # global Y posiition
        self.psi = ca.SX.sym('psi') # heading angle
        self.vx = ca.SX.sym('vx') # longitudinal velocity
        self.vy= ca.SX.sym('vy') # lateral velocity
        self.r = ca.SX.sym('r') # yaw rate

        #combine into single state vector(6x1)
        self.state = ca.vertcat(self.X, self.Y, self.psi, self.vx, self.vy, self.r)
        self.n_states = self.state.numel()

        # control variables, u
        self.delta = ca.SX.sym('delta') # steering angle
        self.a = ca.SX.sym('a') # throttle/brake input

        # combine into single control vector(2x1)
        self.control = ca.vertcat(self.delta, self.a)
        self.n_controls = self.control.numel()

    def _build_physics_model(self):
        """The dynamic bicycle model: calculates exactly how the car moves"""
        p = self.params

        # To prevent dividing by zero when the car is stopped, we add a tiny number
        vx_safe = ca.if_else(ca.fabs(self.vx) < 0.1, 0.1, self.vx)

        # 1. Calculate tyre slip angles(alpha)
        # compare where the slip is pointing to where it's traveling
        alpha_f = ca.arctan2((self.vy + p['Lf'] * self.r), vx_safe) - self.delta
        alpha_r = ca.arctan2((self.vy - p['Lr'] * self.r), vx_safe)

        # 2. calculate lateral tyre forces via a simplified linear model
        Fyf = -p['Cf'] * alpha_f
        Fyr = -p['Cf'] * alpha_r

        # 3. calculate EOMs 
        # rate of change(derivateives) of our state

        # Acceleration in global frame, X(acceleration minus drag and lateral forces)
        dot_vx = self.a - (Fyf * ca.sin(self.delta)) / p['m'] + self.vy * self.r

        # Acceleration in global frame, Y
        dot_vy = (Fyf * ca.cos(self.delta) + Fyr) / p['m'] - self.vx * self.r

        # Yaw rotation(rotational torque from front & rear tyres fighting)
        dot_r = (p['Lf'] * Fyf * ca.cos(self.delta) - p['Lr'] * Fyr) / p['Iz']

        #Global kinematic equations
        dot_X = self.vx * ca.cos(self.psi) - self.vy * ca.sin(self.psi)
        dot_Y = self.vx * ca.sin(self.psi) + self.vy * ca.cos(self.psi)
        dot_psi = self.r

        # combine all derivatives into a single vector(x_dot)
        self.state_dot = ca.vertcat(dot_X, dot_Y, dot_psi, dot_vx, dot_vy, dot_r)

        # 4. create a CasADI function for continuos dynamics: x_dot = f(x, u)
        self.f_continuos = ca.Function('f', [self.sate, self.control], [self.state_dot])

        # 5. Discrete Time Integration (Runge-Kutta 4)
        k1 = self.f_continuos(self.state, self.control)
        k2 = self.f_continuos(self.state + self.dt/2 * k1, self.control)
        k3 = self.f_continuos(self.state + self.dt/2 * k2, self.control)
        k4 = self.f_continuos(self.state + self.dt * k3, self.control)

        state_next = self.state + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)

        # final discrete function: x_{k+1} = f(x_k, u_k)
        self.f_discrete = ca.Function('F', [self.state, self.control], [state_next])

    def _setup_optimizer(self):
        """cost function & non linear solver"""
        # variables for what the optimizer is allowed to change
        # x_var for predicted states oveer the whole horizon of N+1 steps 
        self.X_var = ca.SX.sym('X_var', self.n_states, self.N + 1)

        #U_var for predicted control inputs over the horizon of N steps
        self.U_var = ca.SX.sym('U_var', self.n_controls, self.N)

        # parameters passed in at runtime 
        self.P = ca.SX.sym('P', self.n_states + self.n_states * self.N)

        # state error(X, Y, psi, vx, vy, r)
        Q = ca.diagcat([10.0, 10.0, 5.0, 1.0, 100.0, 1.0])

        # penalizing aggressive control of steering and throttle 
        R = ca.diagcat([0.1, 0.01])
        R_rate  = ca.diagcat([5.0, 0.1])

        cost = 0 # initial score of 0

        constraints = [] # starts with no physics violations
        constraints = ca.vertcat(constraints, self.X_var[:, 0] - self.P[0:self.n_states])

        for k in range(self.N):
            # extract current step's state, control and target reference
            st = self.X_var[:, k]
            con = self.U_var[:, k]
            ref_idx = self.n_states + k * self.n_states
            ref_st = self.P[ref_idx : ref_idx + self.n_states]

            #---THE COST FUNCTION---
            # penalty for deviating from track or not sliding enough
            state_error = st - ref_st
            cost += ca.mtimes([state_error.T, Q, state_error])

            # penalty for aggressive control inputs
            cost += ca.mtimes([con.T, R, con])

            # penalty for aggressive control changes (smoothness)
            if k > 0:
                con_prev = self.U_var[:, k-1]
                delta_con = con - con_prev
                cost += ca.mtimes([delta_con.T, R_rate, delta_con])

            #---PHYSICS CONSTRAINTS---
            st_next = self.X_var[:, k+1]
            st_next_predicted = self.f_discrete(st, con)
            constraints = ca.vertcat(constraints, st_next - st_next_predicted)

        OPT_variables = ca.vertcat(ca.reshape(self.X_var, -1, 1), 
                                   ca.reshape(self.U_var, -1, 1))
        
        nlp_prob = {
            'f': cost,    #minimize!
            'x': OPT_variables, #by tweaking these!
            'g': constraints, # without violating these!
            'p': self.P # given these current real-world parameters
        }

        opts = {
            'ipopt': {
                'max_iter': 100,
                'print_level': 0, # Mute terminal spam
                'acceptable_tol': 1e-4,
                'acceptable_obj_change_tol': 1e-4
            },
            'print_time': 0
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)




























        







