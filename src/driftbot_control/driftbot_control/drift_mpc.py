import casadi as ca
import numpy as np

class DriftMPC:
    def __init__(self):
        self.params = {
            'm': 4.5, 'Iz': 0.070, 'Lf': 0.13, 'Lr': 0.13,
            'mu_f': 1.0, 'mu_r': 0.3, 'g': 9.81,  # Matched to SDF!
            'Cf': 60.0, 'Cr': 40.0
        }
        self.N = 8  # Extended slightly for smoother high-speed planning
        self.dt = 0.05
        self._setup_symbolics()
        self._build_physics_model()
        self._setup_optimizer()

    def _setup_symbolics(self):
        self.X = ca.SX.sym('X'); self.Y = ca.SX.sym('Y'); self.psi = ca.SX.sym('psi')
        self.vx = ca.SX.sym('vx'); self.vy = ca.SX.sym('vy'); self.r = ca.SX.sym('r')
        self.state = ca.vertcat(self.X, self.Y, self.psi, self.vx, self.vy, self.r)
        self.n_states = 6

        self.delta = ca.SX.sym('delta'); self.a = ca.SX.sym('a')
        self.control = ca.vertcat(self.delta, self.a)
        self.n_controls = 2

    def _build_physics_model(self):
        p = self.params
        vx_safe = ca.if_else(ca.fabs(self.vx) < 0.5, 0.5, self.vx)

        alpha_f = ca.atan2((self.vy + p['Lf'] * self.r), vx_safe) - self.delta
        alpha_r = ca.atan2((self.vy - p['Lr'] * self.r), vx_safe)

        Fzf = p['m'] * p['g'] * 0.5
        Fzr = p['m'] * p['g'] * 0.5

        # Differentiable Tire Model
        Fyf = p['mu_f'] * Fzf * ca.tanh((-p['Cf'] * alpha_f) / (p['mu_f'] * Fzf))
        Fyr = p['mu_r'] * Fzr * ca.tanh((-p['Cr'] * alpha_r) / (p['mu_r'] * Fzr))

        dot_vx = self.a - (Fyf * ca.sin(self.delta)) / p['m'] + self.vy * self.r
        dot_vy = (Fyf * ca.cos(self.delta) + Fyr) / p['m'] - self.vx * self.r
        dot_r = (p['Lf'] * Fyf * ca.cos(self.delta) - p['Lr'] * Fyr) / p['Iz']

        dot_X = self.vx * ca.cos(self.psi) - self.vy * ca.sin(self.psi)
        dot_Y = self.vx * ca.sin(self.psi) + self.vy * ca.cos(self.psi)
        dot_psi = self.r

        self.state_dot = ca.vertcat(dot_X, dot_Y, dot_psi, dot_vx, dot_vy, dot_r)
        f = ca.Function('f', [self.state, self.control], [self.state_dot])

        k1 = f(self.state, self.control)
        k2 = f(self.state + self.dt/2 * k1, self.control)
        k3 = f(self.state + self.dt/2 * k2, self.control)
        k4 = f(self.state + self.dt * k3, self.control)
        
        self.f_discrete = ca.Function('F', [self.state, self.control], [self.state + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)])

    def _setup_optimizer(self):
        self.X_var = ca.SX.sym('X_var', 6, self.N + 1)
        self.U_var = ca.SX.sym('U_var', 2, self.N)
        self.P = ca.SX.sym('P', 6 + 6*self.N)

        # Balanced Q Matrix: Respect the track (X,Y=10), but prioritize slip (vy=10)
        Q = ca.diag([10.0, 10.0, 20.0, 5.0, 10.0, 10.0])   
        R = ca.diag([1.0, 0.1])
        R_rate = ca.diag([5.0, 1.0])

        cost = 0
        g = [self.X_var[:, 0] - self.P[0:6]]

        for k in range(self.N):
            st = self.X_var[:, k]; con = self.U_var[:, k]; ref = self.P[6 + 6*k : 6 + 6*(k+1)]
            err = st - ref
            cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([con.T, R, con])
            if k > 0:
                du = con - self.U_var[:, k-1]
                cost += ca.mtimes([du.T, R_rate, du])
            g.append(self.X_var[:, k+1] - self.f_discrete(st, con))

        OPT = ca.vertcat(ca.reshape(self.X_var, -1, 1), ca.reshape(self.U_var, -1, 1))
        self.solver = ca.nlpsol('solver', 'ipopt', {'f': cost, 'x': OPT, 'g': ca.vertcat(*g), 'p': self.P}, 
                                {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.max_iter': 40})