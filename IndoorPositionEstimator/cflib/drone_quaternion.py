"""
Quadcopter Model + LQR Control + BackStepping Control
"""
#
# Author: Alberto Petrucci (petrucci.alberto@gmail.com) 2017
#
#__author__ = "Alberto Petrucci"
#__copyright__ = "Copyright 2017, Alberto Petrucci"
#__credits__ = ["Alberto Petrucci"]
#__license__ = "MIT"
#__version__ = "1.0.0"
#__maintainer__ = "Alberto Petrucci"
#__email__ = "petrucci.alberto@.com"
#__status__ = "Production"
from __future__ import division
from numpy import *
from math import *
from control import *

class Quadcopter:

    def __init__(self, dt):
        ## Parametri ambiente
        self.g = 9.81
        self.airFriction = 0
        self.dt = dt

        ## Parametri drone
        self.m = 27/1000       # massa del drone in g
        self.d = (65.0538/1000)*sin(pi/4)  # distanza dal centro ai motori
        self.c = 0.1           # inerzia delle eliche
        self.alpha = 1
        self.Ix = self.m * self.d * self.d
        self.Iy = self.m * self.d * self.d
        self.Iz = 2 * self.m * self.d * self.d

        # Cambiando tali parametri diamo priorita maggiori o minori
        self.beta1 = 0.3
        self.beta2 = 0.3
        self.beta3x = 0.2#1.0
        self.beta3y = 0.2#1.0
        self.beta3z = 0.2#0.5
        self.beta3x = 5.0#5.0
        self.beta3y = 5.0#5.0
        self.beta3z = 1.0#1.0
        self.beta4 = 0.2
        self.beta = 500#3000
        self.thrustGain = 1.34
        #self.thrustGain = 1.37
        self.Tf = dt
        self.Mat_J = matrix([
            [self.m*self.d*self.d, 0, 0],
            [0, self.m*self.d*self.d, 0],
            [0, 0, 2*self.m*self.d*self.d]
        ])
        self.Mat_Jinv = self.Mat_J.I
        self.Mat_T = matrix([
            [1, 1, 1, 1],
            [-self.d,  -self.d, self.d, self.d],
            [self.d, -self.d, -self.d, self.d],
            [self.c, -self.c, self.c, -self.c]
        ])
        self.Mat_Tinv = self.Mat_T.I

        ## Modello linearizzato
        self.A = matrix([
            [0, 0, 0, 0, 0, 0, -0.5*sqrt(1-self.alpha*self.alpha), 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0.5*self.alpha, -0.5*sqrt(1-self.alpha*self.alpha), 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0.5*sqrt(1-self.alpha*self.alpha), 0.5*self.alpha, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0.5*self.alpha, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, 2*self.g*sqrt(1-self.alpha*self.alpha), 2*self.g*self.alpha, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, -2*self.g*self.alpha, 2*self.g*sqrt(1-self.alpha*self.alpha), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ])
        self.B = matrix([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],

            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],

            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/self.m, 0, 0, 0],
        ])
        self.C = eye(13)
        self.D = zeros((13, 4))

        ## SATURAZIONE MOTORI
        self.fmotmax = 0.5886/4  # max forza generata dai motori
        self.q_bar = matrix([
            [self.alpha],
            [0],
            [0],
            [sqrt(1 - self.alpha*self.alpha)]
        ])
        self.omega_bar = zeros((3, 1))
        self.p_bar = matrix([
            [0],
            [0],
            [1]
        ])
        self.v_bar = zeros((3, 1))
        self.ftot_bar = self.m * self.g
        self.tau_bar = matrix([
            [0],
            [0],
            [0]
        ])
        self.x_bar = vstack((self.q_bar, self.omega_bar, self.p_bar, self.v_bar))
        self.u_bar = vstack((self.ftot_bar, self.tau_bar))
        self.u = matrix([
            [0],
            [0],
            [0],
            [0]
        ])
        self.Qm = matrix([
            [self.beta1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, self.beta1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, self.beta1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, self.beta1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, self.beta2, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, self.beta2, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, self.beta2, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, self.beta3x, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, self.beta3y, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, self.beta3z, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self.beta4, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self.beta4, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self.beta4],
        ])
        self.R = self.beta * eye(4)

        ## LQR
        self.Amm = matrix([
            [0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0],
            [0, 19.62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [-19.62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])
        self.Bmm = matrix([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 8.7393e+03, 0, 0],
            [0, 0, 8.7393e+03, 0],
            [0, 0, 0, 4.3696e+03],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [37.0370, 0, 0, 0]
        ])
        self.Cmm = matrix([
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
        ])
        self.Qmm = matrix([
            [self.beta1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, self.beta1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, self.beta1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, self.beta2, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, self.beta2, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, self.beta2, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, self.beta3x, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, self.beta3y, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, self.beta3z, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, self.beta4, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self.beta4, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self.beta4]
        ])
        self.Ut = matrix([
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
        ])
        [self.Km, self.Pm, self.em] = lqr(self.Amm, self.Bmm, self.Qmm, self.R)
        self.K_LQR = self.Km*self.Ut
        '''
        # stampo guadagni lqr per c
        for k in range(0, 4):
            string = ""
            for i in range(0, 13):
                string += str(self.K_LQR.item((k, i)))+", "
            # rimuovo gli ultimi due caratteri
            string = string[:-2]
            print "{"+string+"},"
        '''

        # Stato
        self.q = matrix([
            [self.alpha], [0], [0], [sqrt(1-self.alpha*self.alpha)]
        ])
        self.omega = matrix([
            [0], [0], [0]
        ])
        self.p = matrix([
            [0], [0], [0]
        ])
        self.v = matrix([
            [0], [0], [0]
        ])
        self.x = vstack((
            self.q,
            self.omega,
            self.p,
            self.v
        ))
        self.setPoint = self.x

        # Variabili per l'osservatore (ricostruzione stato)
        self.x_hat = self.x
        # variabili misurate (quaternioni + posizioni)
        self.y = matrix([
            [0], [0], [0], [0], [0], [0], [0]
        ])

        # Variabili per BackStepping controller
        self.x_levante = matrix([[0], [0], [0]])
        self.y_levante = matrix([[0], [0], [0]])
        self.z_levante = matrix([[0], [0], [0]])
        self.roll_levante = matrix([[0], [0], [0]])
        self.pitch_levante = matrix([[0], [0], [0]])
        self.yaw_levante = matrix([[0], [0], [0]])

    def setSetPoint(self, q0, q1, q2, q3, omegax, omegay, omegaz, px, py, pz, vx, vy, vz):
        self.setPoint = matrix([
            [q0],
            [q1],
            [q2],
            [q3],
            [omegax],
            [omegay],
            [omegaz],
            [px],
            [py],
            [pz],
            [vx],
            [vy],
            [vz],
        ])

    def setState(self, q0, q1, q2, q3, omegax, omegay, omegaz, px, py, pz, vx, vy, vz):
        self.q = matrix([
            [q0], [q1], [q2], [q3]
        ])
        self.q = self.q/linalg.norm(self.q)
        deg2rad = pi/180.0
        self.omega = matrix([
            [omegax*deg2rad], [omegay*deg2rad], [omegaz*deg2rad]
        ])
        self.p = matrix([
            [px], [py], [pz]
        ])
        self.v = matrix([
            [vx], [vy], [vz]
        ])

        '''
        # Aggiorno variabili misurate
        self.y = matrix([
            [q0], [q1], [q2], [q3], [px], [py], [pz]
        ])

        # Aggiorno l'osservatore
        self.update_observer()

        # Aggiorno lo stato (misurato + stimato)
        self.x = vstack((
            self.q,
            [self.x_hat[4, 0]*deg2rad],
            [self.x_hat[5, 0]*deg2rad],
            [self.x_hat[6, 0]*deg2rad],
            self.p,
            [self.x_hat[10, 0]],
            [self.x_hat[11, 0]],
            [self.x_hat[12, 0]]
        ))
        '''
        # Nel caso in cui misuro tutto (e' lento => stimo)
        self.x = vstack((
            self.q,
            self.omega,
            self.p,
            self.v
        ))

    def update(self):
        self.u = self.u_bar - self.K_LQR * (self.x - self.setPoint)

        # Calcolo le forze f1 f2 f3 f4
        f = self.Mat_Tinv*self.u

        # Applico la saturazione
        for i in range(0, 4):
            if f[i, 0] > self.fmotmax:
                f[i, 0] = self.fmotmax
            if f[i, 0] < 0:
                f[i, 0] = 0

        # Calcolo l'ingresso saturato
        self.u = self.Mat_T*f
        #self.predict(self.u)

    def backstepping(self, yaw_desired, x_desired, y_desired, z_desired):

        roll, pitch, yaw = self.quaternion2RPY()
        # Euler Angles
        x1 = yaw    # yaw
        x2 = pitch  # pitch
        x3 = roll   # roll

        # Angular Speeds
        x4 = self.omega[0, 0]  # wx
        x5 = self.omega[1, 0]  # wy
        x6 = self.omega[2, 0]  # wz

        # Positions
        x7 = self.p[0, 0]  # x
        x8 = self.p[1, 0]  # y
        x9 = self.p[2, 0]  # z

        # Speeds
        x10 = self.v[0, 0]  # vx
        x11 = self.v[1, 0]  # vy
        x12 = self.v[2, 0]  # vz

        # Controllers Paramters
        c1 = 6.11
        c3 = 6.11
        c5 = 6.11
        c7 = 6.11
        c2 = 7.96
        c4 = 7.96
        c6 = 7.96
        c8 = 7.96

        # Levante Derivator Gains
        L1 = -10
        L2 = -60

        # Creo il riferimento di roll e pitch per controllare la x e la y
        error = self.x_levante[0, 0] - x_desired
        self.x_levante[0, 0] = self.x_levante[0, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.x_levante[1, 0]
        self.x_levante[1, 0] = self.x_levante[1, 0] + L2 * self.dt * sign(error)

        error = self.x_levante[2, 0] - self.x_levante[1, 0]
        self.x_levante[2, 0] = self.x_levante[2, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.x_levante[3, 0]
        self.x_levante[3, 0] = self.x_levante[3, 0] + L2 * self.dt * sign(error)

        error = self.y_levante[0, 0] - y_desired
        self.y_levante[0, 0] = self.y_levante[0, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.y_levante[1, 0]
        self.y_levante[1, 0] = self.y_levante[1, 0] + L2 * self.dt * sign(error)

        error = self.y_levante[2, 0] - self.y_levante[1, 0]
        self.y_levante[2, 0] = self.y_levante[2, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.y_levante[3, 0]
        self.y_levante[3, 0] = self.y_levante[3, 0] + L2 * self.dt * sign(error)


        k11 = 155
        k12 = 25
        pd = -self.m * (self.x_levante[3, 0] + k11 * (self.x_levante[1, 0] - x10) + k12 * (self.x_levante[0, 0] - x7)) / self.u[0, 0] + self.pitch_levante[0, 0]
        rd = -self.m * (self.y_levante[3, 0] + k11 * (self.y_levante[1, 0] - x11) + k12 * (self.y_levante[0, 0] - x8)) / self.u[0, 0] + self.roll_levante[0, 0]
        pd = pd * self.dt
        rd = rd * self.dt

        # Condizioni di piccolo angolo: roll e pitch compresi tra +/-20deg
        margin = 20 * math.pi / 180
        if rd > margin:
            rd = margin
        if rd < -margin:
            rd = -margin
        if pd > margin:
            pd = margin
        if pd < -margin:
            pd = -margin

        # Altitude Controller
        # Stimo Zd_dot_dot e Zd_dot(con levante)
        error = self.z_levante[0, 0] - z_desired
        self.z_levante[0, 0] = self.z_levante[0, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.z_levante[1, 0]
        self.z_levante[1, 0] = self.z_levante[1, 0] + L2 * self.dt * sign(error)

        error = self.z_levante[2, 0] - self.z_levante[1, 0]
        self.z_levante[2, 0] = self.z_levante[2, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.z_levante[3, 0]
        self.z_levante[3, 0] = self.z_levante[3, 0] + L2 * self.dt * sign(error)

        Ez = z_desired - x9
        U1 = self.m * (-Ez + self.g - self.z_levante[3, 0] - c7 * self.z_levante[1, 0] + c7 * x12 + c8 * (x12 - self.z_levante[1, 0] - c7 * Ez)) / (cos(x2) * cos(x3));

        # Roll Controller
        error = self.roll_levante[0, 0] - rd
        self.roll_levante[0, 0] = self.roll_levante[0, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.roll_levante[1, 0]
        self.roll_levante[1, 0] = self.roll_levante[1, 0] + L2 * self.dt * sign(error)

        error = self.roll_levante[2, 0] - self.roll_levante[1, 0]
        self.roll_levante[2, 0] = self.roll_levante[2, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.roll_levante[3, 0]
        self.roll_levante[3, 0] = self.roll_levante[3, 0] + L2 * self.dt * sign(error)

        z1 = rd - x3
        z2 = x4 - self.roll_levante[1, 0] - c1 * z1
        b1 = 1 / self.Ix; # d / self.Ix
        a1 = (self.Iy - self.Iz) / self.Ix
        U2 = (-c2 * z2 + z1 - x5 * x6 * a1 + self.roll_levante[3, 0] + c1 * self.roll_levante[1, 0] - c1 * x4) / b1

        # Pitch Controller
        error = self.pitch_levante[0, 0] - pd
        self.pitch_levante[0, 0] = self.pitch_levante[0, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.pitch_levante[1, 0]
        self.pitch_levante[1, 0] = self.pitch_levante[1, 0] + L2 * self.dt * sign(error)

        error = self.pitch_levante[2, 0] - self.pitch_levante[1, 0]
        self.pitch_levante[2, 0] = self.pitch_levante[2, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.pitch_levante[3, 0]
        self.pitch_levante[3, 0] = self.pitch_levante[3, 0] + L2 * self.dt * sign(error)

        z3 = pd - x2
        z4 = x5 - self.pitch_levante[1, 0] - c3 * z3
        b2 = 1 / self.Iy; # d / Ix
        a3 = (self.Iz - self.Ix) / self.Iy
        U3 = (-c4 * z4 + z3 - x4 * x6 * a3 + self.pitch_levante[3, 0] + c3 * self.pitch_levante[1, 0] - c3 * x5) / b2

        # Yaw Controller
        error = self.yaw_levante[0, 0] - yaw_desired
        self.yaw_levante[0, 0] = self.yaw_levante[0, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.yaw_levante[1, 0]
        self.yaw_levante[1, 0] = self.yaw_levante[1, 0] + L2 * self.dt * sqrt(abs(error)) * sign(error)

        error = self.yaw_levante[2, 0] - self.yaw_levante[1, 0]
        self.yaw_levante[2, 0] = self.yaw_levante[2, 0] + L1 * self.dt * sqrt(abs(error)) * sign(error) + self.dt * self.yaw_levante[3, 0]
        self.yaw_levante[3, 0] = self.yaw_levante[3, 0] + L2 * self.dt * sqrt(abs(error)) * sign(error)

        z5 = yaw_desired - x1
        z6 = x6 - self.yaw_levante[1, 0] - c5 * z5
        b3 = 1 / self.Iz; # d / Ix
        a5 = (self.Ix - self.Iy) / self.Iz
        U4 = (-c6 * z6 + z5 - x4 * x5 * a5 + self.yaw_levante[3, 0] + c5 * self.yaw_levante[1, 0] - c5 * x6) / b3

        self.u = matrix([
            [U1],
            [U2],
            [U3],
            [U4]
        ])

    def update_observer(self):
        x_hat_dot = self.observer_function(self.x_hat)

        # Eulero
        # self.x_hat = self.x_hat + x_hat_dot*self.dt

        # Runge Kutta 4
        m1 = x_hat_dot
        k1 = self.x_hat + m1 * self.dt

        m2 = self.observer_function(k1)
        k2 = self.x_hat + (m1 + m2) * self.dt / 4

        m3 = self.observer_function(k2)
        self.x_hat = self.x_hat + (m1 + m2 + 4 * m3) * (self.dt / 6)

    def observer_function(self, x_hat):
        x1 = x_hat[0, 0]
        x2 = x_hat[1, 0]
        x3 = x_hat[2, 0]
        x4 = x_hat[3, 0]
        x5 = x_hat[4, 0]
        x6 = x_hat[5, 0]
        x7 = x_hat[6, 0]
        x8 = x_hat[7, 0]
        x9 = x_hat[8, 0]
        x10 = x_hat[9, 0]
        x11 = x_hat[10, 0]
        x12 = x_hat[11, 0]
        x13 = x_hat[12, 0]

        # Funzione stato
        F = matrix([
            [-(x2 * x5 + x3 * x6 + x4 * x7) / 2],
            [(x1 * x5 - x4 * x6 + x3 * x7) / 2],
            [(x4 * x5 + x1 * x6 - x2 * x7) / 2],
            [(-x3 * x5 + x2 * x6 + x1 * x7) / 2],
            [-x6 * x7],
            [x5 * x7],
            [0],
            [x11],
            [x12],
            [x13],
            [0],
            [0],
            [-self.g]
        ])

        # Funzione ingressi
        G = matrix([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 1/(self.m*self.d*self.d), 0, 0],
            [0, 0, 1/(self.m*self.d*self.d), 0],
            [0, 0, 0, 1/(2*self.m*self.d*self.d)],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [(2*x2*x4+2*x1*x3)/self.m, 0, 0, 0],
            [(2*x3*x4-2*x1*x2)/self.m, 0, 0, 0],
            [(x1*x1-x2*x2-x3*x3+x4*x4)/self.m, 0, 0, 0],
        ])

        # Funzione misure
        H = matrix([
            [x1],
            [x2],
            [x3],
            [x4],
            [x8],
            [x9],
            [x10]
        ])

        # Inversa di Q
        Qinv = matrix([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0,1,0,0,0,0,0, 0, 0, 0, 0, 0, 0],
            [0,0,0,1,0,0,0, 0, 0, 0, 0, 0, 0],
            [0,0,0,0,0,1,0, 0, 0, 0, 0, 0, 0],
            [-(x1*x1*x5 + x2*x2*x5 - x1*x3*x7 + x1*x4*x6 + x2*x3*x6 + x2*x4*x7)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), (x1*x3*x6 + x1*x4*x7 + x2*x3*x7 - x2*x4*x6)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),    (2*(x1*x1 + x2*x2))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),  -(x7*x1*x1 + x3*x5*x1 + x7*x2*x2 - x4*x5*x2)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),  (2*(x1*x4 + x2*x3))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),  (18*x1*x3 - 18*x2*x4 + x1*x1*x6 + x2*x2*x6 - x1*x4*x5 - x2*x3*x5)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), -(2*(x1*x3 - x2*x4))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), 0, 0, 0, 0, 0, 0],
            [-(x1*x1*x6 + x3*x3*x6 + x1*x2*x7 - x1*x4*x5 + x2*x3*x5 + x3*x4*x7)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),   (x7*x1*x1 - x2*x6*x1 + x7*x3*x3 - x4*x6*x3)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), -(2*(x1*x4 - x2*x3))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), (x1*x2*x5 + x1*x4*x7 - x2*x3*x7 + x3*x4*x5)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),    (2*(x1*x1 + x3*x3))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), -(18*x1*x2 + 18*x3*x4 + x1*x1*x5 + x3*x3*x5 + x1*x4*x6 - x2*x3*x6)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),  (2*(x1*x2 + x3*x4))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), 0, 0, 0, 0, 0, 0],
            [-(x1*x1*x7 + x4*x4*x7 - x1*x2*x6 + x1*x3*x5 + x2*x4*x5 + x3*x4*x6)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),  -(x6*x1*x1 + x2*x7*x1 + x6*x4*x4 - x3*x7*x4)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),  (2*(x1*x3 + x2*x4))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),   (x5*x1*x1 - x3*x7*x1 + x5*x4*x4 - x2*x7*x4)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), -(2*(x1*x2 - x3*x4))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), -(18*x1*x1 + 18*x4*x4 - x1*x2*x5 - x1*x3*x6 - x2*x4*x6 + x3*x4*x5)/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4),    (2*(x1*x1 + x4*x4))/(x1*x1*x1 + x1*x2*x2 + x1*x3*x3 + x1*x4*x4), 0, 0, 0, 0, 0, 0],
            [0,0,0,0,0,0,0, 1, 0, 0, 0, 0, 0],
            [0,0,0,0,0,0,0, 0, 0, 1, 0, 0, 0],
            [0,0,0,0,0,0,0, 0, 0, 0, 0, 1, 0],
            [0,0,0,0,0,0,0, 0, 1, 0, 0, 0, 0],
            [0,0,0,0,0,0,0, 0, 0, 0, 1, 0, 0],
            [0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 1]
        ])
        
        # Guadagni per la convergenza
        K = matrix([
            [100, 0, 0, 0, 0, 0, 0],
            [0, 100, 0, 0, 0, 0, 0],
            [0, 500, 0, 0, 0, 0, 0],
            [0, 0, 100, 0, 0, 0, 0],
            [0, 0, 500, 0, 0, 0, 0],
            [0, 0, 0, 100, 0, 0, 0],
            [0, 0, 0, 500, 0, 0, 0],
            [0, 0, 0, 0, 100, 0, 0],
            [0, 0, 0, 0, 10000, 0, 0],
            [0, 0, 0, 0, 0, 100, 0],
            [0, 0, 0, 0, 0, 10000, 0],
            [0, 0, 0, 0, 0, 0, 100],
            [0, 0, 0, 0, 0, 0, 10000]
        ])

        # Aggiorno lo stato predetto
        x_hat_dot = F + G*self.u + Qinv*K*(self.y - H)
        return x_hat_dot

    def predict(self, u):
        # Faccio evolvere il sistema
        F_b = matrix([
            [0],
            [0],
            [u[0, 0]]
        ])

        Mw = 0*matrix([
            [0.1],
            [-0.1],
            [0.2]
        ])

        Fv = 0*matrix([
            [1],
            [1],
            [1]
        ])

        Q = matrix([
            [-self.q[1, 0], -self.q[2, 0], -self.q[3, 0]],
            [self.q[0, 0], -self.q[3, 0], self.q[2, 0]],
            [self.q[3, 0], self.q[0, 0], -self.q[1, 0]],
            [-self.q[2, 0], self.q[1, 0], self.q[0, 0]]
        ])

        # Aggiorno lo stato
        q_dot = 0.5 * Q * self.omega
        self.q = self.q + q_dot * self.dt
        self.q = self.q/linalg.norm(self.q)

        U = matrix([
            [u[1, 0]],
            [u[2, 0]],
            [u[3, 0]]
        ])
        omega_dot = self.Mat_Jinv * (U - self.VectorialProduct(self.omega) * self.Mat_J * self.omega) + self.Mat_Jinv * Mw
        self.omega = self.omega + omega_dot * self.dt

        p_dot = self.v
        self.p = self.p + p_dot * self.dt

        R = self.quaternion2RotationMatrix()
        G = matrix([
            [0],
            [0],
            [self.g]
        ])
        v_dot = (1 / self.m) * (R * F_b + Fv) - G - self.airFriction * linalg.norm(self.v) * self.v
        self.v = self.v + v_dot * self.dt

        self.x = vstack((
            self.q,
            self.omega,
            self.p,
            self.v
        ))

    def getMotorInput(self):
        scaleFactor = self.thrustGain * 65536.0 / (self.fmotmax * 4)
        u = self.u
        u[0, 0] = u[0, 0]*scaleFactor
        u[1, 0] = (u[1, 0]/2.0)/self.d
        u[2, 0] = (u[2, 0]/2.0)/self.d
        u[3, 0] = 0/self.c
        percentual = 1
        if u[1, 0] < -65536 * percentual:
            u[1, 0] = -65536 * percentual
        elif u[1, 0] > 65536 * percentual:
            u[1, 0] = 65536 * percentual

        if u[2, 0] < -65536 * percentual:
            u[2, 0] = -65536 * percentual
        elif u[2, 0] > 65536 * percentual:
            u[2, 0] = 65536 * percentual

        if u[3, 0] < -65536 * percentual:
            u[3, 0] = -65536 * percentual
        elif u[3, 0] > 65536 * percentual:
            u[3, 0] = 65536 * percentual

        m1 = u[0, 0] - u[1, 0] + u[2, 0] + u[3, 0]
        m2 = u[0, 0] - u[1, 0] - u[2, 0] - u[3, 0]
        m3 = u[0, 0] + u[1, 0] - u[2, 0] + u[3, 0]
        m4 = u[0, 0] + u[1, 0] + u[2, 0] - u[3, 0]
        return m1, m2, m3, m4

    def quaternion2RotationMatrix(self):
        """
        Genera la matrice di rotazione partendo dai quaternioni dello stato
        :return:
        """
        q0 = self.q[0, 0]
        q1 = self.q[1, 0]
        q2 = self.q[2, 0]
        q3 = self.q[3, 0]
        R = matrix([
            [1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
            [2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)]
        ])
        return R

    def VectorialProduct(self, v):
        """
        Questa funzione prende in ingresso un vettore di tre elementi e ne genera
        la matrice che effettua il prodotto vettoriale
        :param v:
        :return: M
        """
        M = matrix([
            [0, -v[2,0], v[1,0]],
            [v[2,0], 0, -v[0,0]],
            [-v[1,0], v[0,0], 0]
        ])
        return M

    def quaternion2RPY(self):
        q = self.q

        g = 2 * (q[0, 0]*q[2, 0] - q[1, 0]*q[3, 0])
        if g > 1:
            g = 1
        elif g < -1:
            g = -1

        yaw = atan2(2*(q[1, 0]*q[2, 0] + q[0, 0]*q[3, 0]), q[0, 0] * q[0, 0] + q[1, 0] * q[1, 0] - q[2, 0] * q[2, 0] - q[3, 0] * q[3, 0])
        pitch = asin(g)
        roll = atan2(2*(q[2, 0]*q[3, 0] + q[0, 0]*q[1, 0]), q[0, 0] * q[0, 0] - q[1, 0] * q[1, 0] - q[2, 0] * q[2, 0] + q[3, 0] * q[3, 0])
        rad2deg = 180/pi
        euler = matrix([
            [roll * rad2deg], [pitch * rad2deg], [yaw * rad2deg]
        ])
        return euler[0, 0], euler[1, 0], euler[2, 0]


