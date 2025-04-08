import numpy as np
from numpy import pi , cos , sin
import matplotlib.pyplot as plt




class Robot():
    def __init__(self, L1, L2, Rb, Re):
        self.L1 = L1
        self.L2=L2
        self.Rb=Rb
        self.Re=Re
        self.q = []
    
        
    def traceRobot(self, alpha1, beta1, alpha2, beta2, alpha3, beta3) :
        self.q[0] = alpha1
        self.q[1] = beta1
        self.q[2] = alpha2
        self.q[3] = beta2
        self.q[4] = alpha3
        self.q[5] = beta3

        # Matrices de rotation 2*2
        Rot1=np.array([[cos(2*pi/3), -sin(2*pi/3)],
                    [sin(2*pi/3), cos(2*pi/3)]])
        

        Rot2=np.array([[cos(4*pi/3), -sin(4*pi/3)],
                    [sin(4*pi/3), cos(4*pi/3)]])

        # Vecteurs 
        P10=np.array([[0],
                    [-self.Rb]])
        

        T1 = np.array([
            [1, 0,   0],
            [0, 1, -self.Rb],
            [0, 0,   1]
        ])

        # --- P11 ---
        P11_vec = np.array([
            self.L1 * np.cos(alpha1),
            self.L1 * np.sin(alpha1),
            1
        ])
        P11 = T1 @ P11_vec

        # --- P12 ---
        P12_vec = np.array([
            self.L1 * np.cos(alpha1) + self.L2 * np.cos(alpha1 + beta1),
            self.L1 * np.sin(alpha1) + self.L2 * np.sin(alpha1 + beta1),
            1
        ])
        P12 = T1 @ P12_vec

        # --- P20 ---
        P20 = np.array([
            self.Rb * np.sqrt(3) / 2,
            self.Rb / 2
        ])

        # --- Matrice homogène pour P21 et P22 ---
        T2 = np.block([
            [Rot1, P20.reshape(2,1)],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        # --- P21 ---
        P21_vec = np.array([
            self.L1 * np.cos(alpha2),
            self.L1 * np.sin(alpha2),
            1
        ])
        P21 = T2 @ P21_vec

        # --- P22 ---
        P22_vec = np.array([
            self.L1 * np.cos(alpha2) + self.L2 * np.cos(alpha2 + beta2),
            self.L1 * np.sin(alpha2) + self.L2 * np.sin(alpha2 + beta2),
            1
        ])
        P22 = T2 @ P22_vec

        # --- P30 ---
        P30 = np.array([
            -self.Rb * np.sqrt(3) / 2,
            self.Rb / 2
        ])

        # --- Matrice homogène pour P31 et P32 ---
        T3 = np.block([
            [Rot2, P30.reshape(2,1)],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        # --- P31 ---
        P31_vec = np.array([
            self.L1 * np.cos(alpha3),
            self.L1 * np.sin(alpha3),
            1
        ])
        P31 = T3 @ P31_vec

        # --- P32 ---
        P32_vec = np.array([
            self.L1 * np.cos(alpha3) + self.L2 * np.cos(alpha3 + beta3),
            self.L1 * np.sin(alpha3) + self.L2 * np.sin(alpha3 + beta3),
            1
        ])
        P32 = T3 @ P32_vec




        plt.figure()
        plt.title(" Modélisation du robot 3RRR")


        # Tracer bras 1
        plt.plot([P10[0], P11[0], P12[0]], [P10[1], P11[1], P12[1]], label="Bras 1")

        # Tracer bras 2
        plt.plot([P20[0], P21[0], P22[0]], [P20[1], P21[1], P22[1]], label="Bras 2")

        # Tracer bras 3
        plt.plot([P30[0], P31[0], P32[0]], [P30[1], P31[1], P32[1]], label="Bras 3")

        # Tracer effecteur (triangle)
        plt.plot([P12[0], P22[0], P32[0], P12[0]],
                [P12[1], P22[1], P32[1], P12[1]],
                linewidth=2, label="Effecteur")

        # Mise à l’échelle
        plt.axis("equal")
        plt.axis("square")

        # Afficher les légendes (optionnel)
        plt.legend()
        plt.grid(True)
        plt.title("Visualisation des bras + effecteur")

        plt.show()


def main():
    robot= Robot(5,5,10,10)
    robot.traceRobot(pi/2,pi/2,pi/2,pi/2,pi/2,pi/2)

if __name__=="__main__":
    main()