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
    
    def get_L1(self):
        return self.L1
    
    def get_L2(self):
        return self.L2
    
    def get_Rb(self):
        return self.Rb
    
    def get_Re(self):
        return self.LRe

    def traceRobot(self, q):
        alpha1 = q[0]
        beta1 = q[1]
        alpha2 = q[2]
        beta2 = q[3]
        alpha3 = q[4]
        beta3 = q[5]

        # Matrices de rotation 2*2
        Rot1 = np.array([[cos(2 * pi / 3), -sin(2 * pi / 3)],
                        [sin(2 * pi / 3), cos(2 * pi / 3)]])
        Rot2 = np.array([[cos(4 * pi / 3), -sin(4 * pi / 3)],
                        [sin(4 * pi / 3), cos(4 * pi / 3)]])

        # Vecteurs
        P10 = np.array([0, -self.Rb])

        T1 = np.array([
            [1, 0, 0],
            [0, 1, -self.Rb],
            [0, 0, 1]
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
            [Rot1, P20.reshape(2, 1)],
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
            [Rot2, P30.reshape(2, 1)],
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

        # Affichage du robot
        plt.figure()
        plt.title("Modélisation du robot 3RRR")

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

    def MGI_analytique(self,eff):
        # Variables globales
        

        # Matrice de rotation et translation de l'effecteur
        RotEff = np.array([[np.cos(eff[2]), -np.sin(eff[2])], [np.sin(eff[2]), np.cos(eff[2])]])
        Transl = np.array([eff[0], eff[1]])
        THEff = np.block([[RotEff, Transl.reshape(-1, 1)], [0, 0, 1]])

        # Angles R_i par rapport à R_0
        ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]

        # Angles des positions O_i et E_i
        ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

        q = []

        # Calcul pour chaque bras
        for i in range(3):
            # Matrice de rotation et translation de R_i par rapport à R_0
            Rot = np.array([[np.cos(ang1[i]), -np.sin(ang1[i])], [np.sin(ang1[i]), np.cos(ang1[i])]])
            TH = np.block([[Rot, np.array([self.Rb * np.cos(ang2[i]), self.Rb * np.sin(ang2[i])]).reshape(-1, 1)], [0, 0, 1]])

            # Position des points E_i dans R_E
            PEi_E = np.array([self.Re * np.cos(ang2[i]), self.Re * np.sin(ang2[i]), 1])

            # Position des trois points E_i de l'effecteur dans R_0
            PEi_0 = THEff @ PEi_E

            # Position des points effecteur E_i dans les repères des robots R_i
            PEi_i = np.linalg.inv(TH) @ PEi_0

            # MGI 2R plan
            x = PEi_i[0]
            y = PEi_i[1]

            aux = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            if abs(aux) < 1:
                beta = np.arccos(aux)  # Solution coude en haut
            else:
                beta = 0
                print("Problème d'atteignabilité")

            alpha = np.arctan2(y, x) - np.arctan2(self.L2 * np.sin(beta), self.L1 + self.L2 * np.cos(beta))

            q.append([alpha, beta])

        return np.array(q)

    def solve_eq_NL(self,q, eff):
        # Variables globales
        

        # Extraction des valeurs de alpha et beta pour chaque bras
        alpha = [q[0], q[2], q[4]]
        beta = [q[1], q[3], q[5]]

        # Angles R_i par rapport à R_0
        ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]

        # Angles des positions O_i et E_i
        ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

        # Matrice de rotation et translation de l'effecteur
        RotEff = np.array([[np.cos(eff[2]), -np.sin(eff[2])], [np.sin(eff[2]), np.cos(eff[2])]])
        Transl = np.array([eff[0], eff[1]])
        THEff = np.block([[RotEff, Transl.reshape(-1, 1)], [0, 0, 1]])

        F = []

        for i in range(3):
            # Position des points E_i dans R_E
            PEi_E = np.array([self.Re * np.cos(ang2[i]),self.Re * np.sin(ang2[i]), 1])

            # Position des trois points E_i de l'effecteur dans R_0
            PEi_0 = THEff @ PEi_E

            # Matrice de rotation et translation de R_i par rapport à R_0
            Rot = np.array([[np.cos(ang1[i]), -np.sin(ang1[i])], [np.sin(ang1[i]), np.cos(ang1[i])]])
            THRi_0 = np.block([[Rot, np.array([self.Rb * np.cos(ang2[i]), self.Rb * np.sin(ang2[i])]).reshape(-1, 1)], [0, 0, 1]])

            # Points B_i, extrémités des bras en fonction de alpha_i et beta_i dans R_0
            PBi = THRi_0 @ np.array([self.L1 * np.cos(alpha[i]) + self.L2 * np.cos(alpha[i] + beta[i]),
                                    self.L1 * np.sin(alpha[i]) + self.L2 * np.sin(alpha[i] + beta[i]),
                                    1])

            # Les contraintes expriment que B_i doit être confondu avec E_i
            # F(2*i-1) et F(2*i) doivent être égales à zéro (les coordonnées x et y)
            F.append(PBi[0] - PEi_0[0])  # x
            F.append(PBi[1] - PEi_0[1])  # y

        return np.array(F)



def main():

    

    L1=0.10;  # long segment 1
    L2=0.10;  # long segment 2
    Rb=0.1322594;  # Rayon base
    Re=0.07; # Rayon effecteur

    robot= Robot(L1,L2,Rb,Re)
    pos_eff=[0.0, 0.0, 0]; # pose effecteur

    # 1ière méthode : résolution de systèmes d'éq non-linéaires 
    # solutions initiales des angles alpha beta des bras 1,2,3
    q0= np.array([
                [0], 
                [pi/2], 
                [0],
                [pi/2 ],
                [0],
                [pi/2]])

    
    from scipy.optimize import fsolve
    # 1ère méthode: Résoudre le système d'équations non linéaires
    q = fsolve(lambda q: robot.solve_eq_NL(q, pos_eff), q0)
    robot.traceRobot(q)

    # 2ième méthode : Résolution analytique du MGI 2R plan :renvoie alphi_i et beta_i
    # q=robot.MGI_analytique(pos_eff)
    # print(len(q))
    # robot.traceRobot(q)


if __name__=="__main__":
    main()