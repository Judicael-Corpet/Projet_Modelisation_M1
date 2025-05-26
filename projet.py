import numpy as np
from numpy import pi , cos , sin
import matplotlib.pyplot as plt
import random
import pygame
import copy

"""Created on Thue 10 April
@auhors=  Judicaël CORPET & Dan CALAMIA

while 3RRR Project

"""

class Robot():
    # Initalisation 
    def __init__(self, L1, L2, Rb, Re ,dimensionPlateau ,pos_eff, q0 ,theta=0, name="3RRR"):

        # Variables par défaut
        self.L1 = L1
        self.L2=L2
        self.Rb=Rb
        self.Re=Re
        self.dimensionPlateau=dimensionPlateau
        self.pos_eff=pos_eff
        self.q0= q0
        self.theta=theta
        self.trajectory=[]

       
        # Variables Pygame
        self.name=name
        pygame.init()
        self.width=700
        self.height=700
        self.window = pygame.display.set_mode((self.width,self.height))
        self.window.fill((232,220,202))
        pygame.display.set_caption(f"Simulation {self.name} robot")
        self.clock=pygame.time.Clock()
        self.FPS=60

        # Mise à l'échelle pour pygame
        self.scale= self.width/self.dimensionPlateau
        # print(self.scale)
    # Calculs des positons de toutes les liaisons
    def calculPos(self,q):
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
            [Rot2,P30.reshape(2, 1)],
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

        P= [P10,P11,P12,P20,P21,P22,P30,P31,P32]
        return P
    # Trace sur Matplotlib
    def traceRobot(self,P, method= ""):
        P10, P11, P12, P20, P21, P22, P30, P31, P32 = P

        # Affichage du robot
        plt.figure()
        
        # Tracer bras 1
        color1 = (random.random(), random.random(), random.random())
        plt.plot([P10[0], P11[0], P12[0]], [P10[1], P11[1], P12[1]], color = color1 ,label="Bras 1")

        # Tracer bras 2
        color2 = (random.random(), random.random(), random.random())
        plt.plot([P20[0], P21[0], P22[0]], [P20[1], P21[1], P22[1]], color = color2, label="Bras 2")

        # Tracer bras 3
        color3 = (random.random(), random.random(), random.random())
        plt.plot([P30[0], P31[0],P32[0]], [P30[1], P31[1], P32[1]], color = color3, label="Bras 3")

        # Tracer effecteur (triangle)
        plt.plot([P12[0], P22[0], P32[0], P12[0]],
                [P12[1], P22[1], P32[1], P12[1]],
                linewidth=2, label="Effecteur")

        # Tracer du centre de gravité de l'effecteur (qui sera la pointe qui écrit), et qui permettra plus facilement de vérifier si les coordonnées envoyées sont correctes
        # Calcul du barycentre du triangle de l'effecteur
        Gx = (P12[0] + P22[0] + P32[0]) / 3
        Gy = (P12[1] + P22[1] + P32[1]) / 3

        # Tracer le barycentre
        plt.plot(Gx, Gy, 'ro', label='Pointe effecteur')  # 'ro' = point rouge

        # Mise à l’échelle
        plt.axis("equal")
        plt.axis("square")

        # Afficher les légendes (optionnel)
        plt.legend()
        plt.grid(True)
        plt.title(f"Visualisation des bras + effecteur (méthode: {method})") 

        plt.show()
    # Méthode analytique de résolution 
    def MGI_analytique(self,pos_eff):
        # Variables globales
        

        # Matrice de rotation et translation de l'effecteur
        RotEff = np.array([[np.cos(pos_eff[2]), -np.sin(pos_eff[2])], [np.sin(pos_eff[2]), np.cos(pos_eff[2])]])
        Transl = np.array([pos_eff[0], pos_eff[1]])
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

            q.append(alpha)
            q.append(beta)

        return np.array(q)
        
        # return np.array(q)
    # Méthode numérique de résolution
    def solve_eq_NL(self,q):
        if self.check_extesnsion(q)== True:
            print(" Il y a une singularité: extension")
        # Variables globales
        # print(q[1],q[3],q[5])
        # Extraction des valeurs de alpha et beta pour chaque bras
        alpha = [q[0], q[2], q[4]]
        beta = [q[1], q[3], q[5]]

        # Angles R_i par rapport à R_0
        ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]

        # Angles des positions O_i et E_i
        ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

        # Matrice de rotation et translation de l'effecteur
        RotEff = np.array([[np.cos(self.pos_eff[2]), -np.sin(self.pos_eff[2])], [np.sin(self.pos_eff[2]), np.cos(self.pos_eff[2])]])
        Transl = np.array([self.pos_eff[0], self.pos_eff[1]])
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

    def no_linear(self,q0):
        # 1ère méthode: Résoudre le système d'équations non linéaires
        from scipy.optimize import fsolve
        q = fsolve(lambda q: self.solve_eq_NL(q), q0)
        return q
    # Vérifie la singularité de série
    def check_extesnsion(self,q):
        result=False
        if q[1]==0 or q[3]==0 or q[5]==0:
            result=True
        return result
    # Calcul des matrices A et B pour vérifier les singularités
    def calculer_gamma(self,A,B):
        vec_AiBi = B - A
        # Angle entre AiBi et l’axe x (global), puis exprimé dans le repère de l’effecteur
        gamma_i = np.arctan2(vec_AiBi[1], vec_AiBi[0]) - self.theta

    
        return gamma_i

    def calculer_d(self,A, B):
        """
        Calcule le bras de levier signé (d_i) entre le centre de la plateforme C
        et la droite A->B, en gardant le signe du moment.
        """
        E= [self.pos_eff[0], self.pos_eff[1]] # pos centre effecteur
        E=np.array(E)
        AB = B - A
        AE = E - A
        numerateur = AB[0]*AE[1] - AB[1]*AE[0]  # produit vectoriel 2D signé
        denominateur = np.linalg.norm(AB)
        d = numerateur / denominateur
        return d  # signe conservé

    def calculer_e(self,A, B, O):
        AB = B - A
        AO = O - A
        # Produit vectoriel |AB x AO|| 
        numerateur = np.abs(AB[0]*AO[1] - AB[1]*AO[0])
        denominateur = np.linalg.norm(AB)
        e = numerateur / denominateur
        return e

    def est_modulo_pi(self,gammas, tol=1e-6):
        """
        Vérifie si tous les angles de la liste `gammas` sont égaux modulo π (±π).
        
        Paramètres :
        - gammas : liste ou array d'angles en radians
        - tol : tolérance pour comparaison numérique
        
        Retourne : True si tous les γi ≡ γj mod π
        """
        gamma0 = gammas[0] % np.pi
        return all(np.abs((g % np.pi) - gamma0) < tol or
                np.abs((g % np.pi) - gamma0 - np.pi) < tol for g in gammas[1:])

    def calcul_matrices_AB(self,P):
        """
        Calcule les matrices A et B pour un robot parallèle 3RRR.
        
        Paramètres :
        - gamma : liste ou array des angles gammai [gammai1, gammai2, gammai3] en radians
        - d : liste ou array des bras de levier di [d1, d2, d3]
        - e : liste ou array des bras de levier ei [e1, e2, e3]
        
        Retourne :
        - A : matrice 3x3
        - B : matrice 3x3
    
        """
        
        P10, P11, P12, P20, P21, P22, P30, P31, P32 = P

        O1 = np.array([P10[0],P10[1]])
        A1 = np.array([P11[0],P11[1]]) # car on veut que x et y , lors des calculs de positions on a parfois des vecteurs de taille 3 
        B1 = np.array([P12[0],P12[1]])
        O2 = np.array([P20[0],P20[1]])
        A2 = np.array([P21[0],P21[1]])
        B2 = np.array([P22[0],P22[1]])
        O3 = np.array([P30[0],P30[1]])
        A3 = np.array([P31[0],P31[1]])
        B3 = np.array([P32[0],P32[1]])
        # calculer e 
        e1 = self.calculer_e(A1,B1,O1)
        e2 = self.calculer_e(A2,B2,O2)
        e3 = self.calculer_e(A3,B3,O3)
        e= np.array([e1,e2,e3])
        # calculer d
        d1 = self.calculer_d(A1,B1)
        d2 = self.calculer_d(A2,B2)
        d3 = self.calculer_d(A3,B3)
        d= np.array([d1,d2,d3])
        # calculer gamma
        gamma1 = self.calculer_gamma(A1,B1)
        gamma2 = self.calculer_gamma(A2,B2)
        gamma3 = self.calculer_gamma(A3,B3)
        gamma = np.array([gamma1,gamma2,gamma3])

        print(f"gamma: {gamma}")
        print(f"d: {d}")
        print(f"e: {e}")
        A = np.array([
            [np.cos(gamma[0]), np.sin(gamma[0]), d[0]],
            [np.cos(gamma[1]), np.sin(gamma[1]), d[1]],
            [np.cos(gamma[2]), np.sin(gamma[2]), d[2]]
        ])
        
        B = np.diag(e)  # matrice diagonale avec e1, e2, e3 sur la diagonale
        det_A = np.linalg.det(A)
        det_B = np.linalg.det(B)
        print(det_A)
        print(det_B)
        tolerance = 0.01
        if -tolerance < det_A < 0 + tolerance and self.est_modulo_pi(gamma) :
            print(" 1ere Singularite parallele (det(A) = 0 et gamma = Cte(pi)) ")
            print("True")
            return True

        elif -tolerance < det_A < 0 + tolerance:
            print(" 2eme Singularite parallele (det(A) = 0) ")
            print("True")
            return True
        elif -0.001 < det_B < 0 + 0.001 :
            # print(" Singularite série (det(B)) = 0) "
            pass
        
        return False
    # Fonction pour l'affichage sur Pygame
    def to_screen(self, pos):
        """Convertit les coordonnées réelles en pixels pour Pygame (origine au centre)"""
        x = int(self.width / 2 + pos[0] * self.scale)
        y = int(self.height / 2 - pos[1] * self.scale)
        return (x, y)

    def draw_robot(self, P):
        """Dessine le robot sur la fenêtre pygame avec une apparence professionnelle"""
        P10, P11, P12, P20, P21, P22, P30, P31, P32 = P

        # Couleurs sérieuses / scientifiques
        BACKGROUND = (240, 240, 245)
        TEXT_COLOR = (30, 30, 30)
        BASE_COLOR = (80, 80, 80)
        ARM1_COLOR = (200, 70, 70)
        ARM2_COLOR = (70, 160, 90)
        ARM3_COLOR = (70, 130, 180)
        EFFECTOR_COLOR = (120, 90, 200)
        TRAJECTORY_COLOR = (50, 50, 200)
        JOINT_COLOR = (20, 20, 20)

        self.window.fill(BACKGROUND)

        # Affichage de l'angle
        font = pygame.font.SysFont("Arial", 28)
        texte_angle = font.render(f"θ : {self.theta:.1f}°", True, TEXT_COLOR)
        self.window.blit(texte_angle, (10, 10))

        # Liste des bras pour boucle
        bras = [
            (P10, P11, P12, ARM1_COLOR),
            (P20, P21, P22, ARM2_COLOR),
            (P30, P31, P32, ARM3_COLOR)
        ]

        for O, A, B, color in bras:
            pygame.draw.line(self.window, color, self.to_screen(O[:2]), self.to_screen(A[:2]), 4)
            pygame.draw.line(self.window, color, self.to_screen(A[:2]), self.to_screen(B[:2]), 4)

            # Cercles aux joints
            for joint in [O, A, B]:
                pygame.draw.circle(self.window, JOINT_COLOR, self.to_screen(joint[:2]), 4)

        # Triangle de l'effecteur
        pygame.draw.polygon(self.window, EFFECTOR_COLOR, [
            self.to_screen(P12[:2]),
            self.to_screen(P22[:2]),
            self.to_screen(P32[:2])
        ], width=2)

        # Barycentre de l'effecteur
        Gx = (P12[0] + P22[0] + P32[0]) / 3
        Gy = (P12[1] + P22[1] + P32[1]) / 3
        pygame.draw.circle(self.window, (255, 0, 0), self.to_screen((Gx, Gy)), 5)

        # Tracé de la trajectoire
        if len(self.trajectory) > 1:
            pygame.draw.lines(self.window, TRAJECTORY_COLOR, False, self.trajectory, 2)

        pygame.display.update()
    # Simulation
    def run_simulation(self):
        """Lance une animation simple avec des positions successives de l effecteur"""
        running = True
        t = 0
        self.mode = "manual"  # autres valeurs possibles: "circle", "trefoil"
        x, y = self.pos_eff[0], self.pos_eff[1]
        q = self.MGI_analytique(self.pos_eff)

        while running:
            self.clock.tick(self.FPS)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            keys = pygame.key.get_pressed()
            
    
             # Mode sélectionné par touche
            if keys[pygame.K_o]:
                self.mode = "circle"
            if keys[pygame.K_p]:
                self.mode = "trefoil"
            if keys[pygame.K_m]:
                self.mode = "manual"
       
            if self.mode == "manual":
                if keys[pygame.K_UP]:
                    x_temp, y_temp = x, y + 0.001
                    pos_eff_future = np.array([x_temp, y_temp, self.theta])
                    q_future = self.MGI_analytique(pos_eff_future)
                    if  not self.check_extesnsion(q_future):
                        y += 0.001
                    else:
                        print("position inattéignable")
                if keys[pygame.K_DOWN]:
                    x_temp, y_temp = x, y - 0.001
                    pos_eff_future = np.array([x_temp, y_temp, self.theta])
                    q_future = self.MGI_analytique(pos_eff_future)
                    if  not self.check_extesnsion(q_future):
                        y -= 0.001
                    else:
                        print("position inattéignable")
                if keys[pygame.K_LEFT]:
                    x_temp, y_temp = x- 0.001,y
                    pos_eff_future = np.array([x_temp, y_temp, self.theta])
                    q_future = self.MGI_analytique(pos_eff_future)
                    if  not self.check_extesnsion(q_future):
                        x -= 0.001
                    else:
                        print("position inattéignable")

                if keys[pygame.K_RIGHT]:
                    x_temp, y_temp = x+ 0.001,y
                    pos_eff_future = np.array([x_temp, y_temp, self.theta])
                    q_future = self.MGI_analytique(pos_eff_future)
                    if  not self.check_extesnsion(q_future):
                        x += 0.001
                    else:
                        print("position inattéignable")

                if keys[pygame.K_l]:
                    theta_temp = self.theta + 0.01
                    pos_eff_future = np.array([x, y,theta_temp])
                    q_future = self.MGI_analytique(pos_eff_future)
                    if  not self.check_extesnsion(q_future):
                        self.theta = theta_temp
                    else:
                        print("position inattéignable")

                if keys[pygame.K_k]:
                    theta_temp = self.theta - 0.01
                    pos_eff_future = np.array([x, y,theta_temp])
                    q_future = self.MGI_analytique(pos_eff_future)
                    if  not self.check_extesnsion(q_future):
                        self.theta = theta_temp
                    else:
                        print("position inattéignable")


            elif self.mode == "circle":
                x_temp, y_temp = 0.1 * np.cos(t),0.1 * np.sin(t)
                pos_eff_future = np.array([x_temp, y_temp, self.theta])
                q_future = self.MGI_analytique(pos_eff_future)
                if  not self.check_extesnsion(q_future):
                    x = 0.1 * np.cos(t)
                    y = 0.1 * np.sin(t)
                    t += 0.01

            elif self.mode == "trefoil":
                compteur=0
                K = 0.02  # gain
                r = K * (1 + np.cos(4 * t) + 2 * (np.sin(4 * t))**2)
                x_temp, y_temp = r * np.cos(t), r * np.sin(t)
                pos_eff_future = np.array([x_temp, y_temp, self.theta])
                q_future = self.MGI_analytique(pos_eff_future)

                if not self.check_extesnsion(q_future):
                    print("ok check")
                    P_future = self.calculPos(q_future)
                    singularite = self.calcul_matrices_AB(P_future)
                    print("Singularité ?", singularite)

                    if not singularite :
                        # Pas de singularité -> on avance
                        x = x_temp
                        y = y_temp
                        t += 0.01
                        if self.theta > 0: # pour remttre l'effecteur droit, pour etre dans la meilleur position
                            theta_temp = self.theta - 0.01
                            if not self.check_extesnsion(q_test):
                                self.theta = theta_temp
                                print("θ ajusté pour éviter la singularité :", self.theta)
                            else:
                                print("Position inatteignable, même avec rotation.")


                    else:
                        # Singularité détectée, ajuste theta vers la gauche
                        compteur+=1
                        theta_temp = self.theta + 0.01
                            

                      

                        # Réessayer avec nouvel angle
                        pos_eff_test = np.array([x_temp, y_temp, theta_temp])
                        q_test = self.MGI_analytique(pos_eff_test)

                        if not self.check_extesnsion(q_test):
                            self.theta = theta_temp
                            print("θ ajusté pour éviter la singularité :", self.theta)
                        else:
                            print("Position inatteignable, même avec rotation.")

            
            self.pos_eff = np.array([x, y, self.theta])
            pos = (self.pos_eff[0],self.pos_eff[1])
            self.trajectory.append(self.to_screen(pos))
            q = self.MGI_analytique(self.pos_eff)
            P=self.calculPos(q)
            self.draw_robot(P)

        pygame.quit()
 