import numpy as np
from numpy import pi , cos , sin
import matplotlib.pyplot as plt
import random
import pygame
import copy

"""Created on Thue 10 April
@auhors= Dan CALAMIA & Judicaël CORPET

while 3RRR Project

"""

class Robot():

    def __init__(self, L1, L2, Rb, Re ,dimensionPlateau ,pos_eff, q0 , name="3RRR"):

        # Variables par défaut
        self.L1 = L1
        self.L2=L2
        self.Rb=Rb
        self.Re=Re
        self.dimensionPlateau=dimensionPlateau
        self.pos_eff=pos_eff
        self.q0= q0


        # Variables de position du robot
        self.P10, self.P11, self.P12,self.P20,self.P21, self.P22, self.P30, self.P31,self.P32=[],[],[],[],[],[],[],[],[] # pas obligé de les déclaré dans init en réalité

        # Variables Pygame
        self.name=name
        pygame.init()
        self.width=800
        self.height=800
        self.window = pygame.display.set_mode((self.width,self.height))
        self.window.fill((232,220,202))
        pygame.display.set_caption(f"Simulation of {self.name} robot")
        self.clock=pygame.time.Clock()
        self.FPS=60

        # Mise à l'échelle pour pygame
        self.scale= self.width/self.dimensionPlateau
        print(self.scale)
    
    def get_L1(self):
        return self.L1
    
    def get_L2(self):
        return self.L2
    
    def get_Rb(self):
        return self.Rb
    
    def get_Re(self):
        return self.LRe

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
        self.P10 = np.array([0, -self.Rb])

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
        self.P11 = T1 @ P11_vec

        # --- P12 ---
        P12_vec = np.array([
            self.L1 * np.cos(alpha1) + self.L2 * np.cos(alpha1 + beta1),
            self.L1 * np.sin(alpha1) + self.L2 * np.sin(alpha1 + beta1),
            1
        ])
        self.P12 = T1 @ P12_vec

        # --- P20 ---
        self.P20 = np.array([
            self.Rb * np.sqrt(3) / 2,
            self.Rb / 2
        ])

        # --- Matrice homogène pour P21 et P22 ---
        T2 = np.block([
            [Rot1, self.P20.reshape(2, 1)],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        # --- P21 ---
        P21_vec = np.array([
            self.L1 * np.cos(alpha2),
            self.L1 * np.sin(alpha2),
            1
        ])
        self.P21 = T2 @ P21_vec

        # --- P22 ---
        P22_vec = np.array([
            self.L1 * np.cos(alpha2) + self.L2 * np.cos(alpha2 + beta2),
            self.L1 * np.sin(alpha2) + self.L2 * np.sin(alpha2 + beta2),
            1
        ])
        self.P22 = T2 @ P22_vec

        # --- P30 ---
        self.P30 = np.array([
            -self.Rb * np.sqrt(3) / 2,
            self.Rb / 2
        ])

        # --- Matrice homogène pour P31 et P32 ---
        T3 = np.block([
            [Rot2,self.P30.reshape(2, 1)],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        # --- P31 ---
        P31_vec = np.array([
            self.L1 * np.cos(alpha3),
            self.L1 * np.sin(alpha3),
            1
        ])
        self.P31 = T3 @ P31_vec

        # --- P32 ---
        P32_vec = np.array([
            self.L1 * np.cos(alpha3) + self.L2 * np.cos(alpha3 + beta3),
            self.L1 * np.sin(alpha3) + self.L2 * np.sin(alpha3 + beta3),
            1
        ])
        self.P32 = T3 @ P32_vec

    def traceRobot(self, method= ""):
        # Affichage du robot
        plt.figure()
        
        # Tracer bras 1
        color1 = (random.random(), random.random(), random.random())
        plt.plot([self.P10[0], self.P11[0], self.P12[0]], [self.P10[1], self.P11[1], self.P12[1]], color = color1 ,label="Bras 1")

        # Tracer bras 2
        color2 = (random.random(), random.random(), random.random())
        plt.plot([self.P20[0], self.P21[0], self.P22[0]], [self.P20[1], self.P21[1], self.P22[1]], color = color2, label="Bras 2")

        # Tracer bras 3
        color3 = (random.random(), random.random(), random.random())
        plt.plot([self.P30[0], self.P31[0],self. P32[0]], [self.P30[1], self.P31[1], self.P32[1]], color = color3, label="Bras 3")

        # Tracer effecteur (triangle)
        plt.plot([self.P12[0], self.P22[0], self.P32[0], self.P12[0]],
                [self.P12[1], self.P22[1], self.P32[1], self.P12[1]],
                linewidth=2, label="Effecteur")

        # Tracer du centre de gravité de l'effecteur (qui sera la pointe qui écrit), et qui permettra plus facilement de vérifier si les coordonnées envoyées sont correctes
        # Calcul du barycentre du triangle de l'effecteur
        Gx = (self.P12[0] + self.P22[0] + self.P32[0]) / 3
        Gy = (self.P12[1] + self.P22[1] + self.P32[1]) / 3

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
        
    def MGI_analytique(self):
        # Variables globales
        

        # Matrice de rotation et translation de l'effecteur
        RotEff = np.array([[np.cos(self.pos_eff[2]), -np.sin(self.pos_eff[2])], [np.sin(self.pos_eff[2]), np.cos(self.pos_eff[2])]])
        Transl = np.array([self.pos_eff[0], self.pos_eff[1]])
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
    
    def check_extesnsion(self,q):
        result=False
        if q[1]==0 or q[3]==0 or q[5]==0:
            result=True
        return result

# _____________FONCTIONS PYGAME__________________

    def randcolor(self):
        r= random.randint(0,255)
        g= random.randint(0,255)
        b= random.randint(0,255)

        return r,g,b

    def convertToPygame(self,c):
        x= self.width//2+ c[0]*self.scale
        y= self.height//2 - c[1]*self.scale
  

        return (x,y)

    def calculPosPygame(self,q):
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
        self.P10 = np.array([0, -self.Rb])

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
        self.P11 = T1 @ P11_vec

        # --- P12 ---
        P12_vec = np.array([
            self.L1 * np.cos(alpha1) + self.L2 * np.cos(alpha1 + beta1),
            self.L1 * np.sin(alpha1) + self.L2 * np.sin(alpha1 + beta1),
            1
        ])
        self.P12 = T1 @ P12_vec

        # --- P20 ---
        self.P20 = np.array([
            self.Rb * np.sqrt(3) / 2,
            self.Rb / 2
        ])

        # --- Matrice homogène pour P21 et P22 ---
        T2 = np.block([
            [Rot1, self.P20.reshape(2, 1)],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        T22 = np.block([
            [Rot1, self.P20.reshape(2, 1)],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        # --- P21 ---
        P21_vec = np.array([
            self.L1 * np.cos(alpha2),
            -self.L1 * np.sin(alpha2),
            1
        ])
        self.P21 = T22 @ P21_vec

        # --- P22 ---
        P22_vec = np.array([
            self.L1 * np.cos(alpha2) + self.L2 * np.cos(alpha2 + beta2),
            self.L1 * np.sin(alpha2) + self.L2 * np.sin(alpha2 + beta2),
            1
        ])
        self.P22 = T2 @ P22_vec

        # --- P30 ---
        self.P30 = np.array([
            -self.Rb * np.sqrt(3) / 2,
            self.Rb / 2
        ])

        # --- Matrice homogène pour P31 et P32 ---
        T3 = np.block([
            [Rot2,self.P30.reshape(2, 1)],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        # --- P31 ---
        P31_vec = np.array([
            self.L1 * np.cos(alpha3),
            self.L1 * np.sin(alpha3),
            1
        ])
        self.P31 = T3 @ P31_vec

        # --- P32 ---
        P32_vec = np.array([
            self.L1 * np.cos(alpha3) + self.L2 * np.cos(alpha3 + beta3),
            self.L1 * np.sin(alpha3) + self.L2 * np.sin(alpha3 + beta3),
            1
        ])
        self.P32 = T3 @ P32_vec

    def drawPygame(self):
        # color bras
        color_arm=(0,255,0)
        # color effecteur
        color_eff=(0,0,0)

        self.arm_width=3
        red=(255,0,0)
        pygame.draw.line(self.window, color_arm, self.convertToPygame(self.P10), self.convertToPygame(self.P11), self.arm_width)
        pygame.draw.line(self.window, color_arm, self.convertToPygame(self.P11), self.convertToPygame(self.P12), self.arm_width)

        pygame.draw.line(self.window, red, self.convertToPygame(self.P20), self.convertToPygame(self.P21), self.arm_width)
        pygame.draw.line(self.window, red, self.convertToPygame(self.P21), self.convertToPygame(self.P22), self.arm_width)

        pygame.draw.line(self.window, color_arm, self.convertToPygame(self.P30), self.convertToPygame(self.P31), self.arm_width)
        pygame.draw.line(self.window, color_arm, self.convertToPygame(self.P31), self.convertToPygame(self.P32), self.arm_width)

        pygame.draw.line(self.window, color_eff, self.convertToPygame(self.P12), self.convertToPygame(self.P22), self.arm_width)
        pygame.draw.line(self.window, color_eff, self.convertToPygame(self.P22), self.convertToPygame(self.P32), self.arm_width)
        pygame.draw.line(self.window, color_eff, self.convertToPygame(self.P32), self.convertToPygame(self.P12), self.arm_width)



        plt.plot([self.P12[0], self.P22[0], self.P32[0], self.P12[0]],
                [self.P12[1], self.P22[1], self.P32[1], self.P12[1]],
                linewidth=2, label="Effecteur")


        pygame.display.flip() # actualise la frame

    def runPygame(self,q):

            running=True
            nStep=100
            # print(self.q0)
            qtemp=self.q0.copy()
            qtemp=qtemp.flatten()
            # print(qtemp)
            step1=(q[0]-qtemp[0])/nStep
            step2=(q[1]-qtemp[1])/nStep
            step3=(q[2]-qtemp[2])/nStep
            step4=(q[3]-qtemp[3])/nStep
            step5=(q[4]-qtemp[4])/nStep
            step6=(q[5]-qtemp[5])/nStep
           
            for i in range(0,nStep):
                        qtemp[0]+=step1
                        qtemp[1]+=step2
                        qtemp[2]+=step3
                        qtemp[3]+=step4
                        qtemp[4]+=step5
                        qtemp[5]+=step6
                        print(qtemp[1],qtemp[2],qtemp[3])

                        self.calculPos(qtemp) # cela va mettre  a jour les variables Pos donc PARFAITTTT
                        self.window.fill((232,220,202))
                        self.drawPygame()
                        pygame.time.wait(100)


            while running:
                pygame.event.pump() # process event queue
                keys= pygame.key.get_pressed()
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:  # Fermer la fenêtre
                        running = False
                        
                    if keys[ord('z')] or keys[pygame.K_UP]: 
                        pass
                
                self.clock.tick(self.FPS)  

    def move_effector(self, dx, dy):
        # Déplacer l'effecteur avec les touches clavier
        self.pos_eff[0] += dx
        self.pos_eff[1] += dy

    def runPygame3(self,qfinal):


        running = True
        while running:
            self.window.fill((232, 220, 202))  # Réinitialiser la fenêtre
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Gestion des entrées clavier
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]:
                self.move_effector(0, -0.001)  # Déplacer l'effecteur vers le haut
            if keys[pygame.K_DOWN]:
                self.move_effector(0, 0.001)  # Déplacer l'effecteur vers le bas
            if keys[pygame.K_LEFT]:
                self.move_effector(-0.001, 0)  # Déplacer l'effecteur vers la gauche
            if keys[pygame.K_RIGHT]:
                self.move_effector(0.001, 0)  # Déplacer l'effecteur vers la droite

            # Recalculer la position du robot
            q = self.MGI_analytique()  # Résolution analytique des angles
            self.calculPosPygame(q)

            # Tracer le robot
            self.drawPygame()

            # Mettre à jour l'affichage
            pygame.display.update()
            self.clock.tick(self.FPS)  # Limiter la vitesse de l'animation

        pygame.quit()

    # def interpolate(self,a0, a1, t):
    #     return (1 - t) * a0 + t * a1

    # def runPygame2(self,qfinal):
        
    #     alphaf1 = qfinal[0]
    #     betaf1 = qfinal[1]
    #     alphaf2 = qfinal[2]
    #     betaf2 = qfinal[3]
    #     alphaf3 = qfinal[4]
    #     betaf3 = qfinal[5]

    #     print(qfinal)
    #     q0=self.q0.flatten()
    #     print(f'q0: {q0}')
    #     alphai1 =q0[0]
    #     betai1 = q0[1]
    #     alphai2 = q0[2]
    #     betai2 = q0[3]
    #     alphai3 = q0[4]
    #     betai3 = q0[5]
 

    #     self.calculPosPygame(q0)
    #     self.drawPygame()

    #     # Interpolation
    #     steps = 100
    #     frame = 0

    #     running = True
    #     while running:
    #         t=min(frame/steps,1.0) # facteur t pour l'interpolation
    #         q=[]
    #         alpha1 = self.interpolate(alphai1,alphaf1,t)
    #         beta1 = self.interpolate(betai1,betaf1,t)
    #         alpha2 = self.interpolate(alphai2,alphaf2,t)
    #         beta2 = self.interpolate(betai2,betaf2,t)
    #         alpha3 = self.interpolate(alphai3,alphaf3,t)
    #         beta3 = self.interpolate(betai3,betaf3,t)
            
    #         q.extend([alpha1.item(),beta1.item(),alpha2.item(),beta2.item(),alpha3.item(),beta3.item()])
    #         print(q)
    #         self.calculPosPygame(q)
            
    #         self.window.fill((232,220,202))

    #         self.drawPygame()

    #         self.clock.tick(60)

    #         frame += 1
    #         if frame > steps:
    #             frame = steps  # stop interpolation here

    #         for event in pygame.event.get():
    #             if event.type == pygame.QUIT:
    #                 running = False

            

