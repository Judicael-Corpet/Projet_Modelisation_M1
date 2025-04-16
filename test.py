import pygame
import numpy as np
import random
from math import cos, sin, pi

class Robot():
    def __init__(self, L1, L2, Rb, Re, dimensionPlateau, pos_eff, q0, name="3RRR"):
        # Initialisation des paramètres
        self.L1 = L1
        self.L2 = L2
        self.Rb = Rb
        self.Re = Re
        self.dimensionPlateau = dimensionPlateau
        self.pos_eff = pos_eff
        self.q0 = q0

        # Variables de Pygame
        self.name = name
        pygame.init()
        self.width = 800
        self.height = 800
        self.window = pygame.display.set_mode((self.width, self.height))
        self.window.fill((232, 220, 202))
        pygame.display.set_caption(f"Simulation of {self.name} robot")
        self.clock = pygame.time.Clock()
        self.FPS = 60

        # Mise à l'échelle pour pygame
        self.scale = self.width / self.dimensionPlateau
        print(self.scale)

    def calculPos(self, q):
        # Fonction pour calculer la position des points P10, P11, etc. sur les bras
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

        # Calcul des points
        self.P10 = np.array([0, -self.Rb, 1])  # P10 en coordonnées homogènes

        # --- P11 ---
        P11_vec = np.array([self.L1 * np.cos(alpha1), self.L1 * np.sin(alpha1), 1])
        self.P11 = P11_vec

        # --- P12 ---
        P12_vec = np.array([self.L1 * np.cos(alpha1) + self.L2 * np.cos(alpha1 + beta1),
                            self.L1 * np.sin(alpha1) + self.L2 * np.sin(alpha1 + beta1), 1])
        self.P12 = P12_vec

        # --- P20 ---
        self.P20 = np.array([self.Rb * np.sqrt(3) / 2, self.Rb / 2, 1])  # P20 en coordonnées homogènes

        # --- P21 ---
        P21_vec = np.array([self.L1 * np.cos(alpha2), self.L1 * np.sin(alpha2), 1])
        self.P21 = P21_vec

        # --- P22 ---
        P22_vec = np.array([self.L1 * np.cos(alpha2) + self.L2 * np.cos(alpha2 + beta2),
                            self.L1 * np.sin(alpha2) + self.L2 * np.sin(alpha2 + beta2), 1])
        self.P22 = P22_vec

        # --- P30 ---
        self.P30 = np.array([-self.Rb * np.sqrt(3) / 2, self.Rb / 2, 1])  # P30 en coordonnées homogènes

        # --- P31 ---
        P31_vec = np.array([self.L1 * np.cos(alpha3), self.L1 * np.sin(alpha3), 1])
        self.P31 = P31_vec

        # --- P32 ---
        P32_vec = np.array([self.L1 * np.cos(alpha3) + self.L2 * np.cos(alpha3 + beta3),
                            self.L1 * np.sin(alpha3) + self.L2 * np.sin(alpha3 + beta3), 1])
        self.P32 = P32_vec

        # Décalage pour centrer dans la fenêtre Pygame
        center_x = self.width / 2
        center_y = self.height / 2

        # Ajout du décalage homogène (en ajoutant la composante z = 1 pour les coordonnées homogènes)
        self.P10[:2] += np.array([center_x, center_y])
        self.P11[:2] += np.array([center_x, center_y])
        self.P12[:2] += np.array([center_x, center_y])
        self.P20[:2] += np.array([center_x, center_y])
        self.P21[:2] += np.array([center_x, center_y])
        self.P22[:2] += np.array([center_x, center_y])
        self.P30[:2] += np.array([center_x, center_y])
        self.P31[:2] += np.array([center_x, center_y])
        self.P32[:2] += np.array([center_x, center_y])





    def move_effector(self, dx, dy):
        # Déplacer l'effecteur avec les touches clavier
        self.pos_eff[0] += dx
        self.pos_eff[1] += dy



    def traceRobot(self):
        # Affichage du robot avec Pygame
        self.window.fill((232, 220, 202))  # Réinitialiser la fenêtre

        # Tracer bras 1
        color1 = (random.random(), random.random(), random.random())
        pygame.draw.lines(self.window, color1, False, [(self.P10[0] * self.scale, self.P10[1] * self.scale),
                                                    (self.P11[0] * self.scale, self.P11[1] * self.scale),
                                                    (self.P12[0] * self.scale, self.P12[1] * self.scale)], 2)

        # Tracer bras 2
        color2 = (random.random(), random.random(), random.random())
        pygame.draw.lines(self.window, color2, False, [(self.P20[0] * self.scale, self.P20[1] * self.scale),
                                                    (self.P21[0] * self.scale, self.P21[1] * self.scale),
                                                    (self.P22[0] * self.scale, self.P22[1] * self.scale)], 2)

        # Tracer bras 3
        color3 = (random.random(), random.random(), random.random())
        pygame.draw.lines(self.window, color3, False, [(self.P30[0] * self.scale, self.P30[1] * self.scale),
                                                    (self.P31[0] * self.scale, self.P31[1] * self.scale),
                                                    (self.P32[0] * self.scale, self.P32[1] * self.scale)], 2)

        # Tracer effecteur (triangle)
        pygame.draw.polygon(self.window, (0, 0, 0),
                            [(self.P12[0] * self.scale, self.P12[1] * self.scale),
                            (self.P22[0] * self.scale, self.P22[1] * self.scale),
                            (self.P32[0] * self.scale, self.P32[1] * self.scale)], 2)

        # Mettre à jour l'affichage
        pygame.display.update()

    def runPygame(self):
        # Boucle principale de Pygame
        running = True
        while running:
            self.window.fill((232, 220, 202))  # Réinitialiser la fenêtre
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Gestion des entrées clavier
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]:
                self.move_effector(0, -0.01)  # Déplacer l'effecteur vers le haut
            if keys[pygame.K_DOWN]:
                self.move_effector(0, 0.01)  # Déplacer l'effecteur vers le bas
            if keys[pygame.K_LEFT]:
                self.move_effector(-0.01, 0)  # Déplacer l'effecteur vers la gauche
            if keys[pygame.K_RIGHT]:
                self.move_effector(0.01, 0)  # Déplacer l'effecteur vers la droite

            # Recalculer la position du robot
            q = self.MGI_analytique()  # Résolution analytique des angles
            self.calculPos(q)

            # Tracer le robot
            self.traceRobot()

            # Mettre à jour l'affichage
            pygame.display.update()
            self.clock.tick(self.FPS)  # Limiter la vitesse de l'animation

        pygame.quit()

        
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


# Code principal pour tester l'animation
def main():
    L1 = 0.10
    L2 = 0.10
    Rb = 0.1322594
    Re = 0.07
    dimensionPlateau = 0.50
    pos_eff = [0.1, -0.02, 0.0]
    q0 = np.array([0, pi / 2, 0, pi / 2, 0, pi / 2])

    robot = Robot(L1, L2, Rb, Re, dimensionPlateau, pos_eff, q0)

    # Exécuter l'animation Pygame
    robot.runPygame()


if __name__ == "__main__":
    main()
