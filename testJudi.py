import pygame
import numpy as np
import math
import tkinter as tk #Permet de demander à l'utilisateur de rentrer des valeurs
#On n'utilise pas input car la fenêtre est bloquante dans pygame, contrairement à tkinter
from tkinter import simpledialog


# Constantes pour les dimensions et les couleurs
WIDTH, HEIGHT = 800, 800
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

class Robot3RRR:
    def __init__(self, x=0, y=0, theta=0, L1= 80, L2=80, r=40, R=130):
        self.x = x
        self.y = y
        self.theta = theta
        self.L1 = L1
        self.L2 = L2
        self.pos_eff = [x, y, theta]  # Position de l'effecteur
        #Calcule les coordonnées des 3 points fixes Ai sur un cercle de rayon R, espacés de 120°:
        self.Ai_list = [(R * np.cos(2 * i * np.pi / 3), R * np.sin(2 * i * np.pi / 3)) for i in range(3)]
        #Coordonnées locales des points Pi de la plateforme mobile, aussi espacés de 120°:
        self.Pi_local = [(r * np.cos(2 * i * np.pi / 3), r * np.sin(2 * i * np.pi / 3)) for i in range(3)]
        self.trajectory = [] #stocke les positions successives du centre de l’effecteur
        self.tracing_enabled = False #booléen pour activer/désactiver le tracé
        self.Rb = R  # Rayon de la base fixe
        self.Re = r  # Rayon de la plateforme mobile
        # Liste des positions cibles à atteindre (en mètres)
        self.positions_cibles = [(30, 30), (-40, -15), (0, 30), (0, 0)]
        # Variables de contrôle du mode automatique
        self.mode_suivi = False
        self.index_cible = 0

    #Fonction rotation : applique une rotation 2D autour de l'origine à un point donné
    def rotate(self, point, angle):
        x, y = point
        return (x * np.cos(angle) - y * np.sin(angle),
                x * np.sin(angle) + y * np.cos(angle))

    #Fonction pour convertir les coordonnées en coordonnées écran :
    #Convertit des coordonnées cartésiennes vers les coordonnées écran Pygame :
    #(0,0) est au centre de l'écran
    #L’axe y est inversé pour correspondre au haut de l’écran
    def to_screen(self, x, y):
        return int(WIDTH / 2 + x), int(HEIGHT / 2 - y)

    #Fonction pour vérifier qu'on est bien dans la zone de travail :
    #Vérifie si la position (x, y) de l’effecteur est atteignable
    def is_valid_position(self, x, y):
        Pi_global = [(x + self.rotate(p, self.theta)[0], y + self.rotate(p, self.theta)[1]) for p in self.Pi_local]
        #Calcule de la distance entre chaque Ai et son Pi :
        for Ai, Pi in zip(self.Ai_list, Pi_global):
            dx = Pi[0] - Ai[0]
            dy = Pi[1] - Ai[1]
            d = np.hypot(dx, dy)
            if d > self.L1 + self.L2:
                return False #Position non atteignable
        return True #Position atteignable

    def MGI_analytique(self):
        """
        Fonction MGI : calcule les angles α et β des 3 bras pour atteindre une position donnée de l’effecteur
        Ce que la fonction réalise :
        - Transformation de l’effecteur R_E -> R_0
        - Pour chaque bras, transformation R_i -> R_0
        - Calcule les coordonnées de Ei dans R_i
        - Applique la formule du robot 2R plan (triangle de cosinus) pour trouver β puis α.
        """
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
            if abs(aux) <= 1:
                beta = np.arccos(aux)
            else:
                raise ValueError(f"Position inatteignable pour le bras {i+1} : aux = {aux:.4f}")


            alpha = np.arctan2(y, x) - np.arctan2(self.L2 * np.sin(beta), self.L1 + self.L2 * np.cos(beta))

            q.append(alpha)
            q.append(beta)

        return np.array(q)

    
    def solve_eq_NL(self, q):
        """
        Fonction qui vérifie la solution avec des équations non linéaires  
        Données les angles q = [α1, β1, α2, β2, α3, β3], vérifie qu’ils mènent bien aux bons points Pi.
        Pour chaque bras :
        - Calcule le point Bi atteint avec ces angles.
        - Compare avec la position cible Ei.
        - Renvoie la différence (erreur) sous forme d’un vecteur.
        """
        if self.check_extension(q):
            print("Il y a une singularité: extension")
        # Variables globales
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
            PEi_E = np.array([self.Re * np.cos(ang2[i]), self.Re * np.sin(ang2[i]), 1])

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
            F.append(PBi[0] - PEi_0[0])  # x
            F.append(PBi[1] - PEi_0[1])  # y

        return np.array(F)
    
    #Fonction pour détecter les singularités
    def check_extension(self, q, tol=0.001):
        # Vérifie si les bras sont presque en extension (β ≈ 0 avec tolérance)
        return np.all(np.abs(q[1::2]) < tol)

    
    def check_singularite_parallele(self, q):
        gamma = []
        d = []

        # Angles AiBi
        Pi_global = [(self.x + self.rotate(p, self.theta)[0], self.y + self.rotate(p, self.theta)[1]) for p in self.Pi_local]
        
        for Ai, Pi in zip(self.Ai_list, Pi_global):
            dx = Pi[0] - Ai[0]
            dy = Pi[1] - Ai[1]
            gamma_i = np.arctan2(dy, dx)
            d_i = (Pi[0] - self.x) * (-np.sin(gamma_i)) + (Pi[1] - self.y) * (np.cos(gamma_i))
            gamma.append(gamma_i)
            d.append(d_i)

        # Construction de la matrice A
        A = np.array([[np.cos(g), np.sin(g), di] for g, di in zip(gamma, d)])

        # Déterminant
        return np.abs(np.linalg.det(A)) < 1e-3  # Tolérance proche de 0

    
    def move_to(self, x, y, theta):
        """
        Fonction pour déplacer le robot vers une position donnée  
        Ce que fait la fonction :
        - Change la position cible de l’effecteur.
        - Calcule les angles avec MGI.
        - Vérifie que ces angles mènent à la bonne position avec solve_eq_NL.
        - Si oui, met à jour la position du robot.
        """
        self.pos_eff = [x, y, theta]
        try:
            q = self.MGI_analytique()
            #ajout d'une tolérance parce qu'il est difficile d'avoir absolument solve_eq_NL(q) == 0 
            if np.linalg.norm(self.solve_eq_NL(q)) < 1e-3:
                self.x, self.y, self.theta = x, y, theta
            else:
                print("Position non atteignable (erreur > tolérance)")
        except ValueError as e:
            print(f"Position interdite : {e}")

    def draw(self, win, font):
        """
        Affiche tout dans la fenêtre Pygame :  
        - Bras articulés (Ai → Bi → Pi)
        - Cercles pour les articulations
        - Triangle de l’effecteur
        - Trajectoire si activée
        - Texte des coordonnées et orientation
        - Calcul les α à afficher (les theta_values)
        """
        win.fill(WHITE)

        # Calcul des positions globales des points Pi (effecteur)
        Pi_global = [(self.x + self.rotate(p, self.theta)[0], self.y + self.rotate(p, self.theta)[1]) for p in self.Pi_local]

        # Dessin des segments du robot
        for Ai, Pi in zip(self.Ai_list, Pi_global):
            dx = Pi[0] - Ai[0]
            dy = Pi[1] - Ai[1]
            d = np.hypot(dx, dy)
            a = np.arccos((self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d))
            phi = np.arctan2(dy, dx)
            theta_i = phi - a
            Bi = (Ai[0] + self.L1 * np.cos(theta_i), Ai[1] + self.L1 * np.sin(theta_i))

            # Coordonnées écran
            Ai_s = self.to_screen(*Ai)
            Bi_s = self.to_screen(*Bi)
            Pi_s = self.to_screen(*Pi)

            pygame.draw.line(win, BLACK, Ai_s, Bi_s, 3)
            pygame.draw.line(win, BLUE, Bi_s, Pi_s, 3)
            pygame.draw.circle(win, RED, Ai_s, 5)
            pygame.draw.circle(win, GREEN, Bi_s, 5)
            pygame.draw.circle(win, RED, Pi_s, 5)

        # Triangle effecteur
        triangle_points = [self.to_screen(*P) for P in Pi_global]
        pygame.draw.polygon(win, (150, 200, 255), triangle_points, width=3)

        # Centre effecteur (moyenne des Pi)
        center_x = sum([p[0] for p in Pi_global]) / 3
        center_y = sum([p[1] for p in Pi_global]) / 3
        center_screen = self.to_screen(center_x, center_y)

        # Ajout du centre à la trajectoire si le traçage est activé (en coordonnées monde uniquement)
        if self.tracing_enabled:
            if abs(center_x) < 1000 and abs(center_y) < 1000:  # garde-fou
                if not self.trajectory or (abs(center_x - self.trajectory[-1][0]) > 0.1 or abs(center_y - self.trajectory[-1][1]) > 0.1):
                    self.trajectory.append((center_x, center_y))

        # Dessin de la trajectoire
        if len(self.trajectory) > 1:
            points_trajectoire = [self.to_screen(*P) for P in self.trajectory]
            pygame.draw.lines(win, (255, 0, 255), False, points_trajectoire, 3)

        # Calcul des angles
        q = self.MGI_analytique()
        alpha_vals = q[0::2]
        beta_vals = q[1::2]

        # Affichage infos
        text_pos = font.render(f"Coordonnées: ({self.x:.2f}, {self.y:.2f})", True, BLACK)
        text_theta = font.render(f"Orientation: {np.degrees(self.theta):.1f}°", True, BLACK)
        win.blit(text_pos, (10, 10))
        win.blit(text_theta, (10, 40))

        for i, (alpha, beta) in enumerate(zip(alpha_vals, beta_vals)):
            angle_text = font.render(f"Bras {i+1}: α{i+1}={np.degrees(alpha):.1f}°, β{i+1}={np.degrees(beta):.1f}°", True, (0, 0, 150))
            win.blit(angle_text, (10, 80 + i * 30))

        # Détection de singularités
        if self.check_extension(q):
            sing_text = font.render("Singularité : bras en extension", True, (255, 0, 0))
            win.blit(sing_text, (10, 180))

        if self.check_singularite_parallele(q):
            sing_text2 = font.render("Singularité : configuration parallèle", True, (255, 100, 0))
            win.blit(sing_text2, (10, 210))

        # Flèche d’orientation
        longueur_fleche = 25
        fx = center_x + longueur_fleche * np.cos(self.theta)
        fy = center_y + longueur_fleche * np.sin(self.theta)
        pygame.draw.line(win, BLACK, self.to_screen(center_x, center_y), self.to_screen(fx, fy), 3)

        pygame.display.update()

def generer_trajectory_cercle(xc, yc, rayon, N_points=100):
        return [(xc + rayon * np.cos(t), yc + rayon * np.sin(t)) for t in np.linspace(0, 2 * np.pi, N_points)]

def main():
    pygame.init()
    win = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Simulation Robot 3RRR")
    clock = pygame.time.Clock()
    FPS = 60

    # Initialisation de la police pour afficher le texte
    font = pygame.font.SysFont(None, 36)

    robot = Robot3RRR()
    
    run = True
    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t:
                    robot.tracing_enabled = not robot.tracing_enabled
                elif event.key == pygame.K_m:
                    robot.mode_suivi = True
                    index_cible = 0
                    print("Mode suivi automatique activé.")

        keys = pygame.key.get_pressed()
        new_x, new_y = robot.x, robot.y
        if keys[pygame.K_LEFT]:
            new_x -= 1
        if keys[pygame.K_RIGHT]:
            new_x += 1
        if keys[pygame.K_UP]:
            new_y += 1
        if keys[pygame.K_DOWN]:
            new_y -= 1
        if keys[pygame.K_q]:
            robot.theta += np.radians(2)
        if keys[pygame.K_e]:
            robot.theta -= np.radians(2)


        # Vérification de la validité de la nouvelle position
        if robot.is_valid_position(new_x, new_y):
            robot.move_to(new_x, new_y, robot.theta)



        if robot.mode_suivi and robot.index_cible < len(robot.positions_cibles):
            x_cible, y_cible = robot.positions_cibles[robot.index_cible]

            dx = x_cible - robot.x
            dy = y_cible - robot.y
            dist = math.hypot(dx, dy)

            if dist > 0.5:  # tolérance
                pas = 0.8  # vitesse de déplacement
                new_x = robot.x + pas * dx / dist
                new_y = robot.y + pas * dy / dist

                if robot.is_valid_position(new_x, new_y):
                    robot.move_to(new_x, new_y, robot.theta)

                    # Ajouter à la trajectoire uniquement si c'est un nouveau point
                    if not robot.trajectory or (abs(new_x - robot.trajectory[-1][0]) > 0.1 or abs(new_y - robot.trajectory[-1][1]) > 0.1):
                        robot.trajectory.append((new_x, new_y))
            else:
                robot.index_cible += 1
                print(f"Cible {robot.index_cible} atteinte.")
                if robot.index_cible == len(robot.positions_cibles):
                    print("Toutes les cibles ont été atteintes.")
                    robot.mode_suivi = False

        robot.draw(win, font)
        

    pygame.quit()

if __name__ == "__main__":
    main()
