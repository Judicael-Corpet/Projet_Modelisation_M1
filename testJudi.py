import pygame
import numpy as np
import math
import tkinter as tk #Permet de demander à l'utilisateur de rentrer des valeurs
#On n'utilise pas input car la fenêtre est bloquante dans pygame, contrairement à tkinter
from tkinter import simpledialog
from scipy.optimize import root
from scipy.optimize import minimize


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
        self.q = self.MGI_analytique()  # initialisation des angles moteurs
        self.ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]      # Rotation des bras par rapport à R0
        self.ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]  # Position des bras sur le cercle



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

    def MGD(self, q):
        """
        Calcule la position (x, y, θ) de l'effecteur à partir des angles αi et βi
        Résout un problème non linéaire pour retrouver la pose de l'effecteur.
        """
        from scipy.optimize import minimize

        # Fonction objectif : somme des erreurs entre les points Bi et Ei
        def erreur_pose(pos_eff):
            x, y, theta = pos_eff
            self.pos_eff = [x, y, theta]
            try:
                err = self.solve_eq_NL(q)
                return np.linalg.norm(err)
            except:
                return 1e6  # valeur très grande si échec

        # Estimation initiale = pose actuelle
        pos0 = self.pos_eff

        result = minimize(erreur_pose, pos0, method='BFGS')

        if result.success and result.fun < 1e-3:
            x, y, theta = result.x
            self.x, self.y, self.theta = x, y, theta
            self.pos_eff = [x, y, theta]
            self.q = q  #  mise à jour de la configuration
            return True

        else:
            print("MGD : Échec du calcul direct")
            return False

    
    
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

    def update_from_q(self):
        """Recalcule la position de l'effecteur uniquement pour mise à jour du dessin"""
        # Essaie de trouver une pose qui correspond aux q actuels
        self.MGD(self.q)


    def resoudre_depuis_alpha_fixe(self, index_alpha_fixe, valeur_alpha):

        # Angles fixes initiaux
        q0 = self.q.copy()
        autres = [i for i in range(3) if i != index_alpha_fixe]

        def equations(variables):
            alpha = [0.0, 0.0, 0.0]
            beta = [0.0, 0.0, 0.0]

            alpha[index_alpha_fixe] = valeur_alpha
            alpha[autres[0]] = variables[0]
            alpha[autres[1]] = variables[1]

            beta = variables[2:5]
            x, y, theta = variables[5:]

            self.pos_eff = [x, y, theta]

            F = []
            Pi_local = self.Pi_local

            # Coordonnées des Pi globales
            Pi_global = [(x + self.rotate(p, theta)[0], y + self.rotate(p, theta)[1]) for p in Pi_local]

            # Calcul des erreurs de position entre les BiPi reconstruits et les Pi globaux
            for i in range(3):
                Ai = self.Ai_list[i]
                alpha_i = alpha[i]
                beta_i = beta[i]

                Bi = (
                    Ai[0] + self.L1 * np.cos(alpha_i),
                    Ai[1] + self.L1 * np.sin(alpha_i)
                )

                Pi_calc = (
                    Bi[0] + self.L2 * np.cos(alpha_i + beta_i),
                    Bi[1] + self.L2 * np.sin(alpha_i + beta_i)
                )

                Pi_target = Pi_global[i]
                F.append(Pi_calc[0] - Pi_target[0])
                F.append(Pi_calc[1] - Pi_target[1])

            # Contraintes géométriques pour maintenir l'effecteur rigide (triangle équilatéral)
            Lp = 2 * self.Re * np.sin(np.pi / 3)
            d12 = np.linalg.norm(np.array(Pi_global[0]) - np.array(Pi_global[1])) - Lp
            d23 = np.linalg.norm(np.array(Pi_global[1]) - np.array(Pi_global[2])) - Lp
            F.append(d12)
            F.append(d23)

            return np.array(F)

        def objective(variables):
            F = equations(variables)
            return np.linalg.norm(F)

        # Initialisation du guess : à partir de q actuel
        alpha_j = q0[2 * autres[0]]
        alpha_k = q0[2 * autres[1]]
        beta_vals = [q0[1], q0[3], q0[5]]
        x, y, theta = self.pos_eff
        guess = [alpha_j, alpha_k] + beta_vals + [x, y, theta]

        print("🧠 Point de départ (guess) =", guess)
        # 🔁 Si la pose n’a pas été mise à jour correctement, corrige-la manuellement
        if self.pos_eff == [0, 0, 0]:
            self.pos_eff = [self.x, self.y, self.theta]


        result = minimize(objective, guess, method='BFGS', options={'gtol': 1e-6, 'maxiter': 500})

        if result.fun < 1e-3:

            vars = result.x
            alpha_j, alpha_k = vars[0], vars[1]
            beta = vars[2:5]
            x, y, theta = vars[5:]

            # Reconstruction de la configuration complète
            alpha = [0.0, 0.0, 0.0]
            alpha[index_alpha_fixe] = valeur_alpha
            alpha[autres[0]] = alpha_j
            alpha[autres[1]] = alpha_k

            self.q = np.array([
                alpha[0], beta[0],
                alpha[1], beta[1],
                alpha[2], beta[2],
            ])
            self.pos_eff = [x, y, theta]
            self.x, self.y, self.theta = x, y, theta

            print(f"✅ Résolution OK — α{index_alpha_fixe+1} fixé = {np.degrees(valeur_alpha):.1f}°")
        else:
            print("⚠️ Échec résolution avec alpha fixé")
            print("α fixé =", valeur_alpha, " (index =", index_alpha_fixe, ")")
            print("Dernier guess =", guess)
            print("Erreur finale =", result.fun)




    def recalculer_beta_et_pose(self):
        """
        À partir des angles α1, α2, α3 (commandés), recalcule les βi (passifs)
        et déduit la position (x, y, θ) de la plateforme mobile.
        Met à jour self.q et self.pos_eff.
        """
        ang1 = self.ang1
        ang2 = self.ang2

        alpha = self.q[0::2]  # angles moteurs
        beta = []

        # Étape 1 : calcul des Bi (points extrémité bras moteurs) pour chaque αi
        Bi_list = []
        for i in range(3):
            Oi = np.array([self.Rb * np.cos(ang2[i]), self.Rb * np.sin(ang2[i])])
            Rot = np.array([[np.cos(ang1[i]), -np.sin(ang1[i])],
                            [np.sin(ang1[i]),  np.cos(ang1[i])]])
            Bi_local = np.array([self.L1 * np.cos(alpha[i]), self.L1 * np.sin(alpha[i])])
            Bi = Oi + Rot @ Bi_local
            Bi_list.append(Bi)

        # Étape 2 : trouver la transformation rigide (x, y, θ) qui aligne les Pi_local (rigide) aux extrémités Bi + L2 vecteurs
        # On suppose que la plateforme mobile a une forme fixe (Pi_local), et on l'aligne aux extrémités atteintes par les bras

        # Pour chaque bras, le point Pi doit être à une distance L2 de Bi
        # On cherche la meilleure pose (x, y, θ) qui aligne Pi_local avec ces cibles

        # Pour cela, on utilise la méthode d'alignement de points rigides (Kabsch) entre Pi_local et Pi_cible = Bi + L2 * direction arbitraire (initialisation)

        # Initialisation approximative des Pi_cible = Bi + direction initiale
        Pi_init = []
        for Bi in Bi_list:
            Pi_init.append(Bi + np.array([self.L2, 0]))  # direction initiale arbitraire

        # Appliquer Kabsch pour aligner Pi_local sur Pi_init
        Pi_local_np = np.array(self.Pi_local)
        Pi_init_np = np.array(Pi_init)

        centroid_local = Pi_local_np.mean(axis=0)
        centroid_target = Pi_init_np.mean(axis=0)

        A = Pi_local_np - centroid_local
        B = Pi_init_np - centroid_target

        H = A.T @ B
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        theta = np.arctan2(R[1, 0], R[0, 0])
        translation = centroid_target - R @ centroid_local
        x, y = translation

        self.x = x
        self.y = y
        self.theta = theta
        self.pos_eff = [x, y, theta]




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

        q = self.q  # utiliser les angles moteurs modifiés
        alpha_vals = q[0::2]
        beta_vals = q[1::2]

        # Angles R_i par rapport à R_0
        ang1 = [0, 2 * np.pi / 3, 4 * np.pi / 3]
        ang2 = [-np.pi / 2, np.pi / 6, 5 * np.pi / 6]

        Pi_global = []
        Bi_global = []

        for i in range(3):
            # THRi: transformation de chaque base locale
            Rot = np.array([
                [np.cos(ang1[i]), -np.sin(ang1[i])],
                [np.sin(ang1[i]),  np.cos(ang1[i])]
            ])
            Oi = np.array([self.Rb * np.cos(ang2[i]), self.Rb * np.sin(ang2[i])])

            # Calcul Bi et Pi dans le repère du bras i
            alpha = alpha_vals[i]
            beta = beta_vals[i]
            Bi_local = np.array([self.L1 * np.cos(alpha), self.L1 * np.sin(alpha)])
            Pi_local = Bi_local + np.array([self.L2 * np.cos(alpha + beta), self.L2 * np.sin(alpha + beta)])

            # Transformation vers R0
            Bi = Oi + Rot @ Bi_local
            Pi = Oi + Rot @ Pi_local

            Ai_s = self.to_screen(*Oi)
            Bi_s = self.to_screen(*Bi)
            Pi_s = self.to_screen(*Pi)

            pygame.draw.line(win, BLACK, Ai_s, Bi_s, 3)
            pygame.draw.line(win, BLUE, Bi_s, Pi_s, 3)
            pygame.draw.circle(win, RED, Ai_s, 5)
            pygame.draw.circle(win, GREEN, Bi_s, 5)
            pygame.draw.circle(win, RED, Pi_s, 5)

            Pi_global.append(Pi)


            Ai_s = self.to_screen(*Oi)
            Bi_s = self.to_screen(*Bi)
            Pi_s = self.to_screen(*Pi)

            pygame.draw.line(win, BLACK, Ai_s, Bi_s, 3)
            pygame.draw.line(win, BLUE, Bi_s, Pi_s, 3)
            pygame.draw.circle(win, RED, Ai_s, 5)
            pygame.draw.circle(win, GREEN, Bi_s, 5)
            pygame.draw.circle(win, RED, Pi_s, 5)

        # Dessin du triangle effecteur
        triangle_points = [self.to_screen(*P) for P in Pi_global]
        pygame.draw.polygon(win, (150, 200, 255), triangle_points, width=3)

        # Centre de la plateforme (moyenne des Pi)
        center_x = sum(p[0] for p in Pi_global) / 3
        center_y = sum(p[1] for p in Pi_global) / 3
        center_screen = self.to_screen(center_x, center_y)

        if self.tracing_enabled:
            if not self.trajectory or np.linalg.norm(np.array((center_x, center_y)) - np.array(self.trajectory[-1])) > 0.1:
                self.trajectory.append((center_x, center_y))

        if len(self.trajectory) > 1:
            traj = [self.to_screen(*p) for p in self.trajectory]
            pygame.draw.lines(win, (255, 0, 255), False, traj, 2)

        # Texte
        coord_text = font.render(f"Centre: ({center_x:.1f}, {center_y:.1f})", True, BLACK)
        win.blit(coord_text, (10, 10))

        for i, (alpha, beta) in enumerate(zip(alpha_vals, beta_vals)):
            angle_text = font.render(f"Bras {i+1}: α={np.degrees(alpha):.1f}°, β={np.degrees(beta):.1f}°", True, (0, 0, 150))
            win.blit(angle_text, (10, 50 + i * 30))

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
    robot.q = robot.MGI_analytique()
    robot.pos_eff = [robot.x, robot.y, robot.theta]

    
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

        # COMMANDE α1 : touches A (augmenter), Z (diminuer)
        if keys[pygame.K_a]:
            robot.q[0] += np.radians(2)
            robot.resoudre_depuis_alpha_fixe(0, robot.q[0])

        elif keys[pygame.K_z]:
            robot.q[0] -= np.radians(2)
            robot.resoudre_depuis_alpha_fixe(0, robot.q[0])

        # COMMANDE α2 : touches S / X
        if keys[pygame.K_s]:
            robot.q[2] += np.radians(2)
            robot.resoudre_depuis_alpha_fixe(1, robot.q[2])
        elif keys[pygame.K_x]:
            robot.q[2] -= np.radians(2)
            robot.resoudre_depuis_alpha_fixe(1, robot.q[2])

        # COMMANDE α3 : touches D / C
        if keys[pygame.K_d]:
            robot.q[4] += np.radians(2)
            robot.resoudre_depuis_alpha_fixe(2, robot.q[4])
        elif keys[pygame.K_c]:
            robot.q[4] -= np.radians(2)
            robot.resoudre_depuis_alpha_fixe(2, robot.q[4])



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
