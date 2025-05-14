import numpy as np
import pygame
from scipy.optimize import least_squares

BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

class Robot3RRR:
    def __init__(self, L1, L2, Rb, Rp):
        self.L1 = L1
        self.L2 = L2
        self.Rb = Rb
        self.Rp = Rp

        self.Ai_list = self.calcul_Ai()
        self.q_alpha = [0.5, 0.05, 0.5]  # α₁, α₂, α₃
        self.pose = [0.0, 0.0, 0.0]     # X, Y, φ

    def calcul_Ai(self):
        return [
            (self.Rb * np.cos(2 * np.pi * i / 3), self.Rb * np.sin(2 * np.pi * i / 3))
            for i in range(3)
        ]

    def calcul_Bi(self):
        Bi_list = []
        for i in range(3):
            Ai = np.array(self.Ai_list[i])
            alpha = self.q_alpha[i]
            Bi = Ai + self.L1 * np.array([np.cos(alpha), np.sin(alpha)])
            Bi_list.append(Bi)
        return Bi_list

    def Ci_from_pose(self, X, Y, phi):
        Ci_list = []
        for i in range(3):
            angle = phi + 2 * np.pi * i / 3
            Ci = np.array([X, Y]) + self.Rp * np.array([np.cos(angle), np.sin(angle)])
            Ci_list.append(Ci)
        return Ci_list

    def erreur_plateforme(self, pose):
        X, Y, phi = pose
        Bi_list = self.calcul_Bi()
        Ci_list = self.Ci_from_pose(X, Y, phi)
        erreurs = []
        for Bi, Ci in zip(Bi_list, Ci_list):
            d = np.linalg.norm(Bi - Ci)
            erreurs.append(d - self.L2)
        return erreurs

    def update_angles(self, index, delta):
        self.q_alpha[index] += delta

        # Résolution numérique pour (X, Y, φ) avec méthode plus précise
        res = least_squares(
            self.erreur_plateforme,
            self.pose,
            method='lm',       # méthode de Levenberg-Marquardt (plus stable pour petits systèmes)
            ftol=1e-12,
            xtol=1e-12,
            gtol=1e-12,
            max_nfev=200
        )

        # Vérification stricte : chaque erreur doit être proche de 0
        if res.success and np.all(np.abs(res.fun) < 1e-6):
            self.pose = res.x
        else:
            print("Position non atteignable avec ces angles")
            self.q_alpha[index] -= delta  # rollback en cas d’échec


    def to_screen(self, x, y):
        scale = 100
        x_screen = int(x * scale + 400)
        y_screen = int(-y * scale + 300)
        return (x_screen, y_screen)

    def draw(self, win):
        win.fill((255, 255, 255))
        A_ecran = np.array([400, 300])
        scale = 100

        Bi_list = self.calcul_Bi()
        Ci_list = self.Ci_from_pose(*self.pose)

        # Tracer les bras
        for i in range(3):
            Ai = np.array(self.Ai_list[i])
            Bi = Bi_list[i]
            Ci = Ci_list[i]

            pygame.draw.line(win, BLUE, self.to_screen(*Ai), self.to_screen(*Bi), 4)
            pygame.draw.line(win, RED, self.to_screen(*Bi), self.to_screen(*Ci), 4)

        # Tracer la plateforme
        pygame.draw.polygon(win, GREEN, [self.to_screen(*pt) for pt in Ci_list])
        pygame.draw.polygon(win, (0, 100, 0), [self.to_screen(*pt) for pt in Ci_list], 3)

        # Centre
        pygame.draw.circle(win, (0, 0, 0), self.to_screen(*self.pose[:2]), 5)

        pygame.display.update()
