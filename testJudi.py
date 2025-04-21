import pygame
import numpy as np

# Constantes pour les dimensions et les couleurs
WIDTH, HEIGHT = 800, 800
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Paramètres du robot
L1 = 80  # Longueur du premier segment
L2 = 80  # Longueur du second segment
r = 40   # Rayon de la plateforme mobile
R = 130  # Rayon de la base fixe

class Robot3RRR:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.pos_eff = [x, y, theta]  # Position de l'effecteur
        self.Ai_list = [(R * np.cos(2 * i * np.pi / 3), R * np.sin(2 * i * np.pi / 3)) for i in range(3)]
        self.Pi_local = [(r * np.cos(2 * i * np.pi / 3), r * np.sin(2 * i * np.pi / 3)) for i in range(3)]
        self.trajectory = []
        self.tracing_enabled = False
        self.Rb = R  # Rayon de la base fixe
        self.Re = r  # Rayon de la plateforme mobile

    def rotate(self, point, angle):
        x, y = point
        return (x * np.cos(angle) - y * np.sin(angle),
                x * np.sin(angle) + y * np.cos(angle))

    def to_screen(self, x, y):
        return int(WIDTH / 2 + x), int(HEIGHT / 2 - y)

    def is_valid_position(self, x, y):
        Pi_global = [(x + self.rotate(p, self.theta)[0], y + self.rotate(p, self.theta)[1]) for p in self.Pi_local]
        for Ai, Pi in zip(self.Ai_list, Pi_global):
            dx = Pi[0] - Ai[0]
            dy = Pi[1] - Ai[1]
            d = np.hypot(dx, dy)
            if d > L1 + L2:
                return False
        return True

    def MGI_analytique(self):
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

            aux = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
            if abs(aux) <= 1:
                beta = np.arccos(aux)
            else:
                raise ValueError(f"Position inatteignable pour le bras {i+1} : aux = {aux:.4f}")


            alpha = np.arctan2(y, x) - np.arctan2(L2 * np.sin(beta), L1 + L2 * np.cos(beta))

            q.append(alpha)
            q.append(beta)

        return np.array(q)

    def solve_eq_NL(self, q):
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
            PBi = THRi_0 @ np.array([L1 * np.cos(alpha[i]) + L2 * np.cos(alpha[i] + beta[i]),
                                     L1 * np.sin(alpha[i]) + L2 * np.sin(alpha[i] + beta[i]),
                                     1])

            # Les contraintes expriment que B_i doit être confondu avec E_i
            F.append(PBi[0] - PEi_0[0])  # x
            F.append(PBi[1] - PEi_0[1])  # y

        return np.array(F)

    def check_extension(self, q):
        # Vérifie si les bras sont en extension complète
        return np.allclose(q[1::2], 0)

    def move_to(self, x, y, theta):
        self.pos_eff = [x, y, theta]
        q = self.MGI_analytique()
        if np.all(self.solve_eq_NL(q) == 0):
            self.x, self.y, self.theta = x, y, theta
        else:
            print("Position non atteignable")

    def draw(self, win, font):
        win.fill(WHITE)

        # Calcul des positions globales des points Pi
        Pi_global = [(self.x + self.rotate(p, self.theta)[0], self.y + self.rotate(p, self.theta)[1]) for p in self.Pi_local]

        # Dessin des segments du robot
        for Ai, Pi in zip(self.Ai_list, Pi_global):
            # Calcul des positions des points Bi
            dx = Pi[0] - Ai[0]
            dy = Pi[1] - Ai[1]
            d = np.hypot(dx, dy)
            a = np.arccos((L1**2 + d**2 - L2**2) / (2 * L1 * d))
            phi = np.arctan2(dy, dx)
            theta_i = phi - a
            Bi = (Ai[0] + L1 * np.cos(theta_i), Ai[1] + L1 * np.sin(theta_i))

            # Conversion des coordonnées en coordonnées écran
            Ai_s = self.to_screen(*Ai)
            Bi_s = self.to_screen(*Bi)
            Pi_s = self.to_screen(*Pi)

            # Dessin des segments et des articulations
            pygame.draw.line(win, BLACK, Ai_s, Bi_s, 3)
            pygame.draw.line(win, BLUE, Bi_s, Pi_s, 3)
            pygame.draw.circle(win, RED, Ai_s, 5)
            pygame.draw.circle(win, GREEN, Bi_s, 5)
            pygame.draw.circle(win, RED, Pi_s, 5)

        # Dessin du triangle effecteur
        triangle_points = [self.to_screen(*P) for P in Pi_global]
        pygame.draw.polygon(win, (150, 200, 255), triangle_points, width=3)

        # Calcul du centre de l'effecteur
        center_x = sum([p[0] for p in Pi_global]) / 3
        center_y = sum([p[1] for p in Pi_global]) / 3
        center_screen = self.to_screen(center_x, center_y)

        # Ajout du centre à la trajectoire si le traçage est activé
        if self.tracing_enabled:
            self.trajectory.append(center_screen)

        # Dessin de la trajectoire
        if len(self.trajectory) > 1:
            color = RED if self.tracing_enabled else WHITE
            pygame.draw.lines(win, color, False, self.trajectory, 2)

        # Calcul des angles theta pour chaque bras
        q = self.MGI_analytique()
        theta_values = q[0::2]  # Les valeurs de theta sont les éléments pairs de q

        # Affichage des coordonnées de l'effecteur et des angles theta
        coord_text = font.render(f"Coordonnées: ({center_x:.2f}, {center_y:.2f})", True, BLACK)
        text4 = font.render(f"Orientation: {np.degrees(self.theta):.1f}°", True, BLACK)
        win.blit(coord_text, (10, 10))
        win.blit(text4, (10, 70))

        pygame.display.update()

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
                    # Exemple de déplacement vers des coordonnées spécifiques
                    target_x = 28
                    target_y = 38
                    target_theta = 0
                    robot.move_to(target_x, target_y, target_theta)

        keys = pygame.key.get_pressed()
        new_x, new_y = robot.x, robot.y
        if keys[pygame.K_LEFT]:
            new_x -= 2
        if keys[pygame.K_RIGHT]:
            new_x += 2
        if keys[pygame.K_UP]:
            new_y += 2
        if keys[pygame.K_DOWN]:
            new_y -= 2

        # Vérification de la validité de la nouvelle position
        if robot.is_valid_position(new_x, new_y):
            robot.x, robot.y = new_x, new_y

        robot.draw(win, font)

    pygame.quit()

if __name__ == "__main__":
    main()
