import pygame
import numpy as np

# Initialisation Pygame
pygame.init()
width, height = 600, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Animation bras à deux pivots")
clock = pygame.time.Clock()

# Paramètres du bras
L1, L2 = 100, 80  # longueurs des segments
origin = (width // 2, height // 2)

# Angles initiaux et finaux en radians
alpha_start = np.radians(30)
beta_start = np.radians(45)
alpha_end = np.radians(90)
beta_end = np.radians(-30)

# Interpolation
steps = 100
frame = 0

def interpolate(a0, a1, t):
    return (1 - t) * a0 + t * a1

def get_joint_positions(alpha, beta):
    x1 = origin[0] + L1 * np.cos(alpha)
    y1 = origin[1] - L1 * np.sin(alpha)
    x2 = x1 + L2 * np.cos(alpha + beta)
    y2 = y1 - L2 * np.sin(alpha + beta)
    return (x1, y1), (x2, y2)

# Boucle principale
running = True
while running:
    t = min(frame / steps, 1.0)  # interpolation factor

    alpha = interpolate(alpha_start, alpha_end, t)
    beta = interpolate(beta_start, beta_end, t)

    joint1, end_effector = get_joint_positions(alpha, beta)

    screen.fill((240, 240, 240))

    # Dessin du bras
    pygame.draw.line(screen, (0, 0, 255), origin, joint1, 5)
    pygame.draw.line(screen, (0, 255, 0), joint1, end_effector, 5)

    pygame.draw.circle(screen, (255, 0, 0), (int(end_effector[0]), int(end_effector[1])), 6)

    pygame.display.flip()
    clock.tick(60)

    frame += 1
    if frame > steps:
        frame = steps  # stop interpolation here

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
