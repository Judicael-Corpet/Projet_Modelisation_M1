import pygame
import sys
from Test_moteurs_solo import Robot3RRR

pygame.init()
win = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Robot 3RRR - Segments rigides")
clock = pygame.time.Clock()

robot = Robot3RRR(L1=1.5, L2=1.5, Rb=2.0, Rp=0.9)

# Contrôle des 3 moteurs
KEYS = {
    pygame.K_z: (0,  0.02),
    pygame.K_s: (0, -0.02),
    pygame.K_e: (1,  0.02),
    pygame.K_d: (1, -0.02),
    pygame.K_r: (2,  0.02),
    pygame.K_f: (2, -0.02),
}

running = True
while running:
    clock.tick(30)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    for key, (idx, delta) in KEYS.items():
        if keys[key]:
            robot.update_angles(idx, delta)

    robot.draw(win)

pygame.quit()
sys.exit()
