import numpy as np

# Variables globales (à définir avant appel)
L1 = 0.0  # longueur du premier segment
L2 = 0.0  # longueur du deuxième segment
Rb = 0.0  # rayon de base
Re = 0.0  # rayon de l'effecteur

def MGI_analytique(eff):
    global L1, L2, Rb, Re

    # Matrice de rotation et transformation homogène de l'effecteur
    theta = eff[2]
    RotEff = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    Transl = np.array([[eff[0]], [eff[1]]])
    THEff = np.block([
        [RotEff, Transl],
        [np.zeros((1, 2)), np.array([[1]])]
    ])

    # Angles R_i par rapport à R_0
    ang1 = [0, 2*np.pi/3, 4*np.pi/3]

    # Angles des positions O_i et E_i
    ang2 = [-np.pi/2, np.pi/6, 5*np.pi/6]

    q = []

    for i in range(3):
        # Matrice de rotation et transformation homogène de R_i par rapport à R_0
        Rot = np.array([
            [np.cos(ang1[i]), -np.sin(ang1[i])],
            [np.sin(ang1[i]),  np.cos(ang1[i])]
        ])
        TH = np.block([
            [Rot, np.array([[Rb*np.cos(ang2[i])], [Rb*np.sin(ang2[i])]])],
            [np.zeros((1, 2)), np.array([[1]])]
        ])

        # Position des points E_i dans R_E
        PEi_E = np.array([[Re*np.cos(ang2[i])], [Re*np.sin(ang2[i])], [1]])

        # Position des points E_i de l'effecteur dans R_0
        PEi_0 = THEff @ PEi_E
import numpy as np

def MGI_analytique(eff):
    # Variables globales
    global L1, L2, Rb, Re

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
        TH = np.block([[Rot, np.array([Rb * np.cos(ang2[i]), Rb * np.sin(ang2[i])]).reshape(-1, 1)], [0, 0, 1]])

        # Position des points E_i dans R_E
        PEi_E = np.array([Re * np.cos(ang2[i]), Re * np.sin(ang2[i]), 1])

        # Position des trois points E_i de l'effecteur dans R_0
        PEi_0 = THEff @ PEi_E

        # Position des points effecteur E_i dans les repères des robots R_i
        PEi_i = np.linalg.inv(TH) @ PEi_0

        x, y = PEi_i[0, 0], PEi_i[1, 0]

        # MGI du bras 2R plan
        aux = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(aux) < 1:
            beta = np.arccos(aux)  # solution coude en bas
        else:
            beta = 0
            print("Problème d'atteignabilité")

        alpha = np.arctan2(y, x) - np.arctan2(L2*np.sin(beta), L1 + L2*np.cos(beta))
        q.append([alpha, beta])

    return np.array(q)
        # Position des points effecteur E_i dans les repères des robots R_i
        PEi_i = np.linalg.inv(TH) @ PEi_0

        # MGI 2R plan
        x = PEi_i[0]
        y = PEi_i[1]

        aux = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(aux) < 1:
            beta = np.arccos(aux)  # Solution coude en haut
        else:
            beta = 0
            print("Problème d'atteignabilité")

        alpha = np.arctan2(y, x) - np.arctan2(L2 * np.sin(beta), L1 + L2 * np.cos(beta))

        q.append([alpha, beta])

    return np.array(q)
