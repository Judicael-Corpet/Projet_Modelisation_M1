import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import pandas as pd
import seaborn as sns

def est_position_valide(x, y, theta, L1, L2, Re, Rb, seuil_singularite=5):
    # On autorise les positions un peu en dehors de Rb pour ne pas trop restreindre
    if np.hypot(x, y) > Rb + 50:  # marge tolérante
        return False, False

    theta_bases = [0, 2*np.pi/3, 4*np.pi/3]
    theta_plates = [theta, theta + 2*np.pi/3, theta + 4*np.pi/3]
    proche_singularite = False

    for i in range(3):
        Bxi = Rb * np.cos(theta_bases[i])
        Byi = Rb * np.sin(theta_bases[i])
        Exi = x + Re * np.cos(theta_plates[i])
        Eyi = y + Re * np.sin(theta_plates[i])
        dx = Exi - Bxi
        dy = Eyi - Byi
        distance = np.hypot(dx, dy)

        # Ce test est très restrictif, on le supprime pour tester
        # vecteur_bras = np.array([dx, dy])
        # vecteur_moteur_sortant = np.array([Bxi, Byi])
        # if np.dot(vecteur_bras, vecteur_moteur_sortant) < 0:
        #     return False, False

        if not (abs(L1 - L2) <= distance <= (L1 + L2)):
            return False, False

        if abs(distance - (L1 + L2)) < seuil_singularite or abs(distance - abs(L1 - L2)) < seuil_singularite:
            proche_singularite = True

    return True, proche_singularite


def trouver_config_maximale():
    Rb = 150
    resolution = 5
    limite = 200

    meilleures_config = None
    max_aire_sans_sing = -np.inf
    resultats = []
    points_max = []
    points_singuliers_max = []


    L1 = 100
    L2 = 100
    for cote_E in range(90, 160, 10):  # côté effecteur
        Re = cote_E / np.sqrt(3)
        X = np.arange(-limite, limite + 1, resolution)
        Y = np.arange(-limite, limite + 1, resolution)
        points_valides = []
        points_singuliers = []

        for x in X:
            for y in Y:
                valide, singulier = est_position_valide(x, y, 0, L1, L2, Re, Rb)
                if valide and not singulier:
                    points_valides.append((x, y))
                elif valide and singulier:
                    points_singuliers.append((x, y))

        aire_sans_sing = len(points_valides) * resolution**2

        resultats.append(((L1, L2, cote_E), aire_sans_sing, len(points_singuliers)))

        if aire_sans_sing > max_aire_sans_sing:
            max_aire_sans_sing = aire_sans_sing
            meilleures_config = (L1, L2, cote_E)
            points_max = points_valides.copy()
            points_singuliers_max = points_singuliers.copy()

    if meilleures_config is not None and points_max:
        print(f"\n✔️ Meilleure configuration (aire sans singularité) : L1={meilleures_config[0]}, L2={meilleures_config[1]}, côté effecteur={meilleures_config[2]} mm")
        print(f"Aire atteignable sans singularité : {len(points_max) * resolution**2} mm²")
        print(f"Points proches de singularité (exclus) : {len(points_singuliers_max)}")

        x_vals, y_vals = zip(*points_max)
        plt.figure(figsize=(8, 8))
        plt.scatter(x_vals, y_vals, s=1, color='blue', label="Valides sans singularité")
        if points_singuliers_max:
            x_sing, y_sing = zip(*points_singuliers_max)
            plt.scatter(x_sing, y_sing, s=1, color='red', label="Singularités")

        cercle = plt.Circle((0, 0), Rb, color='green', fill=False, linestyle='--', label='Limite Rb')
        plt.gca().add_patch(cercle)

        plt.title("Aire de travail maximale (sans singularités) du robot 3RRR")
        plt.xlabel("x (mm)")
        plt.ylabel("y (mm)")
        plt.axis("equal")
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("❌ Aucune configuration valide trouvée (ou aucun point sans singularité disponible).")

    return resultats

def tracer_aire_vs_effecteur(resultats):
    df = pd.DataFrame(resultats, columns=["params", "aire", "nb_singulier"])
    df["cote_E"] = df["params"].apply(lambda x: x[2])

    # Grouper par dimension de l'effecteur et calculer la moyenne de l'aire
    df_group = df.groupby("cote_E").agg(
        aire_moyenne=("aire", "mean"),
        nb_singulier=("nb_singulier", "mean")
    ).reset_index()

    plt.figure(figsize=(10, 5))
    plt.plot(df_group["cote_E"], df_group["aire_moyenne"], marker="o", color='blue')
    plt.title("Aire de travail sans singularité en fonction du côté de l'effecteur")
    plt.xlabel("Côté de l'effecteur (mm)")
    plt.ylabel("Aire sans singularité (mm²)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Optionnel : graphique du nombre de points proches de singularité
    plt.figure(figsize=(10, 5))
    plt.plot(df_group["cote_E"], df_group["nb_singulier"], marker="x", color='red')
    plt.title("Nombre moyen de points proches de singularité vs côté de l'effecteur")
    plt.xlabel("Côté de l'effecteur (mm)")
    plt.ylabel("Nombre de points proches de singularité")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    resultats = trouver_config_maximale()
    tracer_aire_vs_effecteur(resultats)  # <- nouvelle fonction

