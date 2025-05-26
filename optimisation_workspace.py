import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm  # Pour barre de progression
import pandas as pd
import seaborn as sns


def est_position_valide(x, y, theta, L1, L2, Re, Rb):
    theta_bases = [0, 2*np.pi/3, 4*np.pi/3]
    theta_plates = [theta, theta + 2*np.pi/3, theta + 4*np.pi/3]

    for i in range(3):
        Bxi = Rb * np.cos(theta_bases[i])
        Byi = Rb * np.sin(theta_bases[i])

        Exi = x + Re * np.cos(theta_plates[i])
        Eyi = y + Re * np.sin(theta_plates[i])

        dx = Exi - Bxi
        dy = Eyi - Byi
        distance = np.hypot(dx, dy)

        # Condition de réalisabilité d'un triangle
        if not (abs(L1 - L2) <= distance <= (L1 + L2)):
            return False
    return True

def calculer_aire_travail(L1, L2, cote_E, Rb=120, resolution=5, limite=200):
    Re = cote_E / np.sqrt(3)  # rayon du triangle équilatéral inscrit
    theta = 0  # On suppose une orientation fixe ici
    X = np.arange(-limite, limite + 1, resolution)
    Y = np.arange(-limite, limite + 1, resolution)
    aire = 0

    for x in X:
        for y in Y:
            if est_position_valide(x, y, theta, L1, L2, Re, Rb):
                aire += 1
    return aire * resolution**2  # aire en mm²

def trouver_config_maximale():
    Rb = 300  # Rayon de la base fixe
    resolution = 5  # grille de 5 mm
    limite = 200  # espace de recherche

    meilleures_config = None
    max_aire = 0
    resultats = []
    points_max = []

    for L1 in tqdm(range(80, 151, 10)):
        for L2 in range(80, 151, 10):
            for cote_E in range(80, 200, 5):
                Re = cote_E / np.sqrt(3)
                X = np.arange(-limite, limite + 1, resolution)
                Y = np.arange(-limite, limite + 1, resolution)
                points_valides = []

                for x in X:
                    for y in Y:
                        if est_position_valide(x, y, 0, L1, L2, Re, Rb):
                            points_valides.append((x, y))

                aire = len(points_valides) * resolution**2
                resultats.append(((L1, L2, cote_E), aire))

                if aire > max_aire:
                    max_aire = aire
                    meilleures_config = (L1, L2, cote_E)
                    points_max = points_valides.copy()

    if meilleures_config is not None:
        print(f"\n✔️ Meilleure configuration : L1={meilleures_config[0]}, L2={meilleures_config[1]}, côté effecteur={meilleures_config[2]} mm")
        print(f"Aire atteignable maximale : {max_aire} mm² ({max_aire/100:.2f} cm²)")

        # Affichage graphique
        x_vals, y_vals = zip(*points_max)
        plt.figure(figsize=(8, 8))
        plt.scatter(x_vals, y_vals, s=1, color='blue')
        plt.title("Aire de travail maximale du robot 3RRR")
        plt.xlabel("x (mm)")
        plt.ylabel("y (mm)")
        plt.axis("equal")
        plt.grid(True)
        plt.show()
    else:
        print("❌ Aucune configuration valide trouvée.")

    return resultats


def tracer_courbes(resultats):
    df = pd.DataFrame(resultats, columns=["params", "aire"])
    df["L1"] = df["params"].apply(lambda x: x[0])
    df["L2"] = df["params"].apply(lambda x: x[1])
    df["cote_E"] = df["params"].apply(lambda x: x[2])

    # Courbe aire en fonction de L1
    plt.figure(figsize=(8, 5))
    for L2 in sorted(df["L2"].unique()):
        subset = df[df["L2"] == L2]
        moyennes = subset.groupby("L1")["aire"].mean()
        plt.plot(moyennes.index, moyennes.values, label=f"L2 = {L2}")
    plt.title("Aire vs L1 pour différents L2")
    plt.xlabel("L1 (mm)")
    plt.ylabel("Aire (mm²)")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Courbe aire en fonction de L2
    plt.figure(figsize=(8, 5))
    for L1 in sorted(df["L1"].unique()):
        subset = df[df["L1"] == L1]
        moyennes = subset.groupby("L2")["aire"].mean()
        plt.plot(moyennes.index, moyennes.values, label=f"L1 = {L1}")
    plt.title("Aire vs L2 pour différents L1")
    plt.xlabel("L2 (mm)")
    plt.ylabel("Aire (mm²)")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Courbe aire en fonction de la taille de l'effecteur
    plt.figure(figsize=(8, 5))
    moyennes = df.groupby("cote_E")["aire"].mean()
    plt.plot(moyennes.index, moyennes.values, marker='o', color='purple')
    plt.title("Aire vs Côté effecteur")
    plt.xlabel("Côté effecteur (mm)")
    plt.ylabel("Aire (mm²)")
    plt.grid(True)
    plt.show()

def tracer_heatmap(resultats, cote_E_fixe):
    df = pd.DataFrame(resultats, columns=["params", "aire"])
    df["L1"] = df["params"].apply(lambda x: x[0])
    df["L2"] = df["params"].apply(lambda x: x[1])
    df["cote_E"] = df["params"].apply(lambda x: x[2])

    df_fix = df[df["cote_E"] == cote_E_fixe]

    heatmap_data = df_fix.pivot_table(index="L2", columns="L1", values="aire")

    plt.figure(figsize=(8, 6))
    sns.heatmap(heatmap_data, annot=True, fmt=".0f", cmap="Blues")
    plt.title(f"Aire de travail (mm²) pour côté effecteur = {cote_E_fixe} mm")
    plt.xlabel("L1 (mm)")
    plt.ylabel("L2 (mm)")
    plt.show()

for L1 in tqdm(range(80, 151, 10)):
    for L2 in range(80, 151, 10):
        for cote_E in range(80, 200, 5):

            Re = cote_E / np.sqrt(3)  # Rayon du triangle effecteur équilatéral
            Rb = 300                   # Rayon du triangle base équilatéral
            resolution = 5
            limite = 200

            # Scan de la grille
            points = []
            for x in np.arange(-limite, limite + 1, resolution):
                for y in np.arange(-limite, limite + 1, resolution):
                    if est_position_valide(x, y, 0, L1, L2, Re, Rb):
                        points.append((x, y))

            #print(f"✅ Nombre de points atteignables : {len(points)}")

resultats = trouver_config_maximale()

tracer_courbes(resultats)
tracer_heatmap(resultats, cote_E_fixe=6)
