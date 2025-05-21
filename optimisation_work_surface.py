import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm  # Pour barre de progression
import pandas as pd
import seaborn as sns


# Paramètres géométriques
L1 = 100  # longueur OA
L2 = 150  # longueur AB
Re = 50   # rayon du plateau effecteur
Rb = 120  # rayon de la base
theta_E = 0  # orientation fixe

# Position des moteurs (O1, O2, O3) en triangle équilatéral
Oi = [(Rb * np.cos(np.radians(a)), Rb * np.sin(np.radians(a))) for a in [270, 30, 150]]

# Coordonnées des points Ei (plateau effecteur tourné de theta_E)
def compute_Ei(x, y, theta):
    angles = [270, 30, 150]
    return [
        (
            x + Re * np.cos(np.radians(a + theta)),
            y + Re * np.sin(np.radians(a + theta))
        )
        for a in angles
    ]

# Vérifie si une solution MGI existe à ce point (x, y)
def is_reachable(x, y):
    Ei_points = compute_Ei(x, y, theta_E)
    for i in range(3):
        Ox, Oy = Oi[i]
        Ex, Ey = Ei_points[i]
        dist = np.hypot(Ex - Ox, Ey - Oy)
        if dist > (L1 + L2) or dist < abs(L1 - L2):
            return False  # Triangle impossible
    return True

# Balayage d'une grille
resolution = 2  # mm par pixel
x_range = np.arange(-200, 200, resolution)
y_range = np.arange(-200, 200, resolution)
reachable_points = []

for x in x_range:
    for y in y_range:
        if is_reachable(x, y):
            reachable_points.append((x, y))

# Aire = nombre de points × surface par point
area_mm2 = len(reachable_points) * (resolution**2)
area_cm2 = area_mm2 / 100.0

print(f"Aire de travail estimée : {area_cm2:.2f} cm²")
"""
# Affichage
rx, ry = zip(*reachable_points)
plt.figure(figsize=(6, 6))
plt.scatter(rx, ry, s=1, color='blue')
plt.title("Aire de travail du robot 3RRR")
plt.xlabel("x (mm)")
plt.ylabel("y (mm)")
plt.axis('equal')
plt.grid(True)
plt.show()
"""

def est_position_valide(x, y, theta, L1, L2, Re, Rb):
    # Fonction de MGI inverse pour tester si la position (x, y, theta) est réalisable
    # Avec Re = rayon effecteur, Rb = rayon base (fixe)
    # Renvoie True si réalisable, False sinon
    for k in range(3):
        angle = 2 * np.pi * k / 3
        Ox = Rb * np.cos(angle)
        Oy = Rb * np.sin(angle)
        Ex = x + Re * np.cos(theta + angle)
        Ey = y + Re * np.sin(theta + angle)
        d = np.hypot(Ex - Ox, Ey - Oy)
        if not (abs(L1 - L2) <= d <= L1 + L2):
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
    Rb = 100  # Rayon de la base fixe
    resolution = 5  # grille de 5 mm
    limite = 200  # espace de recherche

    meilleures_config = None
    max_aire = 0
    resultats = []
    points_max = []

    for L1 in tqdm(range(5, 16)):
        for L2 in range(5, 16):
            for cote_E in range(2, 11):
                aire = calculer_aire_travail(L1, L2, cote_E, Rb, resolution, limite)
                resultats.append(((L1, L2, cote_E), aire))
                if aire > max_aire:
                    max_aire = aire
                    meilleures_config = (L1, L2, cote_E)

    for L1 in tqdm(range(5, 16)):
        for L2 in range(5, 16):
            for cote_E in range(2, 11):
                print(f"🔍 Test L1={L1}, L2={L2}, côté effecteur={cote_E}")
                points_valides = calculer_aire_travail(L1, L2, cote_E)
                aire = points_valides
                if aire > max_aire:
                    max_aire = aire
                    meilleures_config = (L1, L2, cote_E)
                    points_max = points_valides
    
    if meilleures_config is not None:
        print(f"\n✔️ Meilleure configuration : L1={meilleures_config[0]}, L2={meilleures_config[1]}, côté effecteur={meilleures_config[2]} mm")
        print(f"Aire atteignable maximale : {max_aire} points")

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

resultats = trouver_config_maximale()

tracer_courbes(resultats)
tracer_heatmap(resultats, cote_E_fixe=6)
