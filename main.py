from proj_mod_rob import *

def main():
 
    # Données qu'on pourra modifier pour tester
    
    L1=0.10;  # long segment 1
    L2=0.10;  # long segment 2
    Rb=0.1322594;  # Rayon base
    Re=0.07; # Rayon effecteur
    dimensionPlateau= 0.50 #  carré en cm
    pos_eff=[0.1, -0.02, 0.0]; # pose effecteur à choisir
    q0= np.array([  # solutions initiales des angles alpha beta des bras 1,2,3
                [0], 
                [pi/2], 
                [0],
                [pi/2 ],
                [0],
                [pi/2]]) 

    robot= Robot(L1,L2,Rb,Re,dimensionPlateau,pos_eff,q0)
    
    # 1ière méthode : résolution de systèmes d'éq non-linéaires 
    q=robot.no_linear()
    robot.calculPos(q)
    robot.traceRobot("Non_Linéaire")
    # 2ième méthode : Résolution analytique du MGI 2R plan :renvoie alphi_i et beta_i
    q=robot.MGI_analytique()
    robot.calculPos(q)
    # robot.traceRobot("MGI_Analytique")

    # Pygame 
    # robot.runPygame(q)

if __name__=="__main__":
    main()