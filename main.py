from proj_mod_rob import *

def main():
 
    
    
    L1=0.10;  # long segment 1
    L2=0.10;  # long segment 2
    Rb=0.1322594;  # Rayon base
    Re=0.07; # Rayon effecteur

    dimensionPlateau= 0.50 #  carré en cm
    

    robot= Robot(L1,L2,Rb,Re,dimensionPlateau)
    pos_eff=[0.0, 0.0, 0.0]; # pose effecteur à choisir 
    
    # 1ière méthode : résolution de systèmes d'éq non-linéaires 
    from scipy.optimize import fsolve
    # 1ère méthode: Résoudre le système d'équations non linéaires
    q1 = fsolve(lambda q: robot.solve_eq_NL(q, pos_eff), robot.q0)
    robot.calculPos(q1)
    # robot.traceRobot()

    # 2ième méthode : Résolution analytique du MGI 2R plan :renvoie alphi_i et beta_i
    # q=robot.MGI_analytique(pos_eff)
    # print(len(q))
    # robot.calculPos(q)
    # robot.traceRobot(q)
    # plt.show()


    # Pygame 

    robot.runPygame(q)

if __name__=="__main__":
    main()