from projet import *

def main():
 
    # Données qu'on pourra modifier pour tester
    
    L1=0.12;  # long segment 1
    L2=0.12;  # long segment 2
    Rb=0.13;  # Rayon base
    Re=0.07; # Rayon effecteur
    dimensionPlateau= 0.50 #  carré en cm
    pos_eff=[0., 0., 0.0]; # pose effecteur à choisir
    q0= np.array([  # solutions initiales des angles alpha beta des bras 1,2,3
                [0], 
                [pi/2], 
                [0],
                [pi/2 ],
                [0],
                [pi/2]]) 

    robot= Robot(L1,L2,Rb,Re,dimensionPlateau,pos_eff,q0)
    
    robot.run_simulation()

if __name__=="__main__":
    main()