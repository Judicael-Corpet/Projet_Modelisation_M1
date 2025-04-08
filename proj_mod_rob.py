from numpy import *

class Robot():
    def __init__(self, longueur_bras):
        self.longueur_bras = longueur_bras
        self.q = []
    
        
    def traceRobot(self, alpha1, beta1, alpha2, beta2, alpha3, beta3) :
        self.q[0] = alpha1
        self.q[1] = beta1
        self.q[2] = alpha2
        self.q[3] = beta2
        self.q[4] = alpha3
        self.q[5] = beta3

        Rot1=[cos(2*pi/3) -sin(2*pi/3); sin(2*pi/3) cos(2*pi/3)];
        Rot2=[cos(4*pi/3) -sin(4*pi/3); sin(4*pi/3) cos(4*pi/3)];

        P10=[0; -Rb];
        P11=[eye(2) [0; -Rb]; 0 0 1]*[L1*cos(alpha1);L1*sin(alpha1); 1];
        P12=[eye(2) [0; -Rb]; 0 0 1]*[L1*cos(alpha1)+L2*cos(alpha1+beta1);L1*sin(alpha1)+L2*sin(alpha1+beta1); 1];

        P20=[Rb*sqrt(3)/2; Rb/2];
        P21=[Rot1 [Rb*sqrt(3)/2; Rb/2]; 0 0 1]*[L1*cos(alpha2);L1*sin(alpha2); 1];
        P22=[Rot1 [Rb*sqrt(3)/2; Rb/2]; 0 0 1]*[L1*cos(alpha2)+L2*cos(alpha2+beta2);L1*sin(alpha2)+L2*sin(alpha2+beta2); 1];

        P30=[-Rb*sqrt(3)/2; Rb/2];
        P31=[Rot2 [-Rb*sqrt(3)/2; Rb/2]; 0 0 1]*[L1*cos(alpha3);L1*sin(alpha3); 1];
        P32=[Rot2 [-Rb*sqrt(3)/2; Rb/2]; 0 0 1]*[L1*cos(alpha3)+L2*cos(alpha3+beta3);L1*sin(alpha3)+L2*sin(alpha3+beta3); 1];

        figure(); 
        plot([P10(1);P11(1);P12(1)],[P10(2);P11(2);P12(2)]); %bras 1
        hold ON
        plot([P20(1);P21(1);P22(1)],[P20(2);P21(2);P22(2)]); %bras 2
        hold ON
        plot([P30(1);P31(1);P32(1)],[P30(2);P31(2);P32(2)]); %bras 3
        hold ON
        plot([P12(1);P22(1);P32(1);P12(1)],[P12(2);P22(2);P32(2);P12(2)],'LineWidth',2); %effecteur
        axis square; axis equal;
