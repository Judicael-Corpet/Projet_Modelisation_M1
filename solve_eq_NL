function F=solve_eq_NL(q,eff)

%q : regroupe les 6 alpha beta dans bras 1,2,3
%eff : vecteur pose de l'effecteur x_E,y_E,theta_E

global L1 L2 Rb Re

alpha(1)=q(1);
beta(1)=q(2);
alpha(2)=q(3);
beta(2)=q(4);
alpha(3)=q(5);
beta(3)=q(6);

%angle R_i par rapport R_0
ang1=[0 2*pi/3 4*pi/3];

%angle des positions O_i et E_i 
ang2=[-pi/2 pi/6 5*pi/6];

%Mat. rotation et TH de l'effecteur
RotEff=[cos(eff(3)) -sin(eff(3)); sin(eff(3)) cos(eff(3))];
Transl=[eff(1); eff(2)];
THEff=[RotEff Transl; 0 0 1];

for i=1:3
    %Position des points E_i dans R_E
    PEi_E=[Re*cos(ang2(i)); Re*sin(ang2(i)); 1];
    
    %Position des trois points E_i de l'effecteur dans R_0
    PEi_0=THEff*PEi_E;

    %Mat Rot et TH de R_i par rapport à R_0
    Rot=[cos(ang1(i)) -sin(ang1(i)); sin(ang1(i)) cos(ang1(i))];
    THRi_0=[Rot [Rb*cos(ang2(i)); Rb*sin(ang2(i))]; [0 0 1]];
    
    % Points  B_i extrémités des bras en fonction alphai et betai dans R_0
    PBi=THRi_0*[L1*cos(alpha(i))+L2*cos(alpha(i)+beta(i));L1*sin(alpha(i))+L2*sin(alpha(i)+beta(i)); 1];

    % Les contraintes (fonction égale à zéro) exprime que B_i doit être
    % confondu avec E_i, coordonnées nulles sur x et y
    F(2*i-1)=PBi(1)-PEi_0(1);
    F(2*i)=PBi(2)-PEi_0(2);
end

end


