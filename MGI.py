function q=MGI_analytique(eff)

global L1 L2 Rb Re

%Mat. rotation et TH de l'effecteur
RotEff=[cos(eff(3)) -sin(eff(3)); sin(eff(3)) cos(eff(3))];
Transl=[eff(1); eff(2)];
THEff=[RotEff Transl; 0 0 1];

%angle R_i par rapport R_0
ang1=[0 2*pi/3 4*pi/3];

%angle des positions O_i et E_i 
ang2=[-pi/2 pi/6 5*pi/6];

q=[];

for i=1:3
    %Mat Rot et TH de R_i par rapport à R_0
    Rot=[cos(ang1(i)) -sin(ang1(i)); sin(ang1(i)) cos(ang1(i))];
    TH=[Rot [Rb*cos(ang2(i)); Rb*sin(ang2(i))]; [0 0 1]];
    
    %Position des points E_i dans R_E
    PEi_E=[Re*cos(ang2(i)); Re*sin(ang2(i)); 1];
    
    %Position des trois points E_i de l'effecteur dans R_0
    PEi_0=THEff*PEi_E;

    % Position des points effecteur E_i dans les repères dans robots R_i
    PEi_i=inv(TH)*PEi_0;

    %MGI 2R plan
    x=PEi_i(1);
    y=PEi_i(2);

    aux=(x^2+y^2-L1^2-L2^2)/(2*L1*L2);
    if abs(aux) < 1 
        beta=acos(aux); % changer le signe pour la solution coude en haut
    else
        beta=0; disp('problème d atteignabilité');
    end
    alpha=atan2(y,x)-atan2(L2*sin(beta),L1+L2*cos(beta));
    q=[q;[alpha; beta]];
    
end



end
