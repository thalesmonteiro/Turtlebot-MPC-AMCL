function J = COST_FUNCTION(SimRobx,SimRoby,SimRobteta,SimRobv,SimRobw,Uref,tPX,tPY,tPTeta,N1,N2,Nu,L1,L2,L3)

sumX   = 0;
sumY   = 0;
sumT   = 0;
%sumDU  = 0;
t = 0.01;
%t = 0.01;

 for i = N1:1:N2 % executa 10 vezes
        % corrigindo problemas com angulos
        if i <= Nu
            v = Uref(1,i);
            w = Uref(2,i);
        else
            v = Uref(1,Nu);
            w = Uref(2,Nu);
        end
        
        for j = 1:1:4
            if SimRobteta > pi
                SimRobteta = SimRobteta - 2*pi;
            end
            
                SimRobteta = SimRobteta + t*w;
                SimRobx = SimRobx + t*cos(SimRobteta)*v;
                SimRoby = SimRoby + t*sin(SimRobteta)*v;
                
        end
    sumX = sumX + (tPX(i) - SimRobx)^2;
    sumY = sumY + (tPY(i) - SimRoby)^2;
        
    %erroTeta = DiffAngle(tPTeta,SimRobteta);
    erroTeta =  DiffAngle(tPTeta(i), SimRobteta);
    sumT = sumT + erroTeta^2;
 end
 
    sumDU = (SimRobv - Uref(1,1) )^2 + (SimRobw - Uref(1,2))^2;
% Horizonte de controle, por isso fica fora do FOR
    J = L1 * (sumX + sumY) + L2*sumT + L3*sumDU;

end

