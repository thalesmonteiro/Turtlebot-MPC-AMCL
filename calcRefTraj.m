%Cálculo da Trajetória de Referência

function [trajPX, trajPY, trajPTeta]=calcRefTraj(lTi,SRx,SRy,SRt,trajX,trajY,trajTeta,...
    V,W,N2,trajXp,trajYp)

N=2001;
i=1;
deltaD = V*0.04;  %0.1 para Pioneer e 0.04 para Turtlebot
deltaW = W*0.04;
dTotal = 0;

trajPX=zeros(1,N2+1);
trajPY=zeros(1,N2+1);
trajPTeta=zeros(1,N2+1);

%angle of current segment to world
teta = atan2(trajYp-trajY,trajXp-trajX);

%angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
al=DiffAngle(trajTeta,SRt)-teta;
d1_3=sqrt(((trajX-SRx)*(trajX-SRx))+((trajY-SRy)*(trajY-SRy)));
%distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
dl= d1_3*cos(al);

segmentSize=sqrt(((trajXp-trajX)*(trajXp-trajX))+((trajYp-trajY)*(trajYp-trajY)));

%sum of distances from segments (of main trajectory)
dSegments = segmentSize - dl;
    
%initial point of predictive controller trajectory
trajPX(i)= trajX + dl*cos(teta);
trajPY(i)= trajY + dl*sin(teta);
trajPTeta(i)=trajTeta;

%build new trajectory
for j=2:1:N2+1

   %reached the end of the trajectory
   if(lTi >= N-1)
       trajPX(j) = trajX;
       trajPY(j) = trajY;
       trajPTeta(j) = trajTeta;
   else
       %add trajectory points along current segment
       trajPX(j) = trajPX(j-1)+deltaD*cos(teta);
       trajPY(j) = trajPY(j-1)+deltaD*sin(teta);
       trajPTeta(j) = trajPTeta(j-1)+deltaW;
         
       dTotal = dTotal + deltaD;
             
       %change segment of the main trajectory that's being tracked
       if (dTotal >= dSegments)
           
           lTi=lTi+1;
           teta = atan2(trajYp-trajY,trajXp-trajX);
           segmentSize = sqrt(((trajXp-trajX)*(trajXp-trajX))+((trajYp-trajY)*(trajYp-trajY))); 

           %add point (already in next segment)
           trajPX(j) = trajX+(dTotal-dSegments)*cos(teta);
           trajPY(j) = trajY+(dTotal-dSegments)*sin(teta);
           trajPTeta(j) = trajTeta;
           
           dSegments = dSegments + segmentSize;
       end
   end
end

%handle teta references
for ab=1:1:N2
    if trajPTeta(ab) > pi 
       trajPTeta(ab) = trajPTeta(ab) - 2*pi;
    end
end

end
