function [TETAref, Vref, Wref] = calcTetaVW(Vx, aX, Vy, aY)

TETAref = atan2(Vy,Vx);

Vref = sqrt(Vx^2+Vy^2);

Wref = (Vx*aY-Vy*aX)/(Vx^2+Vy^2);