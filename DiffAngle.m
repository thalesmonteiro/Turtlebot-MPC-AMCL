function [ang] = DiffAngle(a1,a2)
    ang=a1-a2;
    if ang<0
        ang=-((-ang/(2*pi))-floor((-ang/(2*pi)))*2*pi);
    end
    if ang<-pi
        ang=ang+2*pi;
    else
        ang=((ang/(2*pi))-floor((ang/(2*pi))))*(2*pi);
    end
    if ang>pi
        ang=ang-2*pi;
    end
end