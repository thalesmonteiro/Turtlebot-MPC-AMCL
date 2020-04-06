function [Ua]=scaleForSaturation(U,d,Nu,vmax)

Ua=zeros(2,Nu);

     for i=1:1:Nu
           v=U(1,i);
           w=U(2,i);
           
           %Cinematica Inversa
           v1 = v + ((d*w)/2);
           v2 = v - ((d*w)/2);
           
           %Proportional Saturation
           maxv=max([v1 v2]);
           minv=min([v1 v2]);

           if maxv>vmax 
               scalemax=maxv/vmax;
           else
               scalemax=1;
           end
           if minv<(-vmax) 
               scalemin=minv/(-vmax); 
           else
               scalemin=1;
           end

           scale=max([scalemin scalemax]);

           v1a=v1/scale;
           v2a=v2/scale;
           
           %Cinematica Direta
           vf=(v1a+v2a)/2;
           wf=(v1a-v2a)/d;
  
           Ua(1,i)=vf;
           Ua(2,i)=wf;
           
     end
     
end