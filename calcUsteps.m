function dU = calcUsteps(U,Nu,delta)

    dU = zeros(2,(Nu*4));
    
    for i=0:1:Nu-1
        dU(1,1+4*i)=U(1,i+1)+delta;
        dU(2,1+4*i)=U(2,i+1);
        dU(1,2+4*i)=U(1,i+1)-delta;
        dU(2,2+4*i)=U(2,i+1);
        dU(1,3+4*i)=U(1,i+1);
        dU(2,3+4*i)=U(2,i+1)+delta;
        dU(1,4+4*i)=U(1,i+1);
        dU(2,4+4*i)=U(2,i+1)-delta;
    end
end