function [Vout_MPC, Wout_MPC] = MPC(Xrefp, Yrefp, TRsx, TRsy, TRst, TRsv, TRsw,Xref, Yref, PHIref, Vref, Wref)
    % Par�metros do controlador
    Vmax = 0.2;
    d = 0.23;
    N1 = 1;
    N2 = 10; %Horizonte de predi��o
    Nu = 2;  %Horizonte de controle
    L1 = 100;
    L2 = 2000;
    L3 = 0.05;
    eta = 0.1;
    Imax = 15;
    I = 0;
    Delta = 0.1;
    
    % Par�metros do Otimizador
    alpha = 0.015;
    
    % Vetores do Otimizador
    
    Jgrad      = zeros([4 1]);
    Jgrad_prev = zeros([4 1]);
    Jsteps     = zeros([8 1]);
    % Defini��o das vari�veis de controle
    
    Ubest = zeros([2 2]);
    Uref  = zeros([2 2]);
    Uaux  = zeros([2 2]);
    
    % Cria��o de um struct para guardar o valor da itera��o anterior do
    % controlador
    SimRob = struct('x',0,'y',0,'teta',0,'v',0,'w',0);
    
    % Inicializa��o das vari�veis de controle
    
    %Saida de controle desejada
    Uref = [Vref Vref; Wref Wref];
    
    % C�lculo da trajetoria de referencia
    lTi = 0;
    
    [tPX,tPY,tPTeta] = calcRefTraj(lTi,TRsx,TRsy,TRst,Xref,Yref,PHIref,Vref,Wref,N2,Xrefp,Yrefp);
    
    % Atualiza o modelo do preditor
    SimRob.x = TRsx;
    SimRob.y = TRsy;
    SimRob.teta = TRst;
    SimRob.v = TRsv;
    SimRob.w = TRsw;
    
    % Loop de controle
    
    % Reliza a satura�ao da velocidade das rodas com base na velocidade
    % m�xima
    Uref = scaleForSaturation(Uref,d,Nu,Vmax);
    
    Jatual = COST_FUNCTION(SimRob.x,SimRob.y,SimRob.teta,SimRob.v,SimRob.w,Uref,tPX,tPY,tPTeta,N1,N2,Nu,L1,L2,L3);
    Jbest = Jatual;    

   %Loop de otimiza��o do Ubest que minimiza o Jbest
   while (I < Imax) && (Jatual > eta)
       Usteps = calcUsteps(Uref,Nu, Delta);
        for k = 0:1:Nu-1
            for j=1:1:4
                %Atribui velocidade para cada passo U
                for m = 0:1:Nu-1
                    if m == k
                        Uaux(1,m+1)= Usteps(1,(j+4*k));
                        Uaux(2,m+1)= Usteps(2,(j+4*k));
                    else
                        Uaux(1,m+1)=Uref(1,1);
                        Uaux(2,m+1)=Uref(2,1);
                    end 
                    
                end
                SimRob.x=TRsx;
                SimRob.y=TRsy;
                SimRob.teta=TRst;
                SimRob.v=TRsv;
                SimRob.w=TRsw;

                Uaux=scaleForSaturation(Uaux,d,Nu,Vmax);

                J = COST_FUNCTION(SimRob.x,SimRob.y,SimRob.teta,SimRob.v,SimRob.w,Uaux,tPX,tPY,tPTeta,N1,N2,Nu,L1,L2,L3);
                Jsteps(j+ 4*k,1) = J;
            
            end
        end
        
        % calculo do gradiente de J baseado no Jsteps
        for h=0:1:Nu-1
            Jgrad_prev(2*h+1,1) = Jgrad(2*h+1,1);
            Jgrad(2*h+1,1) = Jsteps(4*h+1,1)- Jsteps(4*h+2,1);
            Jgrad_prev(2*h+2,1) = Jgrad(2*h+2,1);
            Jgrad(2*h+2,1)=Jsteps(4*h+3,1)-Jsteps(4*h+4,1);
        end
    
        %Gradiente conjugado Algoritmo de Polak-Bibieri

        di=[0 0];
        x1=di;

        for z=0:1:Nu-1
            di(1)=Jgrad(2*z+1,1);
            di(2)=Jgrad(2*z+2,1);
            
            x1(1)=(Uref(1,z+1) - alpha*di(1));
            x1(2)=(Uref(2,z+1) - alpha*di(2));
            
            Jgrad_prev(2*z+1,1)=Jgrad(2*z+1,1);
            Jgrad(2*z+1,1)=Jsteps(4*z+1,1)-Jsteps(4*z+2,1);
            
            Jgrad_prev(2*z+2,1)=Jgrad(2*z+2,1);
            Jgrad(2*z+2,1)=Jsteps(4*z+3,1)-Jsteps(4*z+4,1);
            
            beta=0;
            
            if ((Jgrad(2*z+1,1)>=eta)||(Jgrad(2*z+2,1)>=eta))
                
                t1=(Jgrad(2*z+1,1)-Jgrad_prev(2*z+1,1));
                t2=(Jgrad(2*z+2,1)-Jgrad_prev(2*z+2,1));
                
                a1=(Jgrad(2*z+1,1)*t1);
                a2=(Jgrad(2*z+2,1)*t2);
               
                b1=(Jgrad_prev(2*z+1,1)*Jgrad_prev(2*z+1,1));
                b2=(Jgrad_prev(2*z+2,1)*Jgrad_prev(2*z+2,1));
                
                beta=((a1+a2)/(b1+b2));
            
            end
            
            Uref(1,z+1)=x1(1)+alpha*(-Jgrad(2*z+1,1)+beta*Jgrad_prev(2*z+1,1));

            Uref(2,z+1)=x1(2)+alpha*(-Jgrad(2*z+2,1)+beta*Jgrad_prev(2*z+2,1));
        end
        
        
        SimRob.x=TRsx;
        SimRob.y=TRsy;
        SimRob.teta=TRst;
        SimRob.v=TRsv;
        SimRob.w=TRsw;
        
        % ix SATURA A VELOCIDADE DAS RODAS QUE SER�O USADAS NO PR�XIMO LOOP
        % DE CONTROLE (while)
        
        Uref = scaleForSaturation(Uref,d,Nu,Vmax);
        
        Jatual = COST_FUNCTION(SimRob.x,SimRob.y,SimRob.teta,SimRob.v,SimRob.w,Uref,tPX,tPY,tPTeta,N1,N2,Nu,L1,L2,L3);
        
        if Jatual < Jbest
            Jbest = Jatual;
            Ubest = Uref;
        end
        I=I+1;
    end %END_WHILE      
    Vout_MPC = Ubest(1,1);
    Wout_MPC = Ubest(2,1);  