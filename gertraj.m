function [obstacles, xf, yf, current]=gertraj(res)

%Pontos de partida e chegada
%Init=[0.6 0; 0.6 3.5 ; 1.8 4  ; 1.8 0.7 ; 4.5 0.5];
%Goal=[0.6 3.5; 1.8 4 ; 1.8 0.7  ; 4.5 0.5; 4.5 4.5];

%Init=[0.6 0; 0.6 4 ; 1.8 4  ; 1.8 0.5  ; 4.5 0.5];
%Goal=[0.6 4; 2 4   ; 1.8 0.5  ; 4.5 0.5; 4.5 4.5];

Init = [0.6 0  ;  0.8 1.5 ;  0.5 3.7;  1.8 3.7; 1.8 0.5; 4.6 0.5];
Goal = [0.8 1.5;  0.5 3.7 ;  1.8 3.7;  1.8 0.5; 4.6 0.5; 4.6 4.5];
%%Mapa normal para OcupancyGrid
image = imread('mapateste.bmp');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
grid = robotics.BinaryOccupancyGrid(bwimage, 100);
show(grid)


%Infla o OcuppancyGrid map
robotRadius = 0.11;
mapInflated = copy(grid);
inflate(mapInflated, robotRadius);
show(mapInflated)

matriz = occupancyMatrix(mapInflated);
Wo=15;
matrix=flip(matriz*Wo);

xf=0:0.00998043053:5.1; %511
yf=0:0.00998080614:5.2-0.00998080614; %521
[X,Y] = meshgrid(xf,yf);

KA=15;
rG2=KA*sqrt((Goal(6,1)-X).^2+((Goal(6,2)-Y).^2));
Weight=rG2+matrix;
surf(xf,yf,Weight)

%%Identifica os obst�culos da parede como c�rculos
[mx, my]=size(matrix);
cnt=1;
for y=1:7:mx
    for x=1:7:my
        if matrix(y,x)==15
            obstacles{cnt}=[x*0.01 y*0.01 robotRadius];
            plot(obstacles{cnt}(1), obstacles{cnt}(2), 'o')
            hold on
            cnt=cnt+1;
        end
    end
end
cnt=cnt-1;

figure
show(mapInflated)
hold on
plot(Goal(6,1),Goal(6,2),'bo');
hold on
plot(Init(1,1),Init(1,2),'bx');
hold on
for i=1:1:cnt
    plot(obstacles{i}(1),obstacles{i}(2),'ro');
    hold on
end

s = 6;

k=1; %Inicializa o contador de posi��o
for c = 1:s
    %%Gerando a Trajet�ria
    current{1}=Init(c, :); %Diz que posi��o 1 � a inicial.
    
    ka=0.1; %Ganho da fun��o de atra��o
    kr=0.01; %Ganho da fun��o de repuls�o
    DO = 0.2; %limiar do obst�culo
    dist=1; %condi��o inicial do la�o while
    derivxO=0; %inicializa��o do gradiente X do obstaculo
    derivyO=0; %inicializa��o do gradiente Y do obstaculo
    
    while (dist>=0.01)
        for m=1:1:cnt
        RobotObstacleDistance = sqrt(((current{k}(1)-obstacles{m}(1))^2) + ((current{k}(2)-obstacles{m}(2))^2)) - obstacles{m}(3);
            if RobotObstacleDistance <= DO
                derivxO=derivxO+kr*(current{k}(1)-obstacles{m}(1));
                derivyO=derivyO+kr*(current{k}(2)-obstacles{m}(2));
            end
        end
        dist=sqrt((Goal(c,1)-current{k}(1))^2+((Goal(c,2)-current{k}(2))^2));
        derivx=ka*(Goal(c,1)-current{k}(1))/dist;
        derivy=ka*(Goal(c,2)-current{k}(2))/dist;
        current{k+1}(1)=current{k}(1)+(derivx+derivxO); %X
        current{k+1}(2)=current{k}(2)+(derivy+derivyO); %Y
        plot(current{k}(1),current{k}(2),'go');
        hold on
        if dist<=0.1
            break;
        end
        k=k+1;
    end   
end


