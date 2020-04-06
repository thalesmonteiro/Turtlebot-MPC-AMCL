% Realiza o calculo de trajetoria
[obstacles, xf, yf, current] = gertraj();

%Finaliza conexão existente com o ROS
rosshutdown
clear velPub
clear velSub
clear odomSub
clear msg

%Inicia a conexão com o ROS
ipaddress = '192.168.0.144';
rosinit(ipaddress);

%Posiciona o mapa no gazebo
gazebo = ExampleHelperGazeboCommunicator;
map = ExampleHelperGazeboModel('jersey_barrier','gazeboDB');
spawnModel(gazebo,map,[3.835709 6.290333 0],[0, 0, 0]);

%Realiza as inscrições nos topicos do Turtlebot
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
odomSub = rossubscriber('/odom', 'BufferSize', 25);
velSub = rossubscriber('/mobile_base/commands/velocity', 'BufferSize', 1);
laserSub = rossubscriber('/scan');
msg = rosmessage('geometry_msgs/Twist');


% Realiza medidas necessarias para o MPC
xref = repelem(0,length(current));
yref = repelem(0,length(current));

for v = 2:length(current)
   xref(v-1) = current{v}(1);
   yref(v-1) = current{v}(2);
end 

Vx = diff(xref);
aX = diff(Vx);

Vy = diff(yref);
aY = diff(Vy);

% Inicia variaveis do MPC
Xrefp = 0;
Yrefp = 0;
TRst = 0;
TRsv = 0;
TRsw = 0;
Xref = 0;
Yref = 0;
PHIref = 0;
Vref = 0;
Wref = 0;
TRsx = 0;
TRsy = 0;



%Inicializa o modelo de odometria para o AMCL
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.1 0.1 0.1 0.1];

%Configura o laser
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = grid;

% Transformação ROS
tftree = rostf;
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link','/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W,sensorTransform.Transform.Rotation.X, sensorTransform.Transform.Rotation.Y,sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Configura rangeFinder
rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X , sensorTransform.Transform.Translation.Y ,laserRotation(1)];


% Inicia objeto do AMCL
amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.1,0.1,0.1];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 10000];
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3)*0.5;

visualizationHelper = ExampleHelperAMCLVisualization(grid);

% Loop de Controle
for i = 1:length(current)-3
    
   Xrefp = xref(i);
   Yrefp = yref(i);
   
   %Controlador MPC
   [Vout_MPC, Wout_MPC] = MPC(Xrefp, Yrefp, TRsx, TRsy, TRst, TRsv, TRsw,Xref, Yref, PHIref, Vref, Wref);
   
   %Envia informações para o Robo
   msg.Linear.X = Vout_MPC;
   msg.Angular.Z = Wout_MPC;
   send(velPub, msg);
   
   %Pausa necessária para que o robo execute a ação enviada
   pause(2.3);
   
   %Realiza leitura da odometria
   msgOdom = receive(odomSub);
   
   %Posições atuais da odometria
   TRsx = msgOdom.Pose.Pose.Position.X;
   TRsy = msgOdom.Pose.Pose.Position.Y;
   TRst = calcTeta(msgOdom.Pose.Pose.Orientation.Z,msgOdom.Pose.Pose.Orientation.W);
   
   measurement = [TRsx; TRsy; TRst];
  
   %Leitura do Scan
   scanMsg = receive(laserSub);
   scan = lidarScan(scanMsg);
   
   [isUpdated, estimatedPose, estimatedCovarience] = amcl(measurement,scan);
   
   TRsv = velSub.LatestMessage.Linear.X;
   TRsw = velSub.LatestMessage.Angular.Z;
   
   [PHIref, Vref, Wref] = calcTetaVW(Vx(i), aX(i), Vy(i), aY(i));
   Xref = xref(i);
   Yref = yref(i);
   
   %Se o AMCL atualizou a posição
   if(isUpdated) 
       plotStep(visualizationHelper, amcl, estimatedPose, scan, i);
       disp(estimatedPose);
       TRsx = estimatedPose(1);  
       TRsy = estimatedPose(2);
       TRst = estimatedPose(3); 
       plot(xref(i), yref(i), 'ro');
       plot(TRsx, TRsy, 'x');
   else
       hold on;
       plot(xref(i), yref(i), 'ro');
   end  
end 

