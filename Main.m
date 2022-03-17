clc
clear
close all
scenario = 'backwards'; %"oblique";%"parallel"; %  
% ����ʱ��:�ɱ�
fixTime = 0 ;   % Ĭ��:0 (�ɱ�)  %�䲽��3���������ɻ�ý��������������Ҫ10+��

%%
%����
Ts= 0.05;
if scenario == "backwards"   %����
    sampleN = 3;   %�²���
    if fixTime == 1
        Ts = 0.65/3*sampleN; %0.65/3���˶��ֱ���һ��
    else
        Ts = 0.6/3*sampleN; %0.6/3
    end
elseif scenario == "paralle"                        %�෽
    sampleN = 3;
    if fixTime == 1
        Ts = 0.95/3*sampleN;
    else
        Ts = 0.9/3*sampleN;
    end
else
    sampleN = 3;   %�²���
    if fixTime == 1
        Ts = 0.65/3*sampleN; %0.65/3���˶��ֱ���һ��
    else
        Ts = 0.6/3*sampleN; %0.6/3
    end
end
%% ��
L = 2.7; % ���
motionStep = 0.1; % Hybrid A ����

% ��������Ϊ�ο���ԭ�㶨��ߴ篸
% �����ߴ篸: (x_upper+x_lower)+(y_upper+y_lower)
% [x_upper;y_upper;-x_lower;-y_lower]������Ϊ��ֵ
ego = [3.7; 1; 1; 1];

%% �ϰ���
% ˳ʱ�밴���㶨���ϰ���
% for plot
nObPlot = 3 ;  %�ϰ������
vObPlot = [4,4,4]; %ÿ���ϰ�����еĶ����������ζ�Ϊ4
% for opt problem
nOb = 3;              % �ϰ������
vOb = [3,3,2];        % ���������������޽Ӵ����ܵĶ���
vObMPC = vOb-[1,1,1]; % ÿ���������γ�һ��������ÿ���1

% ��ʼ 
if scenario == "backwards"
    disp('start reverse parking');
elseif scenario == "parallel"
    disp('star parallel parking');
elseif scenario == "oblique"
    disp('star oblique parking');
else
    disp('error: wrong parking scenario');
end
%%  90�㵹���� 
if scenario == "backwards"
    % �ϰ����ͼ�ö��㣬�γɱջ�
    % ����2������4.7�� ͣ��λ�� 2.6����5
    % ��࣬�Ҳ࣬�Բ࣬��·�� 10
    lObPlot = {  { [-20;5], [-2;5], [-2;-5], [-20;-5], [-20;5] }  ,...  
	 	         { [2;5], [20;5], [20;-5], [2;-5], [2;5] } ,...       
		         { [-20;15], [20;15], [20;11], [-20,11], [-20;15] }  };     
           
    % �Ż��ö���
    lOb = {  { [-20;5], [-2;5], [-2;-5] }  , ...
	 	     { [2;-5] , [2;5] , [20;5] } , ...
		     { [20;11], [-20;11] }	      };
    % ��ʼλ��
    x0 = [-6,8.5-2,0.0,0.0];
    % ͣ��״̬ [x,y ,����� ,����]
    xF = [0,1.3,pi/2,0];
    % �����ϰ��� ����hybrid a*���ٽ���
    ox = [];
    oy = [];
    % �ϰ��� 1
    for i = -12:0.1:-2
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    for i = -2:0.1:5
        ox(end+1) = -2;
        oy(end+1) = i;
    end
    % �ϰ��� 2
    for i = -2:0.1:5
        ox(end+1) = 2;
        oy(end+1) = i;
    end
    for i = 2:0.1:12
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    % �ϰ��� 3
    for i = -12:0.1:12
        ox(end+1) = i;
        oy(end+1) = 11.0;
    end
%% �෽ͣ��
elseif scenario == "parallel"
    %��࣬�Ҳ࣬�ײࣨ�����ǶԲࣩ����ײ�2.5 ��������ʽͬ��������
    lObPlot = {   { [-15;5], [-3.5;5], [-3.5;0], [-15;0], [-15;5] }  ,... 
		 	      { [3;5], [15;5], [15;0], [3;0], [3;5] } ,... 
			      { [-3.5;0], [-3.5;2.5], [3;2.5], [3,0], [-3.5;0] },...
                  { [-15;15], [15;15], [15;11], [-15,11], [-15;15] }   };
    lOb = {   { [-20;5], [-3.5;5], [-3.5;0] }  , ...
	 	      { [3.;0] , [3.;5] , [20;5] } , ...
		      { [-3.5;2.5], [ 3;2.5] },...
              { [15;11], [-15;11] }    };
    % ��ʼλ��
    x0 = [-6,8.5-2,0.0,0.0];
    % ͣ��״̬
    xF = [-L/2+0.1,4,0,0];
    nObPlot = 4 ;  %�ϰ������
    vObPlot = [4,4,4,4]; %ÿ���ϰ�����еĶ����������ζ�Ϊ4
    
    % �����ϰ��� for hybrid a* �Կ��ǶԲ�
    ox = [];
    oy = [];
    % �ϰ��� 1
    for i = -12:0.1:-3.5
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    for i = 2.5:0.1:5
        ox(end+1) = -3.5;
        oy(end+1) = i;
    end
    % �ϰ��� 2
    for i = -3.5:0.1:3
        ox(end+1) = i;
        oy(end+1) = 2.5;
    end
    % �ϰ��� 3
    for i = 2.5:0.1:5
        ox(end+1) = 3.0;
        oy(end+1) = i;
    end
    for i = 3:0.1:12
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    % �ϰ��� 4
    for i = -12:0.1:12
        ox(end+1) = i;
        oy(end+1) = 11.0;
    end   
%% б��60�㵹��   
    elseif scenario == "oblique"
        
       coe =1.5 *4.7; 
    % �ϰ����ͼ�ö��㣬�γɱջ�
    % ����2������4.7�� ͣ��λ�� 2.6����5
    % ��࣬�Ҳ࣬�Բ࣬��·�� 10
    Angle =60;
    lObPlot = {  { [-12;coe*sin(Angle/180*pi)], [-2-coe *cos(Angle/180*pi);coe*sin(Angle/180*pi)], [-2;0], [-12;0], [-12;coe*sin(Angle/180*pi)] }  ,...  
	 	         { [2;0],  [2-coe*cos(Angle/180*pi);coe*sin(Angle/180*pi)], [12-coe*cos(Angle/180*pi);coe*sin(Angle/180*pi)], [12-coe*cos(Angle/180*pi);0], [2;0] } ,...       
		         { [-12;15], [12-coe*cos(Angle/180*pi);15], [12-coe*cos(Angle/180*pi);11], [-12,11], [-12;15] }  };     
           
    % �Ż��ö���
    lOb = {  { [-12;coe*sin(Angle/180*pi)], [-2-coe *cos(Angle/180*pi);coe*sin(Angle/180*pi)], [-2;0] }  , ...
	 	     { [2;0],  [2-coe*cos(Angle/180*pi);coe*sin(Angle/180*pi)], [12-coe*cos(Angle/180*pi);coe*sin(Angle/180*pi)]} , ...
		     { [12-1.2*L*cos(Angle/180*pi);11], [-12;11] }	      };
         
         figure(2);
        title('Trajectory Comparison');
        hold on
        %plot obs
        for j = 1:nObPlot
            for k = 1:vObPlot(j)
                plot([lObPlot{j}{k}(1),lObPlot{j}{k+1}(1)] , [lObPlot{j}{k}(2),lObPlot{j}{k+1}(2)] ,"k");
            end
        end
    % ��ʼλ��
    x0 = [-8*0,8.5,pi,0.0]; %
    % ͣ��״̬ [x,y ,����� ,����]
    xF = [0-1.0*cos(Angle/180*pi),1.5*sin(Angle/180*pi),4*pi/6,0];%[0-2.5*cos(Angle/180*pi),3.5*sin(Angle/180*pi),4*pi/6,0];
    % �����ϰ��� ����hybrid a*���ٽ���
    ox = [];
    oy = [];
    % �ϰ��� 1
    for i = -12:0.1:-2-coe*cos(Angle/180*pi)
        ox(end+1) = i;
        oy(end+1) = coe*sin(Angle/180*pi);
    end
    for i = coe*sin(Angle/180*pi):-0.1:0
        ox(end+1) = -2-coe *cos(Angle/180*pi)+ (coe*sin(Angle/180*pi)-i)/tan(Angle/180*pi) ;
        oy(end+1) = i;
    end
    % �ϰ��� 2
    for i = coe*sin(Angle/180*pi):-0.1:0
        ox(end+1) = 2-coe*cos(Angle/180*pi) +(coe*sin(Angle/180*pi)-i)/tan(Angle/180*pi)  ;
        oy(end+1) = i;
    end
    for i = 2-coe*cos(Angle/180*pi):0.1:12-coe*cos(Angle/180*pi)
        ox(end+1) = i;
        oy(end+1) = coe*sin(Angle/180*pi);
    end
    % �ϰ��� 3
    for i = -12:0.1:12-coe*cos(Angle/180*pi)
        ox(end+1) = i;
        oy(end+1) = 11.0;
    end
    
end
figure(1)
 plot(ox, oy, ".k")
    hold on
    xlabel('X /m')
    ylabel('Y /m')

%% 
% �߽�
% [x_lower, x_upper, -y_lower,   y_upper  ]
XYbounds = [-15,15,1,10];


 
% x0 = [9,7,0,0];

plot(x0(1),x0(2),'p')
plot(xF(1),xF(2),'p')

%���� hybrid a* ʱ��
tic;                % ��ʱ��ʼ  ��Ӧtoc
% ���ɲο�·�������루��ʼ״̬������״̬���ϰ����դ�ֱ��ʣ�ת�Ƿֱ��ʣ��ϰ����ͼ�ֱ��ʣ�
   
XY_GRID_RESOLUTION = 0.3;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
YAW_GRID_RESOLUTION = deg2rad(5);
OB_MAP_RESOLUTION = 0.1;
% VEHICLE_RADIUS = 1;
% GRID_RESOLUTION = 0.1;
% sx = -6;
% sy =9.5;
% plot(sx,sy,'*')
% gx = -1.35;
% gy = 4;
% plot(gx,gy,'*')
%    [rx, ry] = calc_astar_path(sx, sy, gx, gy, ox, oy, GRID_RESOLUTION, VEHICLE_RADIUS);
   
[rx, ry, ryaw] =...
     hybrid_a_star(x0(1), x0(2), x0(3), xF(1), xF(2), xF(3),...
     ox, oy,XY_GRID_RESOLUTION,...
     YAW_GRID_RESOLUTION,OB_MAP_RESOLUTION);
timeHybAstar = toc; % ��ʱ����
%%

plot(rx,ry,'*')

rv = zeros(length(rx),1);
for i = 1:length(rx)
    if i < length(rx)
        rv(i) = (rx(i+1) - rx(i))/(Ts/sampleN)*cos(ryaw(i))+ ...
                (ry(i+1)-ry(i))/(Ts/sampleN)*sin(ryaw(i));
    else
        rv(i) = 0;
    end
end
% ƽ���ٶȣ����루�ο��ٶȣ������ٶ� 0.3 m/s^2 max acc������ʱ�䣩
[v, a] = veloSmooth(rv,0.3,Ts/sampleN);

% ������ݽ�
for i =1:length(ryaw)-1
    delta(i,1) = atan((ryaw(i+1)-ryaw(i))*L/motionStep*sign(v(i)));   %sign(v(1:end-1)));
%     delta(i) = atan((ryaw(i+1)-ryaw(i))/(Ts/sampleN)*L/v(i));     
    if isnan(delta(i))
        delta(i,1) = 0;
    end
end
        figure(23)
        subplot(5,1,1)
        plot(Ts/sampleN*linspace(0,length(v),length(v)),v)
        hold on
        plot(Ts/sampleN*linspace(0,length(rv),length(rv)),rv)
        legend('smoothed','not smoothed')
        xlabel('time/s');
        ylabel('velocity/ m/s');
        subplot(5,1,2)
        plot(Ts/sampleN*linspace(0,length(a),length(a)),a)
        xlabel('time/s');
        ylabel('acc/ m/s2');
        subplot(5,1,3)
        plot(Ts/sampleN*linspace(0,length(delta),length(delta)),delta*180/pi)
        xlabel('time/s');
        ylabel('delta/ deg');
        subplot(5,1,4)
        plot(Ts/sampleN*linspace(0,length(ryaw),length(ryaw)),ryaw*180/pi)
        xlabel('time/s');
        ylabel('ryaw/ deg');
        subplot(5,1,5)
        plot(Ts/sampleN*linspace(0,length(diff(ryaw)/(Ts/sampleN)),length(diff(ryaw)/(Ts/sampleN))),diff(ryaw)/(Ts/sampleN)*180/pi)
        xlabel('time/s');
        ylabel('yaw rate/ deg');
     u =[delta,a]';
     x(:,1) =x0';
     timeScale=ones(1,length(delta));
     for i = 1:length(delta)
        if fixTime == 1
            x(1,i+1) = (x(1,i) + Ts/sampleN*(x(4,i) + Ts/sampleN/2*u(2,i))*cos((x(3,i) + Ts/sampleN/2*x(4,i)*tan(u(1,i))/L)));
		    x(2,i+1) = (x(2,i) + Ts/sampleN*(x(4,i) + Ts/sampleN/2*u(2,i))*sin((x(3,i) + Ts/sampleN/2*x(4,i)*tan(u(1,i))/L)));
		    x(3,i+1) = (x(3,i) + Ts/sampleN*(x(4,i) + Ts/sampleN/2*u(2,i))*tan(u(1,i))/L);
		    x(4,i+1) = (x(4,i) + Ts/sampleN*u(2,i));
	    else
		    x(1,i+1) = (x(1,i) + timeScale(i)*Ts/sampleN*(x(4,i) + timeScale(i)*Ts/sampleN/2*u(2,i))*cos((x(3,i) + timeScale(i)*Ts/sampleN/2*x(4,i)*tan(u(1,i))/L)));
		    x(2,i+1) = (x(2,i) + timeScale(i)*Ts/sampleN*(x(4,i) + timeScale(i)*Ts/sampleN/2*u(2,i))*sin((x(3,i) + timeScale(i)*Ts/sampleN/2*x(4,i)*tan(u(1,i))/L)));
		    x(3,i+1) = (x(3,i) + timeScale(i)*Ts/sampleN*(x(4,i) + timeScale(i)*Ts/sampleN/2*u(2,i))*tan(u(1,i))/L);
		    x(4,i+1) = (x(4,i) + timeScale(i)*Ts/sampleN*u(2,i));
        end
    end
figure(24)
subplot(2,1,1)
plot(x(1,:),x(2,:),'k.')
xlabel('X/ m')
ylabel('Y/ m')
hold on 
plot(rx,ry,'r')
legend('Vehicle kinemic validated','Hybrid A*')

subplot(2,1,2)
plot(Ts/sampleN*linspace(0,length(x(4,:)),length(x(4,:))),x(4,:),'k.')
hold on 
plot(Ts/sampleN*linspace(0,length(rv),length(rv)),rv,'r')
legend('Smoothed','Hybrid A*')
xlabel('Time/ s')
ylabel('Vehicle Speed/ m/s')
% Ϊ�������²���
rx_sampled = rx(1:sampleN:end);    
ry_sampled = ry(1:sampleN:end);
ryaw_sampled = ryaw(1:sampleN:end);
v_sampled = v(1:sampleN:end);
a_sampled = a(1:sampleN:end);
delta_sampled = delta(1:sampleN:end);

% ��ʼ����������
xWS = [rx_sampled,ry_sampled,ryaw_sampled,v_sampled];  %״̬����
uWS = [delta_sampled,a_sampled];                      
%״̬�ռ�ά��
nx = size(xWS,2);
nu = size(uWS,2);

% xWS = [x0;zeros(size(xWS,1)-2,nx);xF];  %״̬����
% uWS = zeros(size(xWS,1),nu); 



%% OBCA ���
N = size(xWS,1)-1;
[AOB, bOb] = obstHrep(nOb, vOb, lOb); 	% ת��Ϊ�ϰ����ռ����� n0bΪ�ϰ��������v0bΪ���������������޽Ӵ����ܵĶ��㣻10bΪ�Ż��õ��ϰ��ﶥ��
tic;  
opt_dual = DualMultWS_(N,nOb,vObMPC,AOB,bOb,xWS');
opt_dual.tol     = 1e-1;
opt_dual.maxiter = 200;
opt_dual.numvars = sum(vObMPC)*(N+1) + nOb*nx*(N+1);
opt_dual.varsguess =  zeros(opt_dual.numvars,1)+0.1;
[lWS,nWS,l_mat,n_mat,existflag] = opt_dual.optimizer();

% Feasible = ParkingConstraints(x0,xF,N,Ts,L,ego,XYbounds,nOb,vObMPC, AOB, bOb,xWS',uWS',l_mat,n_mat,ones(1,N+1),fixTime,1);

% if existflag ~=1
%     opt_dual.varsguess =  [lWS;nWS];
%     [lWS,nWS,l_mat,n_mat,existflag] = opt_dual.optimizer();
% end
xp10 =xWS';
up10 = uWS';

   Feasible =0;
   exitflag10=0;
   iterat =1;
   while ~Feasible && ~exitflag10
       disp(['Iteration No.: ',num2str(iterat)]);
       mpc =ParkingSignedDist_MPC_(x0,xF,N,nx,nu,Ts,L,ego,XYbounds,nOb,vObMPC,...  % L���
       AOB,bOb,fixTime,xp10,up10,lWS,nWS,xWS');
%        mpc =ParkingSignedDist_MPC_(x0,xF,N,nx,nu,Ts,L,ego,XYbounds,nOb,vObMPC,...  % L���
%        AOB,bOb,fixTime,xp10,up10,lWS,nWS);
       [xp10, up10, scaleTime10, exitflag10, lp10, np10] = mpc.optimizer() ;
       lWS = reshape(lp10,size(lp10,1)*size(lp10,2),1);
       nWS = reshape(np10,size(np10,1)*size(np10,2),1);
       Feasible = ParkingConstraints(x0,xF,N,Ts,L,ego,XYbounds,nOb,vObMPC, AOB, bOb,xp10, up10,lp10, np10,scaleTime10,fixTime,1);
       iterat =iterat+1;
       figure(2);
        title('Trajectory Comparison');
        hold on
        %plot obs
        for j = 1:nObPlot
            for k = 1:vObPlot(j)
                plot([lObPlot{j}{k}(1),lObPlot{j}{k+1}(1)] , [lObPlot{j}{k}(2),lObPlot{j}{k+1}(2)] ,"k.--");
            end
        end
        hold on
        plot(rx,ry,"r.",'LineWidth',2);
        plot(xp10(1,:),xp10(2,:),'MarkerEdgeColor',[abs(sin(iterat)),abs(cos(iterat)),exp(-iterat)]);

        hold on
        plot(x0(1),x0(2),"ob");
        hold on
        plot(xF(1),xF(2),"ob");
        hold on
        legend("Hybrid A*(red)","Global Optimal-based ");
        axis("equal");
   end
   plot(xp10(1,:),xp10(2,:),'k.');
time10 = toc; % ��ʱ����
                        
% ���� H-OBCA ���
if Feasible == 1 || exitflag10 ==1 %���
    disp("H-OBCA successfully completed.");
    figure(11);
    hold on
    plot(xp10(1,:),xp10(2,:),"b");
    plotTraj(xp10',up10',length(rx_sampled)-1,ego,L,nObPlot,...
             vObPlot,lObPlot,'Trajectory generated by Global Optimal-based',Ts);
else
    disp("warning:problem could not be solved")
end

% �Ƚ�  Hybrid a*
figure(29);
title('Trajectory Comparison');
hold on
%plot obs

hold on
h1=plot(rx,ry,"r--",'LineWidth',2);
h2=plot(xp10(1,:),xp10(2,:),"-b");


hold on
axis("equal");
for j = 1:nObPlot
    for k = 1:vObPlot(j)
        plot([lObPlot{j}{k}(1),lObPlot{j}{k+1}(1)] , [lObPlot{j}{k}(2),lObPlot{j}{k+1}(2)] ,"k");
    end
end
hold on
plot(x0(1),x0(2),"ob");
hold on
plot(xF(1),xF(2),"ob");
leg=legend([h1,h2],"Hybrid A*(red)","Global Optimal-based ");
set(leg,'Box','off');
xlabel('X/ m')
ylabel('Y/ m')


figure(26)

hold on 
plot(Ts/sampleN*linspace(0,length(ryaw),length(ryaw)),ryaw,'-b')
plot(Ts*linspace(0,length(xp10(3,:)),length(xp10(3,:))),xp10(3,:),'r--')

leg=legend("Hybrid A*","Global Optimal-based");
set(leg,'Box','off');
% title('yaw compared')
ylabel('ƫ����/ rad');xlabel('Time/ s')
grid on
figure(27)
hold on 
plot(Ts/sampleN*linspace(0,length(rv),length(rv)),rv,'-b')
plot(Ts*linspace(0,length(xp10(4,:)),length(xp10(4,:))),xp10(4,:),'r--')

leg =legend("Hybrid A*","Global Optimal-based");
% title('driving velocity compared')
set(leg,'Box','off');xlabel('Time/ s')
ylabel('����/ m/s')
grid on

figure(28)
plot(Ts*linspace(0,length(scaleTime10),length(scaleTime10)),scaleTime10)
% title('scaleTime')
set(leg,'Box','off');
xlabel('Time/ s')
ylabel('�䲽���Ż�ϵ��')
grid on

totTime = timeHybAstar+time10; % hybrid a + HOBCA
disp(['Total run time: ',num2str(totTime),' s']);
disp([' Hybrid A* time: ',num2str(timeHybAstar), ' s']);
disp([' optimization (OBCA) time: ',num2str(time10),' s']);




