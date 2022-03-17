clc
clear
close all
scenario = 'backwards'; %"oblique";%"parallel"; %  
% 采样时间:可变
fixTime = 0 ;   % 默认:0 (可变)  %变步长3步迭代即可获得结果；定步长则需要10+步

%%
%参数
Ts= 0.05;
if scenario == "backwards"   %倒车
    sampleN = 3;   %下采样
    if fixTime == 1
        Ts = 0.65/3*sampleN; %0.65/3与运动分辨率一致
    else
        Ts = 0.6/3*sampleN; %0.6/3
    end
elseif scenario == "paralle"                        %侧方
    sampleN = 3;
    if fixTime == 1
        Ts = 0.95/3*sampleN;
    else
        Ts = 0.9/3*sampleN;
    end
else
    sampleN = 3;   %下采样
    if fixTime == 1
        Ts = 0.65/3*sampleN; %0.65/3与运动分辨率一致
    else
        Ts = 0.6/3*sampleN; %0.6/3
    end
end
%% 车
L = 2.7; % 轴距
motionStep = 0.1; % Hybrid A 步长

% 后轴中心为参考点原点定义尺寸
% 汽车尺寸: (x_upper+x_lower)+(y_upper+y_lower)
% [x_upper;y_upper;-x_lower;-y_lower]后两项为负值
ego = [3.7; 1; 1; 1];

%% 障碍物
% 顺时针按顶点定义障碍物
% for plot
nObPlot = 3 ;  %障碍物个数
vObPlot = [4,4,4]; %每个障碍物具有的顶点数，矩形都为4
% for opt problem
nOb = 3;              % 障碍物个数
vOb = [3,3,2];        % 顶点数，不考虑无接触可能的顶点
vObMPC = vOb-[1,1,1]; % 每两个顶点形成一个向量，每项减1

% 开始 
if scenario == "backwards"
    disp('start reverse parking');
elseif scenario == "parallel"
    disp('star parallel parking');
elseif scenario == "oblique"
    disp('star oblique parking');
else
    disp('error: wrong parking scenario');
end
%%  90°倒车 
if scenario == "backwards"
    % 障碍物绘图用顶点，形成闭环
    % 车宽2，车长4.7， 停车位宽 2.6，长5
    % 左侧，右侧，对侧，即路宽 10
    lObPlot = {  { [-20;5], [-2;5], [-2;-5], [-20;-5], [-20;5] }  ,...  
	 	         { [2;5], [20;5], [20;-5], [2;-5], [2;5] } ,...       
		         { [-20;15], [20;15], [20;11], [-20,11], [-20;15] }  };     
           
    % 优化用顶点
    lOb = {  { [-20;5], [-2;5], [-2;-5] }  , ...
	 	     { [2;-5] , [2;5] , [20;5] } , ...
		     { [20;11], [-20;11] }	      };
    % 初始位置
    x0 = [-6,8.5-2,0.0,0.0];
    % 停车状态 [x,y ,航向角 ,车速]
    xF = [0,1.3,pi/2,0];
    % 建立障碍物 用于hybrid a*找临近点
    ox = [];
    oy = [];
    % 障碍物 1
    for i = -12:0.1:-2
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    for i = -2:0.1:5
        ox(end+1) = -2;
        oy(end+1) = i;
    end
    % 障碍物 2
    for i = -2:0.1:5
        ox(end+1) = 2;
        oy(end+1) = i;
    end
    for i = 2:0.1:12
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    % 障碍物 3
    for i = -12:0.1:12
        ox(end+1) = i;
        oy(end+1) = 11.0;
    end
%% 侧方停车
elseif scenario == "parallel"
    %左侧，右侧，底侧（不考虑对侧），离底侧2.5 ，数据形式同“倒车”
    lObPlot = {   { [-15;5], [-3.5;5], [-3.5;0], [-15;0], [-15;5] }  ,... 
		 	      { [3;5], [15;5], [15;0], [3;0], [3;5] } ,... 
			      { [-3.5;0], [-3.5;2.5], [3;2.5], [3,0], [-3.5;0] },...
                  { [-15;15], [15;15], [15;11], [-15,11], [-15;15] }   };
    lOb = {   { [-20;5], [-3.5;5], [-3.5;0] }  , ...
	 	      { [3.;0] , [3.;5] , [20;5] } , ...
		      { [-3.5;2.5], [ 3;2.5] },...
              { [15;11], [-15;11] }    };
    % 初始位置
    x0 = [-6,8.5-2,0.0,0.0];
    % 停车状态
    xF = [-L/2+0.1,4,0,0];
    nObPlot = 4 ;  %障碍物个数
    vObPlot = [4,4,4,4]; %每个障碍物具有的顶点数，矩形都为4
    
    % 建立障碍物 for hybrid a* 仍考虑对侧
    ox = [];
    oy = [];
    % 障碍物 1
    for i = -12:0.1:-3.5
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    for i = 2.5:0.1:5
        ox(end+1) = -3.5;
        oy(end+1) = i;
    end
    % 障碍物 2
    for i = -3.5:0.1:3
        ox(end+1) = i;
        oy(end+1) = 2.5;
    end
    % 障碍物 3
    for i = 2.5:0.1:5
        ox(end+1) = 3.0;
        oy(end+1) = i;
    end
    for i = 3:0.1:12
        ox(end+1) = i;
        oy(end+1) = 5.0;
    end
    % 障碍物 4
    for i = -12:0.1:12
        ox(end+1) = i;
        oy(end+1) = 11.0;
    end   
%% 斜置60°倒车   
    elseif scenario == "oblique"
        
       coe =1.5 *4.7; 
    % 障碍物绘图用顶点，形成闭环
    % 车宽2，车长4.7， 停车位宽 2.6，长5
    % 左侧，右侧，对侧，即路宽 10
    Angle =60;
    lObPlot = {  { [-12;coe*sin(Angle/180*pi)], [-2-coe *cos(Angle/180*pi);coe*sin(Angle/180*pi)], [-2;0], [-12;0], [-12;coe*sin(Angle/180*pi)] }  ,...  
	 	         { [2;0],  [2-coe*cos(Angle/180*pi);coe*sin(Angle/180*pi)], [12-coe*cos(Angle/180*pi);coe*sin(Angle/180*pi)], [12-coe*cos(Angle/180*pi);0], [2;0] } ,...       
		         { [-12;15], [12-coe*cos(Angle/180*pi);15], [12-coe*cos(Angle/180*pi);11], [-12,11], [-12;15] }  };     
           
    % 优化用顶点
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
    % 初始位置
    x0 = [-8*0,8.5,pi,0.0]; %
    % 停车状态 [x,y ,航向角 ,车速]
    xF = [0-1.0*cos(Angle/180*pi),1.5*sin(Angle/180*pi),4*pi/6,0];%[0-2.5*cos(Angle/180*pi),3.5*sin(Angle/180*pi),4*pi/6,0];
    % 建立障碍物 用于hybrid a*找临近点
    ox = [];
    oy = [];
    % 障碍物 1
    for i = -12:0.1:-2-coe*cos(Angle/180*pi)
        ox(end+1) = i;
        oy(end+1) = coe*sin(Angle/180*pi);
    end
    for i = coe*sin(Angle/180*pi):-0.1:0
        ox(end+1) = -2-coe *cos(Angle/180*pi)+ (coe*sin(Angle/180*pi)-i)/tan(Angle/180*pi) ;
        oy(end+1) = i;
    end
    % 障碍物 2
    for i = coe*sin(Angle/180*pi):-0.1:0
        ox(end+1) = 2-coe*cos(Angle/180*pi) +(coe*sin(Angle/180*pi)-i)/tan(Angle/180*pi)  ;
        oy(end+1) = i;
    end
    for i = 2-coe*cos(Angle/180*pi):0.1:12-coe*cos(Angle/180*pi)
        ox(end+1) = i;
        oy(end+1) = coe*sin(Angle/180*pi);
    end
    % 障碍物 3
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
% 边界
% [x_lower, x_upper, -y_lower,   y_upper  ]
XYbounds = [-15,15,1,10];


 
% x0 = [9,7,0,0];

plot(x0(1),x0(2),'p')
plot(xF(1),xF(2),'p')

%计算 hybrid a* 时间
tic;                % 计时开始  对应toc
% 生成参考路径，输入（初始状态，最终状态，障碍物，格栅分辨率，转角分辨率，障碍物地图分辨率）
   
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
timeHybAstar = toc; % 计时结束
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
% 平滑速度，输入（参考速度，最大加速度 0.3 m/s^2 max acc，采样时间）
[v, a] = veloSmooth(rv,0.3,Ts/sampleN);

% 计算操纵角
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
% 为热启动下采样
rx_sampled = rx(1:sampleN:end);    
ry_sampled = ry(1:sampleN:end);
ryaw_sampled = ryaw(1:sampleN:end);
v_sampled = v(1:sampleN:end);
a_sampled = a(1:sampleN:end);
delta_sampled = delta(1:sampleN:end);

% 初始化热启动解
xWS = [rx_sampled,ry_sampled,ryaw_sampled,v_sampled];  %状态变量
uWS = [delta_sampled,a_sampled];                      
%状态空间维度
nx = size(xWS,2);
nu = size(uWS,2);

% xWS = [x0;zeros(size(xWS,1)-2,nx);xF];  %状态变量
% uWS = zeros(size(xWS,1),nu); 



%% OBCA 求解
N = size(xWS,1)-1;
[AOB, bOb] = obstHrep(nOb, vOb, lOb); 	% 转换为障碍物半空间向量 n0b为障碍物个数；v0b为顶点数，不考虑无接触可能的顶点；10b为优化用的障碍物顶点
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
       mpc =ParkingSignedDist_MPC_(x0,xF,N,nx,nu,Ts,L,ego,XYbounds,nOb,vObMPC,...  % L轴距
       AOB,bOb,fixTime,xp10,up10,lWS,nWS,xWS');
%        mpc =ParkingSignedDist_MPC_(x0,xF,N,nx,nu,Ts,L,ego,XYbounds,nOb,vObMPC,...  % L轴距
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
time10 = toc; % 计时结束
                        
% 绘制 H-OBCA 结果
if Feasible == 1 || exitflag10 ==1 %标记
    disp("H-OBCA successfully completed.");
    figure(11);
    hold on
    plot(xp10(1,:),xp10(2,:),"b");
    plotTraj(xp10',up10',length(rx_sampled)-1,ego,L,nObPlot,...
             vObPlot,lObPlot,'Trajectory generated by Global Optimal-based',Ts);
else
    disp("warning:problem could not be solved")
end

% 比较  Hybrid a*
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
ylabel('偏航角/ rad');xlabel('Time/ s')
grid on
figure(27)
hold on 
plot(Ts/sampleN*linspace(0,length(rv),length(rv)),rv,'-b')
plot(Ts*linspace(0,length(xp10(4,:)),length(xp10(4,:))),xp10(4,:),'r--')

leg =legend("Hybrid A*","Global Optimal-based");
% title('driving velocity compared')
set(leg,'Box','off');xlabel('Time/ s')
ylabel('车速/ m/s')
grid on

figure(28)
plot(Ts*linspace(0,length(scaleTime10),length(scaleTime10)),scaleTime10)
% title('scaleTime')
set(leg,'Box','off');
xlabel('Time/ s')
ylabel('变步长优化系数')
grid on

totTime = timeHybAstar+time10; % hybrid a + HOBCA
disp(['Total run time: ',num2str(totTime),' s']);
disp([' Hybrid A* time: ',num2str(timeHybAstar), ' s']);
disp([' optimization (OBCA) time: ',num2str(time10),' s']);




