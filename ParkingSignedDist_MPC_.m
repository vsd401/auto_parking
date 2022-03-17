classdef ParkingSignedDist_MPC_ < handle
    
    %% Lagrang dual constrains are descriped in nonlinear equality and inequality form in fmincom
    properties
        x0
        xF
        tol =10e-5;
        maxiter = 200       % max solver iterations
        N
        nx
        nu
        Ts
        L
        ego
        XYbounds
        nob
        vob
        A
        b
        xWS
        uWS
        fixTime
        lwarm
        nwarm
        xWS_rs
        no_weight =11*0;
        %% desired safety distance
        dmin = 0.05			% anything bigger than 0, e.g. 0.05
    end
    
    methods
        
        function obj = ParkingSignedDist_MPC_(x0,xF,N,nx,nu,Ts,L,ego,XYbounds,nob,vob,A,b,fixTime,xWS,uWS,lwarm,nwarm,xWS_rs)  
            obj.x0 =x0;      %起始点
            obj.xF =xF;      %终点
            obj.N =N;        %预测步长
            obj.nx =nx;      %状态变量维度【X ,Y ,Yaw ,v 】
            obj.nu =nu;      %控制量维度 【delta,acc】
            obj.Ts =Ts;      %计算步长
            obj.L = L;       %轴距
            obj.ego = ego;   %本车尺寸
            obj.XYbounds = XYbounds; %搜索边界
            obj.nob =nob;    % 障碍物个数
            obj.vob=vob;     % 顶点数，不考虑无接触可能的顶点
            obj.A=A;         %障碍物空间描述矩阵
            obj.b =b;        %障碍物空间描述矩阵
            obj.xWS =xWS;    %初始状态量热启动
            obj.uWS=uWS;      %初始控制量热启动
            obj.fixTime =fixTime;
            obj.lwarm =lwarm;
            obj.nwarm =nwarm;
            obj.xWS_rs =xWS_rs;% RS结果
%             obj.dmin =dmin;
        end
        
        function [x,u,Timescale,existflag,l,n] = optimizer(obj)
           x = obj.xWS; 
           %定义变量个数 and Set initial guess for optimization variables  -------
           n_x = (obj.N+1) * obj.nx;  %%%                 x 
           n_u = obj.N*obj.nu;       %%%                 u 
           n_l = sum(obj.vob)*(obj.N+1);      %%%                 l 
           n_n = obj.nob*size(x,1)*(obj.N+1); %%%                 n 
           n_weight = obj.no_weight;
           n_timescale = obj.N;             %%%                 timescale 
           
           numvars = n_x + n_u + n_l  + n_n  + n_weight;
           varsguess = [reshape(obj.xWS,[n_x,1]); reshape(obj.uWS(:,1:obj.N),[n_u,1]); obj.lwarm; obj.nwarm; ones(n_weight,1)]; % 初始值
           %定义约束
           lb = [repmat([obj.XYbounds(1) obj.XYbounds(3) -inf -1]', obj.N+1,1); repmat([-0.6 -0.4]', obj.N,1); zeros(n_n + n_l,1); zeros(n_weight,1)];
           ub = [repmat([obj.XYbounds(2) obj.XYbounds(4) inf 2]',   obj.N+1,1); repmat([0.6 0.4]',   obj.N,1);   inf(n_n + n_l,1);  inf(n_weight,1)];
%            nonlcon = @(vars) lcon(vars,x);  %非线性约束
           Ain = [-diag(ones(1,obj.N*obj.nu), 0)+ diag(ones(1,(obj.N-1)*obj.nu), obj.nu); diag(ones(1,obj.N*obj.nu), 0)- diag(ones(1,(obj.N-1)*obj.nu), obj.nu)];
           Aineq_u   =[Ain(1:obj.N-obj.nu,:); Ain(obj.N+1:end-obj.nu,:)];
           Aineq =   [ zeros(size(Aineq_u,1), n_x)  Aineq_u  zeros(size(Aineq_u,1), n_l+n_n+n_weight) ]; %%blkdiag(  diag(zeros(obj.N+1)),  Aineq_u,  diag(zeros(n_l+n_n)) );
           Bineq=obj.Ts * [0.6*ones(obj.N-1,1);  0.6*ones(obj.N-1,1);  -0.6*ones(obj.N-1,1); -0.6*ones(obj.N-1,1)];
           Aeq= [eye(obj.nx) zeros(obj.nx,numvars-obj.nx);  zeros(obj.nx,n_x-obj.nx) eye(obj.nx) zeros(obj.nx,numvars- n_x)];  % x0 = x0; xF =xF
           Beq= [obj.x0  obj.xF]';
           nonlcon=@(vars) obj.Nonlcon(vars);
           if obj.fixTime == 0
%                [~,~,~,~,timescale] = splitvariables(obj,vars);
               %%%        x     u     l     n           timeScale
               numvars = n_x + n_u + n_l + n_n  + n_weight + n_timescale;
               varsguess = [reshape(obj.xWS,[n_x,1]); reshape(obj.uWS(:,1:obj.N),[n_u,1]); obj.lwarm; obj.nwarm; ones(n_weight,1); ones(n_timescale,1)]; % 初始值
               lb = [repmat([obj.XYbounds(1) obj.XYbounds(3) -inf -1]', obj.N+1,1); repmat([-0.6 -0.4]', obj.N,1); zeros(n_n + n_l,1); zeros(n_weight,1); 0.8*ones(n_timescale,1)];
               ub = [repmat([obj.XYbounds(2) obj.XYbounds(4) inf 2]',   obj.N+1,1); repmat([0.6 0.4]',   obj.N,1);   inf(n_n + n_l,1);  inf(n_weight,1) ; 1.2*ones(n_timescale,1)];
               Aineq =   [ zeros(size(Aineq_u,1), n_x)  Aineq_u  zeros(size(Aineq_u,1), n_l+n_n+n_weight+n_timescale) ]; %%blkdiag(  diag(zeros(obj.N+1)),  Aineq_u,  diag(zeros(n_l+n_n)) );
               Bineq=obj.Ts * [0.6*ones(obj.N-1,1);  0.6*ones(obj.N-1,1);  -0.6*ones(obj.N-1,1); -0.6*ones(obj.N-1,1)];
               Aeq= [eye(obj.nx) zeros(obj.nx,numvars-obj.nx);  zeros(obj.nx,n_x-obj.nx) eye(obj.nx) zeros(obj.nx,numvars- n_x)];
               Beq= [obj.x0  obj.xF]';
               nonlcon=@(vars) obj.Nonlcon(vars);
           end
           
           
           % 定义目标函数
           costfun = @(vars) obj.objadd(vars) ;


           % 定义算法
            options = optimoptions('fmincon',...
                                   'Display','iter',...
                                   'Algorithm', 'interior-point',... % 'interior-point',... % 'sqp','interior-point'
                                   'SpecifyConstraintGradient',false,...
                                   'UseParallel',true,... %为真时，求解器用并行方式估计梯度。通过设置为默认值false，可以禁用
                                   'ConstraintTolerance',obj.tol,... 
                                   'MaxIterations',obj.maxiter,... 
                                   'MaxFunctionEvaluations',11*numvars);  %寻优最大迭代次数
           [vars_opt,fval,existflag,~] = fmincon(costfun,varsguess,Aineq,Bineq,Aeq,Beq,lb,ub,nonlcon,options);
                    %                      fmincom（目标函数；初始值；线性不等式约束A，b；线性等式约束Aeq，beq，变量x的上、下界， 非线性等式/不等式，定义优化参数）
                    %         [x,fval]=fmincon(fun,   x0,           A,b,                            Aeq,beq,                         lb,ub ,                   nonlcon,              options)

                    %------------------ Output results  ---------------------------
                    % split variables since vars_opt = [x_opt; u_opt; e_opt]
            [x,u,l,n,~,Timescale] = splitvariables(obj, vars_opt);
            
           %*********************************************************************
        end
        
        function cost =objadd(obj,vars)   % d = -g'*mu + (A*t - b)*lambda
            [x,u,~,~,weight,timeScale] = obj.splitvariables(vars);            
            
            u0 = [0,0]'; 
            cost_du =0;
            cost_X =0;
            cost_curveLength =0;
            cost_u =0.01*sum(u(:,1).^2) + 0.5*sum(u(:,2).^2);
            [A_L,G1, G2, G3]  = Lagdual(obj,vars);
            cost_AL = sum(sum(obj.Relu(A_L)));% + sum(sum(obj.Relu(-A_L)));  % 0<=norm(A'*lambda)-1 <= 0
            cost_G1 = 1*sum(sum(obj.Relu(G1)+obj.Relu(-G1)));    % 0<=G'*mu + R'*A*lambda <=0
            cost_G2 = 1*sum(sum(obj.Relu(G2)+obj.Relu(-G2)));    % 0<=G'*mu + R'*A*lambda <=0
            cost_G3 = sum(sum(obj.Relu(G3)));
            diff_X = diff(x(1,:));
            diff_Y = diff(x(2,:));
            for i = 1:obj.N+1
                if i < obj.N
                    cost_du = cost_du + 0.1*((u(1,i+1)-u(1,i))/(timeScale(i)*obj.Ts))^2 + 0.1*((u(2,i+1)-u(2,i))/(timeScale(i)*obj.Ts))^2;
                    cost_curveLength = cost_curveLength + sqrt(diff_X(i)^2+diff_Y(i)^2);
                end
                cost_X = cost_X + 0.1*(x(1,i)-obj.xWS_rs(1,i))^2 + 0.1*(x(2,i)-obj.xWS_rs(2,i))^2 + 0.1*(x(3,i)-obj.xWS_rs(3,i))^2;
            end
            cost_du = cost_du +(0.1*((u(1,1)-u0(1,1))/(timeScale(1)*obj.Ts))^2 + 0.1*((u(2,1)-u0(2,1)) /(timeScale(1)*obj.Ts))^2);
            if obj.fixTime == 0
                cost_timeScale = sum(0.5*timeScale + 0.5*timeScale.^2 );
            else
                cost_timeScale = 0;
            end
            cost =  cost_AL + cost_G1 + cost_G2 + 1e1*cost_G3 + 1e1*cost_u + 1e3*cost_X + 1e1*cost_du + cost_timeScale*0 + 1e2* (cost_curveLength + sqrt(diff_X(end)^2+diff_Y(end)^2)  ) ;
        end  
        
        function [x,u,l,n,weight,Timescale] = splitvariables(obj,vars)
        %------------------------------------------------------------------
        % args:
        %   vars: <optSize,1> optimization variables
        % out:
        %   uvec: <m,N>
        %   evec: <ne,N>
        %------------------------------------------------------------------
            % split variables
            length_x = (obj.N+1)*obj.nx;
            length_u = obj.N*obj.nu; 
            x = reshape(vars(1:length_x),[obj.nx, obj.N+1]);
            u = reshape(vars(1+length_x:length_u+length_x),[obj.nu, obj.N]);
            l = vars( 1+length_x+length_u: (obj.N+1)*sum(obj.vob)+length_x+length_u);
            n = vars( 1+length_x + length_u + length(l) : obj.nob*size(obj.xWS,1)*(obj.N+1) + length_x + length_u + length(l));
            weight = vars( 1+length_x + length_u + length(l) +length(n)  :  length_x + length_u + length(l) +length(n) +obj.no_weight);
            if obj.fixTime == 0
                Timescale = vars( 1+length_x + length_u + length(l) + length(n) + length(weight) :end);                
            else
                Timescale = ones(obj.N,1);
            end
             % reshape the column vector <2*N,1> to <2,N>
            l = reshape(l, sum(obj.vob), obj.N+1);
            n = reshape(n, obj.nob*size(obj.xWS,1), obj.N+1); 
                       
        end
        

          
        function xK_=f(obj,x,u,timeScale)
            %%泊车运动学模型
            L = obj.L;
            Ts =obj.Ts;
            
%             if fixTime == 1  % sampling time is fixed
%                 xK_(1) = x(1) + Ts*(x(4) + Ts/2*u(2))*cos((x(3) + Ts/2*x(4)*tan(u(1))/L));
%                 xK_(2) = x(2) + Ts*(x(4) + Ts/2*u(2))*sin((x(3) + Ts/2*x(4)*tan(u(1))/L));
%                 xK_(3) = x(3) + Ts*(x(4) + Ts/2*u(2))*tan(u(1))/L;
%                 xK_(4) = x(4) + Ts*u(2);
%             else			% sampling time is variable
                xK_(1) = x(1) + timeScale*Ts*(x(4) + timeScale*Ts/2*u(2))*cos((x(3) + timeScale*Ts/2*x(4)*tan(u(1))/L));
                xK_(2) = x(2) + timeScale*Ts*(x(4) + timeScale*Ts/2*u(2))*sin((x(3) + timeScale*Ts/2*x(4)*tan(u(1))/L));
                xK_(3) = x(3) + timeScale*Ts*(x(4) + timeScale*Ts/2*u(2))*tan(u(1))/L;
                xK_(4) = x(4) + timeScale*Ts*u(2);
%             end                            
        end
        

        
        function [cineq,ceq] = Nonlcon(obj, vars) %非线性约束
        % function [cineq,ceq,gradvars_cineq,gradvars_ceq] = nonlcon(obj, vars, t0, x0)
        %------------------------------------------------------------------
        % Evaluate nonlinear equality and inequality constraints
        % out:
        %   cineq = g(x,u) <= 0 : inequality constraint function
        %   ceq   = h(x,u) == 0 : equality constraint function
        %   gradx_cineq(x,u): gradient of g(x,u) w.r.t. x
        %   gradx_ceq(x,u):   gradient of h(x,u) w.r.t. x
        %------------------------------------------------------------------             
            
            % init vectors to speedup calculations
            ceq1     = zeros(obj.nx, obj.N);
%             cineq = []; % provided inequality constraints (g<=0)
            
            % vars_size = obj.optSize();
            % gradvars_cineq = zeros(vars_size,obj.n);
        
            % split variables
            [x,u,~,~,~,Timescale] = splitvariables(obj,vars);
            
            
            % calculate state sequence for given control input sequence and x0
%             xk = obj.predictStateSequence(vars,u);
            for i =1: obj.N
                xK = obj.f(x(:,i),u(:,i),Timescale(i));                
                ceq1(:,i) =   x(:,i+1)  -  xK'   ;            
            end
            
            
           [A_L,G1, G2, G3]  = Lagdual(obj,vars);
           ceq =[reshape(ceq1,obj.N*obj.nx,1);reshape(A_L,(obj.N+1)*obj.nob,1);reshape(G1,(obj.N+1)*obj.nob,1);reshape(G2,(obj.N+1)*obj.nob,1)];
           cineq =  reshape(G3,(obj.N+1)*obj.nob,1); 
           
        end
        
        function [A_L,G1, G2, G3]  = Lagdual(obj,vars) %线性约束
           ego = obj.ego ; % 后轴中心为参考点原点定义尺寸 % 汽车尺寸: (x_upper+x_lower)+(y_upper+y_lower) % [x_upper;y_upper;-x_lower;-y_lower]后两项为负值
           A_L =NaN(obj.N+1,obj.nob);
           G1 = NaN(obj.N+1,obj.nob);
           G2 = NaN(obj.N+1,obj.nob);
           G3 =NaN(obj.N+1,obj.nob);
           W_ev = ego(2)+ego(4);
           L_ev = ego(1)+ego(3);
           g = [L_ev/2,W_ev/2,L_ev/2,W_ev/2]';
           % offset from the center of the ego
           offset = (ego(1)+ego(3))/2 - ego(3);
           [x,~,l,n,~,~] = splitvariables(obj,vars);
           for  i = 1:obj.N+1
               for j = 1: obj.nob
                   if j ==1
                       vob1=1;
                   else
                       vob1 = sum(obj.vob(1:j-1)) + 1;
                   end
                   vob2 = sum(obj.vob(1:j)) ;
                   Aj = obj.A(vob1:vob2,:);     %第j个障碍物的相关矩阵
                   lj = l(vob1:vob2,:);     %第j个障碍物的lambda对偶变量
                   nj = n((j-1)*4+1:j*4 ,:);%第j个障碍物的mu对偶变量
                   bj = obj.b(vob1:vob2);       %第j个障碍物的相关矩阵
               % norm(A'*lambda) = 1
                   sum1 = 0;
                   sum2 = 0;
                   sum3 = 0;
                   sum4 = 0;
                   for k = 1:obj.vob(j)
                       sum1 = sum1 + Aj(k,1)*lj(k,i);
                       sum2 = sum2 + Aj(k,2)*lj(k,i); 
                       sum4 = sum4 + bj(k)*lj(k,i);
                   end
                    A_L(i,j) = sum1^2 + sum2^2 -1 ;  % norm(A'*lambda) -1 
               % G'*mu + R'*A'*lambda = 0      G =[1 0; 0 1; -1 0; 0 -1]
                   G1(i,j) = nj(1,i) - nj(3,i) + cos(x(3,i))*sum1 + sin(x(3,i))*sum2;
                   G2(i,j) = nj(2,i) - nj(4,i) - sin(x(3,i))*sum1 + cos(x(3,i))*sum2;
                   
                   
                   % -g'*mu + (A*t - b)*lambda > dmin offset是从t这里做了偏移
                   for k = 1:4
                       sum3 = sum3 + g(k)*nj(k,i);  %g'*mu
                   end
                   G3(i,j) = obj.dmin -(-sum3 + (x(1,i)+cos(x(3,i))*offset)*sum1 + (x(2,i)+sin(x(3,i))*offset)*sum2 - sum4);   % dmin-(-g'*mu + (A*t - b)*lambda )
                    
               end
           end
        end
        
        function error_relex= Relu(obj,error)
            lambda = -0.1;
            gamma = 1000;
            error_relex=5*(sqrt((4+gamma*(lambda-error).^2)/gamma) - (lambda-error));

        end
        
      
    end
     
end
        
        

        

   

