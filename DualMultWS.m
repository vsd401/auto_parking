classdef DualMultWS < handle
    
    
    properties
        tol =10e-3;
        maxiter = 200       % max solver iterations
        N
        nob
        vob
        A
        b
        xWS
        varsguess
        numvars
        dmin = 0.05;			% anything bigger than 0, e.g. 0.05
    end
    
    methods
        
        function obj = DualMultWS(N,nob,vob,A,b,xWS)
            obj.N =N;
            obj.nob =nob;
            obj.vob=vob;
            obj.A=A;
            obj.b =b;
            obj.xWS =xWS;
        end
        
        function [lp,np,l_mat,n_mat,existflag] = optimizer(obj)
           x = obj.xWS; 
           %定义变量个数：
           %%%                  l                           n
%            numvars = sum(obj.vob)*(obj.N+1)   + obj.nob*size(x,1)*(obj.N+1) ;
           %-------- Set initial guess for optimization variables  -------
%            varsguess = zeros(numvars,1)+0.1; % 初始值
           %定义约束
           lb = zeros(obj.numvars,1);
           ub = inf(obj.numvars,1);
%          nonlcon = @(vars) lcon(vars);  %非线性约束
           Aineq =[];
           Bineq=[];
           Aeq=[];
           Beq=[];
           nonlcon=[];
           % 定义目标函数
           costfun = @(vars) obj.objadd(vars,x) ;


           % 定义算法
            options = optimoptions('fmincon',...
                                   'Display','iter',...
                                   'Algorithm', 'interior-point',... % 'interior-point',... % 'sqp','interior-point'
                                   'SpecifyConstraintGradient',false,...
                                   'UseParallel',false,... %为真时，求解器用并行方式估计梯度。通过设置为默认值false，可以禁用
                                   'ConstraintTolerance',obj.tol,... 
                                   'MaxIterations',obj.maxiter,... 
                                   'MaxFunctionEvaluations',5*obj.numvars);  %寻优最大迭代次数
           [vars_opt,fval,existflag,~] = fmincon(costfun,obj.varsguess,Aineq,Bineq,Aeq,Beq,lb,ub,nonlcon,options);
                    %                      fmincom（目标函数；初始值；线性不等式约束A，b；线性等式约束Aeq，beq，变量x的上、下界， 非线性不等式，定义优化参数）
                    %         [x,fval]=fmincon(fun,   x0,           A,b,                            Aeq,beq,                         lb,ub ,                   nonlcon,              options)

                    %------------------ Output results  ---------------------------
                    % split variables since vars_opt = [x_opt; u_opt; e_opt]
            [lp, np,l_mat,n_mat] = splitvariables(obj, vars_opt);
           %*********************************************************************
        end
        
        function cost =objadd(obj,vars,x)   % d = -g'*mu + (A*t - b)*lambda
            [A_L,G1, G2, G3]  = obj.lcon(vars,x);
            cost1 = sum(sum(G3));
            cost2 = sum(sum(obj.Relu(A_L)));  % norm(A'*lambda)-1 <= 0
            cost3 = 0.01* sum(sum(obj.Relu(G1)+obj.Relu(-G1)));    % 0<=G'*mu + R'*A*lambda <=0
            cost4 = 0.01* sum(sum(obj.Relu(G2)+obj.Relu(-G2)));    % 0<=G'*mu + R'*A*lambda <=0
            cost5 = sum(sum(obj.Relu(G3)));
            cost = cost2 +cost3 + cost4 + cost5;  %cost1+
        end  
        
        function [l,n,l_mat,n_mat] = splitvariables(obj,vars)
        %------------------------------------------------------------------
        % args:
        %   vars: <optSize,1> optimization variables
        % out:
        %   uvec: <m,N>
        %   evec: <ne,N>
        %------------------------------------------------------------------
            % split variables
            
            l = vars( 1: (obj.N+1)*sum(obj.vob));
            n = vars(  length(l) +1 : obj.nob*size(obj.xWS,1)*(obj.N+1) + length(l) );
            
            
            % reshape the column vector <2*N,1> to <2,N>
            l_mat = reshape(l, sum(obj.vob), obj.N+1);
            n_mat = reshape(n, obj.nob*size(obj.xWS,1), obj.N+1); 
        end
        
        function [A_L,G1, G2, G3]  = lcon(obj,vars,x) %线性约束
           ego = [3.7,1,1,1];
           G3 = NaN(obj.N+1,obj.nob);
           A_L =NaN(obj.N+1,obj.nob);
           G1 = NaN(obj.N+1,obj.nob);
           G2 = NaN(obj.N+1,obj.nob);
           W_ev = ego(2)+ego(4);
           L_ev = ego(1)+ego(3);
           g = [L_ev/2,W_ev/2,L_ev/2,W_ev/2]';
           % offset from the center of the ego  ego =[后轴中心到前端 后轴中心到左端 后轴中心到后端 后轴中心到右端]
           offset = (ego(1)+ego(3))/2 - ego(3); %轴距一半
           [~,~,l,n] = splitvariables(obj,vars);
           for  i = 1:obj.N+1
               for j = 1: obj.nob
                   if j ==1
                       vob1=1;
                   else
                       vob1 = sum(obj.vob(1:j-1)) + 1;
                   end
                   vob2 = sum(obj.vob(1:j)) ;
                   Aj = obj.A(vob1:vob2,:);     %第j个障碍物的相关矩阵
                   lj =     l(vob1:vob2,:);     %第j个障碍物的lambda对偶变量
                   nj = n((j-1)*4+1:j*4 ,:);%第j个障碍物的mu对偶变量
                   bj = obj.b(vob1:vob2);       %第j个障碍物的相关矩阵
               % norm(A'*lambda) <= 1
                   sum1 = 0;
                   sum2 = 0;
                   sum3 = 0;
                   sum4 = 0;
                   for k = 1:obj.vob(j)
                       sum1 = sum1 + Aj(k,1)*lj(k,i);
                       sum2 = sum2 + Aj(k,2)*lj(k,i); 
                       sum4 = sum4 + bj(k)*lj(k,i);
                   end
                   A_L(i,j) = sum1^2 + sum2^2 -1 ;
               % G'*mu + R'*A*lambda = 0
                   G1(i,j) = nj(1,i) - nj(3,i) + cos(x(3,i))*sum1 + sin(x(3,i))*sum2;
                   G2(i,j) = nj(2,i) - nj(4,i) - sin(x(3,i))*sum1 + cos(x(3,i))*sum2;
               % -g'*mu + (A*t - b)*lambda > 0
                   for k = 1:4
                       sum3 = sum3 + g(k)*nj(k,i);
                   end
                   G3(i,j) = obj.dmin-(-sum3 + (x(1,i)+cos(x(3,i))*offset)*sum1 + (x(2,i)+sin(x(3,i))*offset)*sum2 - sum4);   % -(-g'*mu + (A*t - b)*lambda )
               end
           end
        end
        
        function error_relex= Relu(obj,error)
            lambda = -0.1;
            gamma = 1000;
            error_relex=50*(sqrt((4+gamma*(lambda-error).^2)/gamma) - (lambda-error));

        end
        
      
    end
     
end
        
        

        

   

