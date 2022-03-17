function Feasible = ParkingConstraints(x0,xF,N,Ts,L,ego,XYbounds,nOb,vOb, A, b,x,u,l,n,timeScale,fixTime,sd)
dmin = 0.05; %比0大
    c0 = zeros(5,1);
	c1 = zeros(4,1);
	c2 = zeros(4,1);
	c3 = zeros(4,N);
	c4 = zeros(1,1);
	c5 = zeros(1,1);
	c6 = zeros(3,N+1);

	e = zeros(7,1);
    c0(1) = max(abs(u(1,:)))-0.6;
    c0(2) = max(abs(u(2,:)))-0.4;
    c0(3) = max(abs(timeScale-1))-0.2;
    c0(4) = -min(min(l));
    c0(5) = -min(min(n));
    
    %start point
    c1(1) = abs(x(1,1)-x0(1));
    c1(2) = abs(x(2,1)-x0(2));
    c1(3) = abs(x(3,1)-x0(3));
    c1(4) = abs(x(4,1)-x0(4));
    %end point
    c2(1) = abs(x(1,N+1)-xF(1));
    c2(2) = abs(x(2,N+1) - xF(2));
	c2(3) = abs(x(3,N+1) - xF(3));
	c2(4) = abs(x(4,N+1) - xF(4));
    
    %dynamics of the car
    for i = 1:N
        if fixTime == 1
            c3(1,i) = x(1,i+1) - (x(1,i) + Ts*(x(4,i) + Ts/2*u(2,i))*cos((x(3,i) + Ts/2*x(4,i)*tan(u(1,i))/L)));
		    c3(2,i) = x(2,i+1) - (x(2,i) + Ts*(x(4,i) + Ts/2*u(2,i))*sin((x(3,i) + Ts/2*x(4,i)*tan(u(1,i))/L)));
		    c3(3,i) = x(3,i+1) - (x(3,i) + Ts*(x(4,i) + Ts/2*u(2,i))*tan(u(1,i))/L);
		    c3(4,i) = x(4,i+1) - (x(4,i) + Ts*u(2,i));
	    else
		    c3(1,i) = x(1,i+1) - (x(1,i) + timeScale(i)*Ts*(x(4,i) + timeScale(i)*Ts/2*u(2,i))*cos((x(3,i) + timeScale(i)*Ts/2*x(4,i)*tan(u(1,i))/L)));
		    c3(1,i) = x(2,i+1) - (x(2,i) + timeScale(i)*Ts*(x(4,i) + timeScale(i)*Ts/2*u(2,i))*sin((x(3,i) + timeScale(i)*Ts/2*x(4,i)*tan(u(1,i))/L)));
		    c3(1,i) = x(3,i+1) - (x(3,i) + timeScale(i)*Ts*(x(4,i) + timeScale(i)*Ts/2*u(2,i))*tan(u(1,i))/L);
		    c3(1,i) = x(4,i+1) - (x(4,i) + timeScale(i)*Ts*u(2,i));
        end
    end
    u0 = [0,0];
    
    if fixTime == 1
        c5 = max(abs(diff(u(1,:)))/Ts) - 0.6;
		c4 = 0;
	else
		c4 = max(abs(diff(timeScale)));
		c5 = max(abs(diff(u(1,:)))/(timeScale(1)*Ts)) - 0.6;
    end
    
    W_ev = ego(2)+ego(4);
	L_ev = ego(1)+ego(3);
    g = [L_ev/2,W_ev/2,L_ev/2,W_ev/2];
    offset = (ego(1)+ego(3))/2 - ego(3);
    
    for i = 1:N+1
        for j = 1: nOb
               if j ==1
                   vOb1=1;
               else
                   vOb1 = sum(vOb(1:j-1)) + 1;
               end
               vOb2 = sum(vOb(1:j)) ;
               Aj = A(vOb1:vOb2,:);     %第j个障碍物的相关矩阵
               lj = l(vOb1:vOb2,:);     %第j个障碍物的lambda对偶变量
               nj = n((j-1)*4+1:j*4 ,:);%第j个障碍物的mu对偶变量
               bj = b(vOb1:vOb2);       %第j个障碍物的相关矩阵
           % norm(A'*lambda) <= 1
               sum1 = 0;
               sum2 = 0;
               sum3 = 0;
               sum4 = 0;
               for k = 1:vOb(j)
                   sum1 = sum1 + Aj(k,1)*lj(k,i);
                   sum2 = sum2 + Aj(k,2)*lj(k,i); 
                   sum4 = sum4 + bj(k)*lj(k,i);
               end
               c6(1,i,j) = sum1^2 + sum2^2 -1 ;  % norm(A'*lambda) -1 
           % G'*mu + R'*A*lambda = 0
               c6(2,i,j) = nj(1,i) - nj(3,i) + cos(x(3,i))*sum1 + sin(x(3,i))*sum2;
               c6(3,i,j) = nj(2,i) - nj(4,i) - sin(x(3,i))*sum1 + cos(x(3,i))*sum2;
           % -g'*mu + (A*t - b)*lambda > dmin
               for k = 1:4
                   sum3 = sum3 + g(k)*nj(k,i);  %g'*mu
               end
               c7(1,i,j) = dmin -(-sum3 + (x(1,i)+cos(x(3,i))*offset)*sum1 + (x(2,i)+sin(x(3,i))*offset)*sum2 - sum4);   % dmin-(-g'*mu + (A*t - b)*lambda )
       end
    end
             
    e(1) =  (  max(c0)<= 5e-5*1000  ); %[u timescale,l,n]
	e(2) =  (  max(c1)<= 5e-5*1000  ); %[x0]
    e(3) =  (  max(c2)<= 5e-5*1000  ); %[xF]
	e(4) =  (  max(max(abs(c3)))<= 5e-5*1000*1.5  ); % [%[x0 ... xF]
	e(5) =  (  c4 <= 5e-5*10000     ); % [ det_timescale]
	e(6) =  (  c5 <= 5e-5*10000     ); % [ det_u]
	e(7) =  (  max(max(max(abs(c6)))) <= 5e-5 *1000  ); % [dual]
    e(8) =  (  max(max(max( c7 ))) <= 5e-5 *1000  ); % [dual] -g'*mu + (A*t - b)*lambda > dmin
    
    if sum(e) > 6 || sum(e) ==6
      Feasible = 1;
    else
        Feasible = 0;
    end
    
end





