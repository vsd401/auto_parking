%%
function path = reeds_shepp2(sx,sy,syaw,gx,gy,gyaw,maxc,step_size)
if nargin<8
    step_size = 0.1;
end
   paths = calc_paths(sx,sy,syaw,gx,gy,gyaw,maxc,step_size);
   
   minL =inf;
   best_path_index = -1;
   a = size(paths);
   b = a(1,2);
   for i = 1:b
%        plot(paths(i).x,paths(i).y,'')
       if paths(i).L<=minL
           minL = paths(i).L;
           best_path_index = i;
       end
   end
    path = paths(best_path_index);
end
%% 
function iangle = pi_2_pi(iangle)
if iangle>pi
    iangle = iangle -2*pi;
elseif iangle<-pi
    iangle = iangle+2*pi;
end
end
%%
function path0 = calc_paths(sx,sy,syaw,gx,gy,gyaw,maxc,step_size)
    q0 = [sx, sy, syaw];
    q1 = [gx, gy, gyaw];
    
    paths = generate_path(q0, q1, maxc);
    a = size(paths);
    b = a(1,2);
    path0 = [];
    for i = 1:b
        path = paths(i);
        [x, y, yaw, directions] = generate_local_course(path.L, path.lengths, path.ctypes, maxc, step_size*maxc) ;
        path.directions = directions;
        path.L = path.L/maxc;
        for j = 1:length(x)
            ix = x(j);
            iy = y(j);
            iyaw = yaw(j);
            path.x(j,1) = cos(-q0(3))*ix + sin(-q0(3))*iy +q0(1);
            path.y(j,1) =  -sin(-q0(3)) * ix + cos(-q0(3)) * iy + q0(2);
            path.yaw(j,1) = pi_2_pi(iyaw + q0(3));
        end
        for k = 1:length(path.lengths)
            path.lengths(k) = path.lengths(k)/maxc ;
        end 
        path0 = [path0,path];
    end
end
%%
function [r,theta] = polar(x,y)
r = sqrt(x^2+y^2);
theta = atan2(y,x);
end
%%
function v = mod2pi(x)
v = mod(x,2*pi);
if v <-pi
    v = v+2*pi;
else
    if v >pi
        v= v-2*pi;
    end
end
end
%%
function [flag,t,u,v]=LSL(x,y,phi)

[u,t]=polar(x-sin(phi),y-1+cos(phi));
if t >=0 
    v = mod2pi(phi-t);
    if v>= 0
        flag = true;
        return
    end
end
flag =false;u = 0;t = 0;v =0;
end
%%
function [flag ,t ,u, v] = LSR(x,y,phi)
  flag = false; t = 0; u = 0; v = 0;
    [u1, t1] = polar(x + sin(phi), y - 1.0 - cos(phi));
    u1 = u1^2;
    if u1 >= 4.0
        u = sqrt(u1 - 4.0);
        theta = atan2(2.0, u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);

        if t >= 0.0 && v >= 0.0
            flag = true; 
          
        end
    end
end
%%
function [flag ,t ,u, v] = LRL(x,y,phi)

  [u1, t1] = polar(x - sin(phi), y - 1.0 + cos(phi));

    if u1 <= 4.0
        u = -2.0*asin(0.25 * u1);
        t = mod2pi(t1 + 0.5 * u + pi);
        v = mod2pi(phi - t + u);

        if t >= 0.0 && u <= 0.0
            flag = true;
            return
        end
    end
     flag = false; t = 0; u = 0; v = 0;
end
%%
function Paths = set_path(paths,lengths,ctypes)
    path = Path([],[],0.0,[],[],[],[]);
    path.ctypes = ctypes;
    path.lengths = lengths;
    
    % 检查是否为相同路径，相同则直接输出
    a = size(paths);
    for i = 1:a(1,2)
        tpath = paths(i);
        b1 = size(tpath.ctypes);
        b2 = size(path.ctypes);
        if b1(1,2) ==  b2(1,2)
            typeissame = (tpath.ctypes==path.ctypes);
        else
            typeissame = false;
        end
        if typeissame
            if sum(tpath.lengths - path.lengths) <= 0.01
                Paths = paths;
                return
            end
        end
    end
     L = 0 ; 
     for i = 1:length(lengths)
        L = L +abs(lengths(i));
     end
     path.L = L;
     Paths = [paths,path];   

end

function paths = SCS(x,y,phi,paths)
     [flag, t, u, v] = SLS(x, y, phi) ;
    if flag 
        paths = set_path(paths, [t, u, v], ["S","L","S"]);
    end
   [ flag, t, u, v] = SLS(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, u, v], ["S","R","S"]);
    end
end

function [flag, t, u, v] =SLS(x,y,phi)
 flag = false; t = 0; u = 0; v = 0;
    if y > 0.0 && phi > 0.0 && phi < pi*0.99
        xd = - y/tan(phi) + x;
        t =  xd - tan(phi/2.0);
        u = phi;
        v = sqrt((x-xd)^2+y^2)-tan(phi/2.0);
        flag = true;
    elseif y < 0.0 && phi > 0.0 && phi < pi*0.99
        xd = - y/tan(phi) + x;
        t =  xd - tan(phi/2.0);
        u = phi;
        v = -sqrt((x-xd)^2+y^2)-tan(phi/2.0);
        flag = true;
    end
end

function paths = CSC(x,y,phi,paths)
 [flag, t, u, v] = LSL(x, y, phi) ;
    if flag
      paths = set_path(paths, [t, u, v], ["L","S","L"]);
    end
    [flag, t, u, v] =LSL(-x, y, -phi) ;
    if flag
        paths = set_path(paths,  [-t, -u, -v], ["L","S","L"]);
    end
   [ flag, t, u, v] = LSL(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, u, v], ["R","S","R"]);
    end
    [flag, t, u, v] = LSL(-x, -y, phi) ;
    if flag
        paths = set_path(paths, [-t, -u, -v], ["R","S","R"]);
    end

    
    [flag, t, u, v] = LSR(x, y, phi) ;
    if flag
        paths = set_path(paths, [t, u, v], ["L","S","R"]);
    end
   [ flag, t, u, v] = LSR(-x, y, -phi) ;
    if flag
        paths = set_path(paths,[-t, -u, -v], ["L","S","R"]);
    end
  [  flag, t, u, v ]= LSR(x, -y, -phi) ;
    if flag

        paths = set_path(paths, [t, u, v], ["R","S","L"]);
    end
    [flag, t, u, v] = LSR(-x, -y, phi) ;
    if flag

        paths = set_path(paths, [-t, -u, -v], ["R","S","L"]);
    end
end

function paths = CCC(x,y,phi,paths)
 [flag, t, u, v] = LRL(x, y, phi) ;
    if flag
        paths = set_path(paths, [t, u, v], ["L","R","L"]);
    end
   [ flag, t, u, v] = LRL(-x, y, -phi) ;
    if flag
        paths = set_path(paths, [-t, -u, -v], ["L","R","L"]);
    end
   [ flag, t, u, v] = LRL(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, u, v], ["R","L","R"]);
    end
    [flag, t, u, v] = LRL(-x, -y, phi) ;
    if flag
        paths = set_path(paths, [-t, -u, -v], ["R","L","R"]);
    end

    xb = x*cos(phi) + y*sin(phi);
    yb = x*sin(phi) - y*cos(phi);
  

    [flag, t, u, v ]= LRL(xb, yb, phi);
    if flag
        paths = set_path(paths, [v, u, t], ["L","R","L"]);
    end
   [ flag, t, u, v ]= LRL(-xb, yb, -phi);
    if flag
        paths = set_path(paths, [-v, -u, -t], ["L","R","L"]);
    end
   [ flag, t, u, v] = LRL(xb, -yb, -phi);
    if flag
        paths = set_path(paths, [v, u, t], ["R","L","R"]);
    end
   [ flag, t, u, v] = LRL(-xb, -yb, phi);
    if flag
        paths = set_path(paths, [-v, -u, -t], ["R","L","R"]);
    end
 
end
function [tau,omega] = calc_tauOmega(u,v,xi,eta,phi)
     delta = mod2pi(u-v);
    A = sin(u) - sin(delta);
    B = cos(u) - cos(delta) - 1.0;

    t1 = atan2((eta*A - xi*B),(xi*A + eta*B));
    t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;

    if t2 < 0
        tau = mod2pi(t1+pi);
    else
        tau = mod2pi(t1);
    end
    omega = mod2pi(tau - u + v - phi);
end

function [flag, t, u, v] =LRLRn(x,y,phi)
flag = false;t =0;u = 0;v=0;
 xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    rho = (2+sqrt(xi^2+eta^2))/4;

    if rho <= 1.0
        u = acos(rho);
       [ t, v ]= calc_tauOmega(u, -u, xi, eta, phi);
        if t >= 0 && v <= 0
            flag = true;
        end
    end
end

function [flag,t,u,v] = LRLRp(x,y,phi)
    flag = false; t = 0; u = 0; v = 0;
   xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    rho = (20.0 - xi*xi - eta*eta) / 16.0;

    if rho>=0.0 && rho<=1.0
        u = -acos(rho);
        if u >= - 0.5 * pi                               % 原为 u >= -0.5 * pi  改成
           [ t, v ]= calc_tauOmega(u, u, xi, eta, phi);
            if t >= 0.0 && v >= 0.0
                flag =true;
            end
        end
    end
end

function paths = CCCC(x,y,phi,paths)
[ flag, t, u, v] = LRLRn(x, y, phi) ;
    if flag
        paths = set_path(paths, [t, u, -u, v], ["L","R","L","R"]);
    end

   [ flag, t, u, v] = LRLRn(-x, y, -phi) ;
    if flag
        paths = set_path(paths, [-t, -u, u, -v], ["L","R","L","R"]);
    end
 
   [ flag, t, u, v] = LRLRn(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, u, -u, v], ["R","L","R","L"]);
    end
 
    [flag, t, u, v] = LRLRn(-x, -y, phi) ;
    if flag
        paths = set_path(paths, [-t, -u, u, -v], ["R","L","R","L"]);
    end
%**********************************************
   [ flag, t, u, v ]= LRLRp(x, y, phi) ;
    if flag
        paths = set_path(paths, [t, u, u, v], ["L","R","L","R"]);
    end

   [ flag, t, u, v] = LRLRp(-x, y, -phi) ;
    if flag
        paths = set_path(paths, [-t, -u, -u, -v], ["L","R","L","R"]);
    end

    [flag, t, u, v] = LRLRp(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, u, u, v], ["R","L","R","L"]);
    end

    [flag, t, u, v ]= LRLRp(-x, -y, phi) ;
    if flag
        paths = set_path(paths, [-t, -u, -u, -v], ["R","L","R","L"]);
    end

end

function [flag,t,u,v] = LRSR(x,y,phi)
 flag = false; t = 0; u = 0; v = 0;
 xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
   [ rho, theta] = polar(-eta, xi);

    if rho >= 2.0
        t = theta;
        u = 2.0 - rho;
        v = mod2pi(t + 0.5*pi - phi);
        if t >= 0.0 && u <= 0.0 && v <=0.0
            flag = true;
        end
    end
end

function [flag,t,u,v] = LRSL(x,y,phi)
flag = false; t = 0; u = 0; v = 0;
xi = x - sin(phi);
    eta = y - 1.0 + cos(phi);
    [rho, theta] = polar(xi, eta);

    if rho >= 2.0
        r = sqrt(rho*rho - 4.0);
        u = 2.0 - r;
        t = mod2pi(theta + atan2(r, -2.0));
        v = mod2pi(phi - 0.5*pi - t);
        if t >= 0.0 && u<=0.0 && v<=0.0
            flag =  true;
        end
    end
end

function paths = CCSC(x,y,phi,paths)
 [flag, t, u, v] = LRSL(x, y, phi) ;
    if flag
        paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","L"]);
    end

  [  flag, t, u, v] = LRSL(-x, y, -phi) ;
    if flag
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","L"]);
    end

  [  flag, t, u, v ]= LRSL(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","R"]);
    end

  [  flag, t, u, v] = LRSL(-x, -y, phi) ;
    if flag
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","R"]);
    end
    
   [ flag, t, u, v ]= LRSR(x, y, phi) ;
    if flag
        paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","R"]);
    end

   [ flag, t, u, v] = LRSR(-x, y, -phi) ;
    if flag
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","R"]);
    end

   [ flag, t, u, v] = LRSR(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","L"]);
    end

   [ flag, t, u, v ]= LRSR(-x, -y, phi) ;
    if flag
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","L"]);
    end
    
    xb = x*cos(phi) + y*sin(phi);
    yb = x*sin(phi) - y*cos(phi);
    [flag, t, u, v] = LRSL(xb, yb, phi) ;
    if flag
        paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","R","L"]);
    end

   [ flag, t, u, v ]= LRSL(-xb, yb, -phi);
    if flag
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","R","L"]);
    end

    [flag, t, u, v ]= LRSL(xb, -yb, -phi) ;
    if flag
        paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","L","R"]);
    end

   [ flag, t, u, v] = LRSL(-xb, -yb, phi) ;
    if flag
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","L","R"]);
    end
    [flag, t, u, v] = LRSR(xb, yb, phi) ;
    if flag
        paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","R","L"]);
    end

    [flag, t, u, v ]= LRSR(-xb, yb, -phi) ;
    if flag
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","R","L"]);
    end

    [flag, t, u, v] = LRSR(xb, -yb, -phi) ;
    if flag
        paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","L","R"]);
    end

    [flag, t, u, v ]= LRSR(-xb, -yb, phi) ;
    if flag
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","L","R"]);
    end
end
%%
function [flag,t,u,v] = LRSLR(x, y, phi)
flag = false; t = 0; u = 0; v = 0;
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
   [ rho, ~] = polar(xi, eta);
    if rho >= 2.0
        u = 4.0 - sqrt(rho*rho - 4.0);
        if u <= 0.0
            t = mod2pi(atan2((4.0-u)*xi -2.0*eta, -2.0*xi + (u-4.0)*eta));
            v = mod2pi(t - phi);

            if t >= 0.0 && v >=0.0
                flag = true;
                
            end
        end
    end
end
%%
function paths = CCSCC(x,y,phi,paths)
[flag, t, u, v] = LRSLR(x, y, phi) ;
 if flag
        paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["L","R","S","L","R"]);
    end
    [flag, t, u, v] = LRSLR(-x, y, -phi) ;
    if flag
        paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["L","R","S","L","R"]);
    end

   [ flag, t, u, v ]= LRSLR(x, -y, -phi) ;
    if flag
        paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["R","L","S","R","L"]);
    end

   [ flag, t, u, v] = LRSLR(-x, -y, phi) ;
    if flag
        paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["R","L","S","R","L"]);
    end

end
%%
function [px,py,pyaw,directions] = generate_local_course(L,lengths,mode,maxc,step_size)
 npoint = floor(L/step_size)+length(lengths)+3;
 px = zeros(npoint,1);
 py= zeros(npoint,1);
 pyaw= zeros(npoint,1);
 directions = zeros(npoint,1);
 ind = 2;
 if lengths(1) >0
     directions(1) = 1;
     d = step_size;
 else
     directions(1) = -1;
     d = -step_size;
 end
   pd = d;
   ll = 0;
   
   for j = 1:length(mode)
       m = mode(j);
       l = lengths(j);
       i = j;
       
       if l>0
           d = step_size;
       else
           d = -step_size;
       end
       ox = px(ind);
       oy = py(ind);
       oyaw = pyaw(ind);
       
       ind = ind -1;
       if i >=2 && (lengths(i-1)*lengths(i)) > 0
             pd = -d -ll;
       else
             pd =d -ll;
       end
         
       while abs(pd) <= abs(l)
             ind = ind+1;
             [px,py,pyaw,directions] = interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions);
             pd =pd +d;
       end
         
         ll = l - pd - d;
         ind = ind+1;
         [px, py, pyaw, directions] = interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions);
    end
         
    while px(end) == 0
             px(end) = [];
             py(end) = [];
             pyaw(end) = [];
             directions(end) = [];
    end
end

function [px,py,pyaw,directions] = interpolate(ind,l,m,maxc,ox,oy,oyaw,px,py,pyaw,directions)
    if m == "S"
        px(ind) = ox + l / maxc * cos(oyaw);
        py(ind) = oy + l / maxc * sin(oyaw);
        pyaw(ind) =  oyaw;
    else % curve
        ldx = sin(l) / maxc;
        if m == "L"  % left turn
            ldy = (1.0 - cos(l)) / maxc;
        elseif m == "R"  % right turn
            ldy = (1.0 - cos(l)) / (-maxc) ;
        end
        gdx = cos(-oyaw) * ldx + sin(-oyaw) * ldy ;
        gdy = -sin(-oyaw) * ldx + cos(-oyaw) * ldy;
        px(ind) = ox + gdx;
        py(ind) = oy + gdy;
    end

    if m == "L"  % left turn
        pyaw(ind) = oyaw + l;
    elseif m == "R"  % right turn
        pyaw(ind) = oyaw - l;
    end

    if l > 0.0
        directions(ind) = 1;
    else
        directions(ind) = -1;
    end
end

function paths = generate_path(q0,q1,maxc)
    dx = q1(1) - q0(1);  % x相对距离
    dy = q1(2) - q0(2);  % y相对距离
    dth = q1(3) - q0(3);
    c = cos(q0(3));
    s = sin(q0(3));
    x = (c*dx + s*dy)*maxc;   %maxc最大曲率也就是最小转弯半径
    y = (-s*dx + c*dy)*maxc;
    paths = [];
    paths = SCS(x, y, dth, paths);
    paths = CSC(x, y, dth, paths);
    paths = CCC(x, y, dth, paths);
    paths = CCCC(x, y, dth, paths);
    paths = CCSC(x, y, dth, paths);
    paths = CCSCC(x, y, dth, paths);
%     path0 = [paths1,paths2,paths3,paths4,paths5,paths6];
%     minL = inf;
%     for i = 1:length(path0)
%         if ~isempty(path0(i).L)
%             if minL > path0(i).L
%                 minL = path0(i).L;
%                 paths = path0(i);
%             end
%         end
%     end
                
end