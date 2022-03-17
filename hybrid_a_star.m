function [rx,ry,ryaw] = hybrid_a_star(sx,sy,syaw,gx,gy,gyaw,ox,oy,xyreso,yawreso,obreso)
    global VEHICLE_RADIUS BUBBLE_DIST OB_MAP_RESOLUTION YAW_GRID_RESOLUTION N_STEER XY_GRID_RESOLUTION
    global MOTION_RESOLUTION USE_HOLONOMIC_WITH_OBSTACLE_HEURISTIC USE_NONHOLONOMIC_WITHOUT_OBSTACLE_HEURISTIC
    global SB_COST BACK_COST STEER_CHANGE_COST STEER_COST H_COST
    global WB MAX_STEER
    VEHICLE_RADIUS = 1; % （后轮）后圆半径
    BUBBLE_DIST = 1.7;  %  距离“前泡泡”的距离
    OB_MAP_RESOLUTION = 0.1; % 障碍物分辨率
    YAW_GRID_RESOLUTION = deg2rad(5); % 转角分辨率，5度
    N_STEER = 5; % 转向指令数
    XY_GRID_RESOLUTION = 0.3; %XY栅格地图分辨率
    MOTION_RESOLUTION = 0.1;  %运动分辨率
    USE_HOLONOMIC_WITH_OBSTACLE_HEURISTIC = true; % 默认为 运动约束
    USE_NONHOLONOMIC_WITHOUT_OBSTACLE_HEURISTIC = false; 
    SB_COST = 10; % 转回惩罚
    BACK_COST = 0; % 倒车惩罚
    STEER_CHANGE_COST = 10; % 操纵角改变惩罚
    STEER_COST = 0; %操纵角惩罚
    H_COST = 1; %启发代价
    WB = 2.7; %轴距
    MAX_STEER = 0.6; %最大转向角 [rad]
    % config结构体****************************************************
%     config.minx; config.miny; config.minyaw; %范围最小值
%     config.maxx; config.maxy; config.maxyaw; %范围最大值
%     config.xw; config.yw; config.yaww;       %
%     config.xyreso; config.yawreso;           %分辨率
%     config.obminx; config.obminy;            %障碍物范围
%     config.obmaxx; config.obmaxy;
%     config.obxw; config.obyw;               
%     config.obreso;
%     % 
    %*******************************************************************
    
    syaw = pi_2_pi(syaw);   % 将角度限制在[-pi,pi]
    gyaw = pi_2_pi(gyaw);
    
    c = calc_config(ox,oy,xyreso,yawreso,obreso); % 求解，并定义基本参数
    
    kdtree = KDTreeSearcher([ox;oy]'); % 创建kdtree搜索模型
%     kdtree = kDTree([ox;oy]');%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [obmap,gkdtree] = calc_obstacle_map(ox,oy,c); %计算障碍物地图
    
    %（x index, y index, yaw index, ...
    %  [运动方向： 向前：true；后退：false], x位置， y位置， 角度
    %  操纵角 输入 ，  代价  ，   父节点index）
    nstart = Node(round(sx/xyreso),round(sy/xyreso),round(syaw/yawreso),...
             true,sx,sy,syaw,0,0,-1);
    ngoal = Node(round(gx/xyreso),round(gy/xyreso),round(gyaw/yawreso),...
             true,gx,gy,gyaw,0,0,-1);

    
    if USE_HOLONOMIC_WITH_OBSTACLE_HEURISTIC        % 完整约束，考虑障碍物
        h_dp = calc_holonomic_with_obstacle_heuristic(ngoal,ox,oy,xyreso);
            %    figure(33)
            %    bar3(pmap)
            %     set(gca,'YTickLabel',linspace(-12,9,8));
            %     set(gca,'XTickLabel',linspace(3,12,4));
            %     xlabel('Y/ m')
            %     ylabel('X/ m')
            %     zlabel('heuristic cost')
    else
        h_dp = [];% zeros()
    end
    if USE_NONHOLONOMIC_WITHOUT_OBSTACLE_HEURISTIC  % 非完整约束，不考虑障碍物
        h_rs = calc_nonholonomic_without_obstacle_heuristic(ngoal,c);
    else
        h_rs = [];
    end
    %***********************************************************
    open = containers.Map; %键值字典结构（或者用struct）
    closed = containers.Map;
    open(num2str(calc_index(nstart,c))) = nstart; %Key为string类型        ind 先放入起点
    %pq = PriorityQueue()
    %enqueue!(pq, calc_index(nstart, c), calc_cost(nstart, h_rs, h_dp, ngoal, c))
    pq = [];               % 记录 序号 与 cost
    
    a1=calc_index(nstart,c);
    a2=calc_cost(nstart, h_rs, h_dp, ngoal, c);
    new = [a1, a2]; %终点初默认为0               [序号，cost]
    pq = [pq;new];
    pq = sortrows(pq,2);   % 按 cost 进行排序,每次搜索点应为cost值最小的对应的序号!!
    
    %*******************************************************************
    [u,d] = calc_motion_inputs();
    nmotion = length(u);

    while true
        if isempty(open)    %如果开集为空，无可继续搜索的点，则未能找到路径
            disp("Error: Cannot find path");
            rx = NaN;
            ry = NaN;
            ryaw = NaN;
            return
        end
 %************************************************************       
        c_id = pq(1,1);    %最小cost对应序号,初始为起点
        pq(1,:) = [];      %取出预扩展的点的序号，同时从pq中移除，不再搜索
        
        current = open(num2str(c_id)); %从开集中得到当前搜索点
        plot(current.x(end),current.y(end),'o')
        hold on
 %***************************************************
       %通过rspath寻找该点是否可以达到终点，如果可以：
       %路径为当前节点（x,y,yaw,cost）的路径与RS路径之和，返回标记已找到
        [isupdated, current] = update_node_with_analystic_expantion(current, ngoal, obmap, c, kdtree, ox, oy);
        if isupdated
            closed(num2str(calc_index(ngoal,c))) = current; 
            break  % 达到终点，退出循环
        end
        %否则，继续从搜索，已计算过的c_id放入闭集
        open.remove(num2str(c_id)); 
        closed(num2str(c_id)) = current;
        %以c_id继续 扩展搜索
        for i = 1:nmotion
            node = calc_next_node(current,c_id,u(i),d(i),c,gkdtree); %通过运动学分析，更新下一个点的相关信息
%             plot(node.x,node.y,'y')
            if ~verify_index(node,obmap,c,kdtree,ox,oy)
                continue
            end
            node_ind = calc_index(node,c);
            % 如果已在闭集中，跳过
            if closed.isKey(num2str(node_ind))    %  K 大写
                continue
            end
            if ~open.isKey(num2str(node_ind))     % 不在开集，则付给开集
                open(num2str(node_ind)) = node;
                new = [node_ind,calc_cost(node, h_rs, h_dp, ngoal, c) ];%给出当前点的序号与cost
                pq = [pq;new];
                pq = sortrows(pq,2); %排序
            end
        end
    end
    %************************************************
 %    
    [rx,ry,ryaw] = get_final_path(closed, ngoal, nstart, c);
end

function [isupdated,current] = update_node_with_analystic_expantion(...
                                 current,ngoal,obmap,c,kdtree,ox,oy)
    apath = analystic_expantion(current,ngoal,obmap,c,kdtree,ox,oy);
    isupdated = false;
    if class(apath)~= "double"  % find path
        %%
        current.x = [current.x;apath.x(2:end-1)];   %记录路径
        current.y = [current.y;apath.y(2:end-1)];
        current.yaw = [current.yaw;apath.yaw(2:end-1)];
        current.cost = current.cost+calc_rs_path_cost(apath); 
        isupdated = true;  %路径已更新
        return
    end
end

function cost = calc_rs_path_cost(rspath)
   global MAX_STEER SB_COST BACK_COST STEER_CHANGE_COST STEER_COST 
    cost = 0;
    H_COST = 1;
    % for/backward penalty
    for l = 1:length(rspath.lengths)         
        if rspath.lengths(l) >= 0 %forward
            cost = cost+1;
        else      %backward
            cost = cost+abs(rspath.lengths(l))*BACK_COST;
        end
    end
    
    % switch back penalty
    for i = 1:length(rspath.lengths)-1
        if rspath.lengths(i)*rspath.lengths(i+1) < 0
            cost = cost+SB_COST;
        end
    end
    
    %steer penalty
    for ctype = 1:length(rspath.ctypes)
        if rspath.ctypes(ctype) ~= 'S'
            cost = cost+STEER_COST*abs(MAX_STEER);
        end
    end
    
    %steer change penalty
    nctypes = length(rspath.ctypes);
    ulist = zeros(nctypes,1);
    for i = 1:nctypes
        if rspath.ctypes(i) == 'R'
            ulist(i) = - MAX_STEER;
        elseif rspath.ctypes(i) == 'L'
            ulist(i) =  MAX_STEER;
        end
    end
    
    for i = 1:length(rspath.ctypes)-1
        cost = cost+STEER_CHANGE_COST*abs(ulist(i+1)-ulist(i));
    end
end

function path = analystic_expantion(n,ngoal,obmap,c,kdtree,ox,oy)
    global MAX_STEER WB MOTION_RESOLUTION
    sx = n.x(end);
    sy = n.y(end);
    syaw = n.yaw(end);
    max_curvature = tan(MAX_STEER)/WB; 
    [sx,sy,syaw];
    [ngoal.x,ngoal.y,ngoal.yaw(end)];
    path = reeds_shepp2(sx,sy,syaw,ngoal.x(end),ngoal.y(end),...
                     ngoal.yaw(end),max_curvature,MOTION_RESOLUTION);
    pfigure =plot(path.x,path.y,':', 'LineWidth', 1);
    pfigure.Color(4) = 1;
    if isempty(path.lengths)
       path = NaN;
       return
    end
    
    if ~collision_check(path.x, path.y, path.yaw, kdtree, ox, oy)
       path = NaN ;
       return
    end 
       [sx,sy,syaw];
       [path.x,path.y, path.yaw];
end

function [u,d] = calc_motion_inputs() %计算运动输入
%     up = [];
%     for i = MAX_STEER/N_STEER:MAX_STEER/N_STEER:MAX_STEER %单方向
%         up(end+1) = i;
%     end   
    global MAX_STEER N_STEER
    u1 = [];
    for i = MAX_STEER/N_STEER:MAX_STEER/N_STEER:MAX_STEER
        u1(end+1) = i;
    end
    u1 = u1';
    u2 = -u1;
    
    u = [0;u1;u2];
   
    d1 = [];           %前进为正，做标记
    for i = 1:length(u)
        d1(end+1) = 1;
    end
    d1 = d1';
    
    
    d2 = -d1;         %后退为负，做标记
    
    d = [d1;d2];
    u = [u;u];  
end

function istrue = verify_index(node,obmap,c,kdtree,ox,oy) 
    istrue = true;
    if (node.xind - c.minx) >= c.xw 
        istrue = false;
    elseif (node.xind -c.minx) <= 0
        istrue = false;
    end
    if (node.yind - c.miny) >= c.yw
        istrue = false;
    elseif (node.yind - c.miny) <= 0
        istrue = false;
    end
    
    if ~collision_check(node.x,node.y,node.yaw,kdtree,ox,oy) 
        istrue = false;
    end
end

function iangle = pi_2_pi(iangle)
    if iangle > pi
        iangle = iangle -2*pi;
    end
    if iangle < -pi
        iangle = iangle+2*pi;
    end
end

function node = calc_next_node(current,c_id,u,d,c,gkdtree)
    %该点的类包括：xind,yind,yawind,direction,x,y,yaw,steer,cost,pind
    global XY_GRID_RESOLUTION MOTION_RESOLUTION WB BACK_COST SB_COST STEER_COST STEER_CHANGE_COST
    arc_1 = XY_GRID_RESOLUTION;
    nlist = round(arc_1/MOTION_RESOLUTION)+1;
    xlist = zeros(nlist,1);
    ylist = zeros(nlist,1);
    yawlist = zeros(nlist,1);
    % 当前点的x,y,yaw
    xlist(1) = current.x(end)+d*MOTION_RESOLUTION*cos(current.yaw(end)); %方向 X 步长
    ylist(1) = current.y(end)+d*MOTION_RESOLUTION*sin(current.yaw(end));
    yawlist(1) = pi_2_pi(current.yaw(end)+d*MOTION_RESOLUTION/WB*tan(u));
    % 根据motion延申路径
    for i = 1:(nlist-1)
        xlist(i+1) = xlist(i)+d*MOTION_RESOLUTION*cos(yawlist(i)); %
        ylist(i+1) = ylist(i)+d*MOTION_RESOLUTION*sin(yawlist(i));
        yawlist(i+1) = pi_2_pi(yawlist(i)+d*MOTION_RESOLUTION/WB*tan(u));
    end
    
    %最终位置记为xind,yind,yawind
    xind = round(xlist(end)/c.xyreso);
    yind = round(ylist(end)/c.xyreso);
    yawind = round(yawlist(end)/c.yawreso);
    
    addedcost = 0.0;
    if d > 0
        direction = true;
        addedcost = addedcost + abs(arc_1);
    else
        direction = false;
        addedcost = addedcost + abs(arc_1) * BACK_COST;
    end

    % swich back penalty
    if direction ~= current.direction 
        addedcost = addedcost + SB_COST;
    end

    % steer penalyty
    addedcost = addedcost + STEER_COST*abs(u);

    % steer change penalty
    addedcost = addedcost+ STEER_CHANGE_COST*abs(current.steer - u);

    cost = current.cost + addedcost ;
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, u, cost, c_id);   
end

function is_true = is_same_grid(node1,node2)
    is_true = true;
    if node1.xind ~= node2.xind           
        is_true = false;
    end
    if node1.yind ~= node2.yind
        is_true = false;
    end
    if node1.yawind ~= node2.yawind
        is_true = false;
    end
end

function ind = calc_index(node,c)
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx);
    % 将序号从三维降为一维，且无重叠
    if ind <= 0
        disp("Error(calc_index):");
        disp(ind);
    end
end


function h_dp = calc_holonomic_with_obstacle_heuristic(gnode,ox,oy,xyreso)
VEHICLE_RADIUS = 1;
h_dp = a_star(gnode.x(end), gnode.y(end), ...
                           ox, oy, xyreso, VEHICLE_RADIUS);% a_star改成calc_dist_policy
end

function h_rs = calc_nonholonomic_without_obstacle_heuristic(ngoal,c)
   global MAX_STEER WB
    h_rs = zeros(c.xw,c.yw,c.yaww);
    max_curvature = tan(MAX_STEER)/WB;

    for ix = 1:c.xw
        for iy = 1:c.yw
            for iyaw = 1:c.yaww
                sx = (ix + c.minx)*c.xyreso;
                sy = (iy + c.miny)*c.xyreso;
                syaw = pi_2_pi((iyaw + c.minyaw)*c.yawreso);
                L = reeds_shepp2(sx, sy, syaw,... 
                            ngoal.x(end), ngoal.y(end), ngoal.yaw(end),...
                            max_curvature, MOTION_RESOLUTION); 
                h_rs(ix,iy,iyaw) = L;
            end
        end
    end
    
end


function config = calc_config(ox,oy,xyreso,yawreso,obreso)
    config.minx = round(min(ox)./xyreso);     %该xy栅格分辨率下，障碍物 xmin 
    config.miny = round(min(oy)./xyreso);     
    config.maxx = round(max(ox)./xyreso);     % 两种分辨率！
    config.maxy = round(max(oy)./xyreso);
    config.obminx = round(min(ox)./obreso);   %该障碍物栅格分辨率下，障碍物 xmin
    config.obminy = round(min(oy)./obreso);
    config.obmaxx = round(max(ox)./obreso);
    config.obmaxy = round(max(oy)./obreso);

    config.xw = round(config.maxx - config.minx);         % x方向范围
    config.yw = round(config.maxy - config.miny);         % y方向范围
    config.obxw = round(config.obmaxx - config.obminx);
    config.obyw = round(config.obmaxy - config.obminy);

    config.minyaw = round(- pi/yawreso) - 1;
    config.maxyaw = round(pi/yawreso);
    config.yaww = round(config.maxyaw - config.minyaw);              %范围
    
    config.xyreso = xyreso;
    config.yawreso = yawreso;
    config.obreso = obreso;

%     config = Config(minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww,...
%             xyreso, yawreso, obminx, obminy, obmaxx, obmaxy, obxw, obyw, obreso);
end

function [obmap,gkdtree] = calc_obstacle_map(ox,oy,c)
   global VEHICLE_RADIUS
    ox = ox/c.obreso;     %障碍物在障碍物分辨率中的表示
    oy = oy/c.obreso;

    obmap = false(c.obxw, c.obyw); % 逻辑判断不可行区域

    gkdtree = KDTreeSearcher([ox;oy]'); %建立KDtree模型
    for ix = 1:c.obxw 
        x = ix + c.obminx;        % 障碍物占据范围
        for iy = 1:c.obyw 
            y = iy + c.obminy;
%             [x,y]
            [idxs, onedist] = knnsearch(gkdtree, [x, y]);  
            if onedist(1) <= VEHICLE_RADIUS/c.obreso   %确定障碍物碰撞边界
                obmap(ix,iy) = true;
%                 plot(x*c.obreso,y*c.obreso,'*')
%                 hold on
            end
        end
    end
     save obmap
end

function [rx,ry,ryaw] = get_final_path(closed,ngoal,nstart,c)
    rx = ngoal.x;
    ry = ngoal.y;
    ryaw = ngoal.yaw;
    nid = calc_index(ngoal, c);
    
    while true
        n = closed(num2str(nid));
        rx = [rx; flipud(n.x)];% 倒序
        ry = [ry; flipud(n.y)];
        ryaw = [ryaw; flipud(n.yaw)];
        nid = n.pind;
        if is_same_grid(n, nstart)
           % disp("done")
            break
        end
    end

    rx = flipud(rx);%正序
    ry = flipud(ry);
    ryaw = flipud(ryaw);

    %dist = sum([sqrt(idx^2+idy^2) for (idx,idy) in zip(diff(rx), diff(ry))])
end

function cost = calc_cost(n,h_rs,h_dp,ngoal,c) 
    H_COST = 1;
    % cost = 过去点+当前点到终点
    cost = n.cost + H_COST*...
        calc_euclid_dist(n.x(end) - ngoal.x(end),  n.y(end) - ngoal.y(end),  n.yaw(end) - ngoal.yaw(end));
    if length(h_rs) > 1 && length(h_dp) > 1  % Both heuristic cost are activated
        c_h_dp = h_dp(n.xind - c.minx, n.yind - c.miny);
        c_h_rs = h_rs(n.xind - c.minx, n.yind - c.miny, n.yawind - c.minyaw);
        cost = n.cost + H_COST*max(c_h_dp, c_h_rs);
        return
    elseif length(h_dp) > 1 % Distance policy heuristics is activated
        cost = n.cost + H_COST*h_dp(n.xind - c.minx, n.yind - c.miny);
        return
    elseif length(h_rs) > 1 % Reed Sheep path heuristics is activated
        cost = n.cost + H_COST*h_rs(n.xind - c.minx, n.yind - c.miny, n.yawind - c.minyaw);
        return
    end    
end

function dist = calc_euclid_dist(x,y,yaw)
    if yaw >= pi
        yaw = yaw -pi;
    elseif yaw <= -pi
        yaw =yaw +pi;
    end
    dist = sqrt(x^2+y^2+yaw^2);
end
