% 从终点开始遍历a*可达所有cost
function pmap = a_star(gx,gy,ox,oy,reso,vr)
%    global GRID_RESOLUTION VEHICLE_RADIU
%    GRID_RESOLUTION = 1;
%    VEHICLE_RADIU = 1;
   
   nGoal = Node1(round(gx/reso),round(gy/reso),0,-1);
   
   ox = ox/reso;
   oy = oy/reso;
   
   [obmap, minx, miny, maxx, maxy, xw, yw] = calc_obstacle_map(ox, oy, reso, vr); %vr:VEHICLE_RADIUS
   
    %*********************************************************
    open = containers.Map;
    closed = containers.Map;
    open(num2str(calc_index(nGoal, xw, minx, miny))) = nGoal;
    motion = get_motion_model();      %上下左右+斜方向 的 方位与距离
    nmotion = length(motion(:,1));    %一共8个需要搜索的位置
    %pq = PriorityQueue()************************************
    %enqueue!(pq, calc_index(ngoal, xw, minx, miny), ngoal.cost)
    pq = [];               % 记录 序号 与 cost
    new = [calc_index(nGoal, xw, minx, miny), nGoal.cost]; %终点cost初默认为0
    pq = [pq;new];
    pq = sortrows(pq,2);   % 按 cost 从小到大进行排序
    %*******************************************************
    while true
        if isempty(open)               % 遍历结束都放入closed，则isempty(open)=1停止循环
            break
        end
 %************************************************************       
        c_id = pq(1,1);%最小cost对应序号
        pq(1,:) = [];%移除
        
        current = open(num2str(c_id)); %将开集中cost最小的搜索点 放到，闭集
        open.remove(num2str(c_id));
        closed(num2str(c_id)) = current;
%         figure(1)
%              hold on
%              plot(current.x*reso,current.y*reso,'x')
        
 %***************************************************
% 
%         delete!(open, c_id)
%         closed[c_id] = current
     for i = 1:nmotion               %从当前节点，扩大搜索栅格，依次判断这八个点
         node = Node1(current.x+motion(i,1),current.y+motion(i,2),current.cost+motion(i,3),c_id);%按方位扩展节点
         if ~verify_node(node, minx, miny,xw,yw,obmap)   %判断该点是否符合要求：1是否超出边界；2是否碰撞
             continue                            %如果不符合要求，则搜索下一个点
         end
         node_ind = calc_index(node, xw, minx, miny);  % 符合要求，结算该节点的序号
         if closed.isKey(num2str(node_ind))            % 判断该点是否已在闭集中，在则跳过，已计算过
             continue
         end
         if open.isKey(num2str(node_ind))              % 判断是否在开集中，在，继续判断
             node1 = open(num2str(node_ind))  ;        % 得到之前该序号的节点
             if node1.cost > node.cost    % 比较该节点的cost,如果之前的大
                 node1.cost = node.cost;  % 则替换成小值
                 node1.pind = c_id;       % 更新该点的父节点为扩展前的节点(原节点)
                 open(num2str(node_ind)) = node1 ;  %更新字典
             end
         else
             open(num2str(node_ind)) = node;                  % 不在，则放入开集中
             new = [node_ind,calc_cost(node, nGoal)];
             pq = [pq;new];                                   % 也加入pq中，计算cost信息
             pq = sortrows(pq,2);
         end
     end
     
    end
    pmap = calc_policy_map(closed, xw, yw, minx, miny);
%     c = closed.keys;
%       for i = 1:length(closed)-1
%            d = string(c{length(closed)-i});
%            dd = string(c{length(closed)-i+1});
%            figure(1)
%              hold on
%              plot([closed(d).x*0.3,closed(dd).x*0.3],[closed(d).y*0.3,closed(dd).y*0.3],'-.')
%                 
%       end
   %***********************************************************

end
function  [rx, ry] = calc_astar_path(sx,sy,gx,gy,ox,oy,reso,vr)
   global GRID_RESOLUTION VEHICLE_RADIU
   GRID_RESOLUTION = 1;
   VEHICLE_RADIU = 1;
   
   nstart = Node1(round(sx/reso),round(sy/reso),0,-1);
   ngoal = Node1(round(gx/reso),round(gy/reso),0,-1);
  
   
   ox = ox/reso;
   oy = oy/reso;
   
   [obmap, minx, miny, maxx, maxy, xw, yw] = calc_obstacle_map(ox, oy, reso, vr);
   

    %*********************************************************
    open = containers.Map;
    closed = containers.Map;
    
    
    
    open(num2str(calc_index(nstart, xw, minx, miny))) = nstart;
    motion = get_motion_model();      %上下左右+斜方向 的 方位与距离
    nmotion = length(motion(:,1));    %一共8个需要搜索的位置
    
    %pq = PriorityQueue()************************************
    %enqueue!(pq, calc_index(ngoal, xw, minx, miny), ngoal.cost)
    pq = [];               % 记录 序号 与 cost
    new = [calc_index(nstart, xw, minx, miny), calc_cost(nstart, ngoal)]; %终点初默认为0
    pq = [pq;new];
    pq = sortrows(pq,2);   % 按 cost 进行排序
    

    %*******************************************************
    while true
        if isempty(open)
            disp("Error: No open set");
            break
        end
     c_id = pq(1,1);%最小cost对应序号
     pq(1,:) = [];%移除
        
     current = open(num2str(c_id)); %将开集中cost最小的搜索点 放到，闭集
     
     if current.x == ngoal.x && current.y == ngoal.y % check goal
        disp("Goal!!");
        closed(num2str(c_id)) = current;
         break
      end
     
     open.remove(num2str(c_id));
     closed(num2str(c_id)) = current;  
     
     for i = 1:nmotion               %从当前节点，扩大搜索栅格，依次判断这八个点
         node = Node1(current.x+motion(i,1),current.y+motion(i,2),current.cost+motion(i,3),c_id);
         if ~verify_node(node, minx, miny, xw, yw, obmap)   %判断该点是否符合要求：1是否超出边界；2是否碰撞
             continue                            %如果不符合要求，则搜索下一个点
         end
         node_ind = calc_index(node, xw, minx, miny);  % 符合要求，则添加序号
         if closed.isKey(num2str(node_ind))            % 判断该点是否已在闭集中，在则跳过，已计算过
             continue
         end
         if open.isKey(num2str(node_ind))              % 判断是否在开集中，在，继续判断
             if open(num2str(node_ind)).cost > node.cost    % 如果开集中该点的cost大于之前的cost
                 node.cost = node.cost;  %
                 node.pind = c_id;       % 更新该点的父节点
             end
         else
             open(num2str(node_ind)) = node;                  % 不在，则放入开集中
             new = [node_ind,calc_cost(node, ngoal)];
             pq = [pq;new];                               % 也加入pq中，计算cost信息
             pq = sortrows(pq,2);
         end
     end
    end
     [rx, ry] = get_final_path(closed, ngoal, nstart, xw, minx, miny, reso) ;
     
end
function pmap = calc_policy_map(closed, xw, yw, minx, miny)
   pmap = inf(xw,yw);
   c = closed.keys;
  for i = 1:length(closed)
       d = string(c{i});
       pmap(closed(d).x-minx, closed(d).y-miny) = closed(d).cost;       
  end
end



function isnode = verify_node(node,minx,miny,xw,yw,obmap)
    isnode = true;
    if (node.x - minx) >= xw 
        isnode = false;
    elseif (node.x - minx) <= 0 
        isnode = false;
    end

    if (node.y - miny) >= yw
        isnode = false;
    elseif (node.y - miny) <= 0 
        isnode = false;
    end

    %collision check
    if (node.x-minx) == 0||(node.y-miny)== 0
        isnode = false;
    else
    if obmap(node.x-minx, node.y-miny)
        isnode = false;
    end 
    end
end

function calc_cost = calc_cost(n,ngoal)
    calc_cost = (n.cost + h(n.x - ngoal.x, n.y - ngoal.y)); 
end

function motion = get_motion_model()
    motion = [1 0 1;
          0 1 1;
          -1 0 1;
          0 -1 1;
          -1 -1 sqrt(2);
          -1 1 sqrt(2);
          1 -1 sqrt(2);
          1 1 sqrt(2);];
end

function c = calc_index(node,xwidth,xmin,ymin)
   c = (node.y - ymin)*xwidth + (node.x -xmin);
   % 将 x y 的记号划分为两个区域，同时y的区域为x宽度的倍数，x为使得 两部分相加没有重叠，且二维降为一维
end


function [obmap, minx, miny, maxx, maxy, xwidth, ywidth] = calc_obstacle_map(ox,oy,reso,vr)
%     ox: x position list of Obstacles [m] 障碍物范围等同于地图边界
%     oy: y position list of Obstacles [m]
    minx = round(min(ox));
    miny = round(min(oy));
    maxx = round(max(ox));
    maxy = round(max(oy));
    xwidth = round(maxx-minx);
    ywidth = round(maxy-miny);
    
    obmap = false(xwidth,ywidth);%建立地图
    kdtree = KDTreeSearcher([ox;oy]'); %建立障碍物边界kdtree搜索模型
    for ix = 1:xwidth
        x = ix+minx;
        for iy = 1:ywidth
            y = iy+miny;
            [idxs,onedist] = knnsearch(kdtree,[x,y]); %对于地图中任意一点搜索与之相关的障碍物最近点
            if onedist(1) <= vr/reso
                obmap(ix,iy) = true;
            end
        end
    end
end

function [rx,ry] = get_final_path(closed,ngoal,nstart,xw,minx,miny,reso)
     rx = [ngoal.x];
     ry = [ngoal.y];
     nid = calc_index(ngoal,xw,minx,miny);
     while true
         n = closed(num2str(nid));
         rx = [rx;n.x];
         ry = [ry;n.y]; % 都记为列向量
         nid = n.pind;
         if rx(end) == nstart.x && ry(end) == nstart.y
            % disp("done");
            break
         end
     end
     rx = flipud(rx).*reso;
     ry = flipud(ry).*reso;        
end

function mnode = search_min_cost_node(open,ngoal)
     mnode = NaN;
     mcost = inf;
     c = open.keys;
     for i = 1:length(open)
       d = string(c{i});
       cost = open(d).cost+h(open(d).x-ngoal.x,open(d).y-ngoal.y);
        if mcost > cost
            mnode = open(d) ;
            mcost = cost ;
        end
    end 
end

function y = h(x,y)
     y = sqrt(x^2+y^2);
end

% main