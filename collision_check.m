function iscollision = collision_check(x,y,yaw,kdtree,ox,oy)   %用于hybrid a碰撞检测
     B = 1.0; % 后轴到车尾距离
     C = 3.7; % 后轴到车头距离
     I = 2.0; % 车宽
     WBUBBLE_DIST = (B+C)/2.0-B; % 后轴到车中心距离
%     WBUBBLE_R = (B+C)/2.0;      % 车长一半
     WBUBBLE_R = hypot((B+C)/2,I/2);
%      vrx = [C, C, -B, -B, C ];
%      vry = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]; 

     iscollision = true;        % 默认碰撞
     
     for i = 1:length(x)
        ix = x(i);
        iy = y(i);
%         plot(ix,iy,'*')
        iyaw = yaw(i);
        cx = ix + WBUBBLE_DIST*cos(iyaw);
        cy = iy + WBUBBLE_DIST*sin(iyaw);
        % whole bubble check
        ids = rangesearch(kdtree,[cx,cy],WBUBBLE_R);
        if isempty(ids)          %则无“碰撞”
            continue
        end
       %****************************************
        ids = cell2mat(ids);   %转为矩阵
        ox1 = [];
        oy1 = [];
        for j = 1:length(ids)
            ox1(end+1) = ox(ids(j));
            oy1(end+1) = oy(ids(j));
        end
       %****************************************
        if ~rect_check(ix,iy,iyaw,ox1,oy1) %若有“碰撞”，矩形检测，是否真碰撞
            iscollision = false;
        end
     end
end

function iscollision = rect_check(ix,iy,iyaw,ox,oy)
% iscollision = true;
% B = 1;
% C = 3.7;
% I = 2;
% vrx = [C, C, -B, -B, C ]';
% vry = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]';
% R = [cos(iyaw),-sin(iyaw);
%      sin(iyaw),cos(iyaw)];
%  vrx1 = [];
%  vry1 = [];
%  for i =1:length(vrx)
%    vr = R*[vrx(i);vry(i)]+[ix;iy];
%    vrx1(end+1) = vr(1);
%    vry1(end+1) = vr(2);
%  end
% [in,on] = inpolygon(ox,oy,vrx1,vry1);
% if sum(in) > 0 || sum(on) > 0 
%     iscollision = true ;
%     return
% else
%     iscollision = false;
%     return
% end

% function iscollision = rect_check(ix,iy,iyaw,ox,oy)
     iscollision = true;
     B = 1.0;%后轴到车尾距离
     C = 3.7;%后轴到车头距离
     I = 2.0;%车宽
     vrx = [C, C, -B, -B, C ]';
     vry = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]';
     c = cos(-iyaw);
    s = sin(-iyaw);
%     figure(1)
%     plot(ix,iy,'o')
%     quiver(ix,iy,cos(iyaw),sin(iyaw),'r','MaxHeadSize',0.8,'LineWidth',1,'AutoScaleFactor',0.89)
%     hold on
%     plot(ox,oy,'r.','LineWidth',5)
     for i = 1:length(ox)
         iox = ox(i);
         ioy = oy(i);
         tx = iox -ix;   % iox,ioy为障碍点；      ix，iy为被检查的规划的轨迹点
         ty = ioy -iy;
         lx = (c*tx - s*ty); %投影到车辆偏航角方向
         ly = (s*tx + c*ty);
%          hold on
%          plot(linspace(0,lx,10),linspace(0,ly,10),'k.')
         sumangle = 0;
         for j = 1:length(vrx) - 1
             x1 = vrx(j) - lx;   %车的相邻顶点  与   轨迹点指向各个障碍点
             y1 = vry(j) - ly;
%              plot(linspace(0,vrx(j),10),linspace(0,vry(j),10),'b.')
             x2 = vrx(j+1) - lx;
             y2 = vry(j+1) - ly;
%              plot(linspace(0,vrx(j+1),10),linspace(0,vry(j+1),10),'b.')
             d1 = hypot(x1,y1); %  sqrt(x^2 + y^2)
             d2 = hypot(x2,y2);
             theta1 = atan2(y1,x1);
             tty = -sin(theta1)*x2 + cos(theta1)*y2; % 如果矢量【x2,y2】与矢量【x1,y1】+90方向的夹角为锐角，则夹角为正，否则为负数。cos(theta1+pi/2)*x2+sin(theta1+pi/2)*y2
             tmp = (x1*x2+y1*y2)/(d1*d2);
%              figure(3)
%              hold on
%              plot([0 x1],[0 y1],'r')
%              plot(x1,y1,'*')
%              plot([0 x2],[0 y2],'k')
%              plot(x2,y2,'o')
%              plot(0,0,'p')
%              axis equal
%              hold off
             if tmp >=1
                 tmp = 1;
             end
             if tty >= 0
                 sumangle = sumangle + acos(tmp);
             else
                 sumangle = sumangle - acos(tmp);
             end
         end
         
         if sumangle >= pi
            iscollision = false;
            return
         end
     end

end
