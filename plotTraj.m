function plotTraj(xp,up,N,ego,L,nOb,vOb,lOb,title1,Ts)
W_ev = ego(2)+ego(4);
L_ev = ego(1)+ego(3);
up = [up ; zeros(1,2)];
w = W_ev/2;
offset = L_ev/2 - ego(3);

% 初始状态
x0_s = xp(1,:);
Rot0 = [cos(x0_s(3)),-sin(x0_s(3));sin(x0_s(3)),cos(x0_s(3))];
x0 = [x0_s(1);x0_s(2)];
centerCar0 = x0 + Rot0*[offset;0];

% 最终状态
xF_s = xp(end,:);
RotF = [cos(xF_s(3)) -sin(xF_s(3)); sin(xF_s(3)) cos(xF_s(3))];
xF = [xF_s(1); xF_s(2)];
centerCarF = xF + RotF*[offset;0];

    for i = 1:1:N+1
        h=figure(121);
        width=600;%宽度，像素数
        height=270;%高度
        left=635;%距屏幕左下角水平距离
        bottem=200;%距屏幕左下角垂直距离
        set(gcf,'position',[left,bottem,width,height],'color','w')
        hold on
        xlabel('X [m]')
        ylabel('Y [m]')
        plot(xp(1:i,1),xp(1:i,2),"y",'LineWidth',4); 	% plot trajectory so far
        title(title1);
        hold on
        % plot trajectory
        for j = 1:nOb
            for k = 1 : vOb(j)
                plot([lOb{j}{k}(1),lOb{j}{k+1}(1)] , [lOb{j}{k}(2),lOb{j}{k+1}(2)], 'k');
            end
        end

        Rot = [cos(xp(i,3)) -sin(xp(i,3));sin(xp(i,3)) cos(xp(i,3))];
        x_cur = [xp(i,1);xp(i,2)];
        centerCar = x_cur + Rot*[offset;0];

        carBox(centerCar,xp(i,3),W_ev/2,L_ev/2);
        carBox(x_cur + (Rot*[L;w-0.15]), xp(i,3) + up(i,1),0.15,0.3);
        carBox(x_cur + (Rot*[L;-w+0.15]),xp(i,3) + up(i,1),0.15,0.3);
        carBox(x_cur + (Rot*[0; w-0.15]) ,xp(i,3),0.15,0.3);
        carBox(x_cur + (Rot*[0;-w+0.15]) ,xp(i,3),0.15,0.3);

        % plot start position
        plot(x0(1),x0(2),"p");
        carBox(centerCar0,x0_s(3),W_ev/2,L_ev/2);
        carBox(x0 + (Rot0*[L;w-0.15])  ,x0_s(3),0.15,0.3);
        carBox(x0 + (Rot0*[L;-w+0.15]) ,x0_s(3),0.15,0.3);
        carBox(x0 + (Rot0*[0; w-0.15]) ,x0_s(3), 0.15,0.3);
        carBox(x0 + (Rot0*[0;-w+0.15]) ,x0_s(3), 0.15,0.3);

        % plot end position
        carBox_dashed(centerCarF,xF_s(3),W_ev/2,L_ev/2);
        carBox_dashed(xF + (RotF*[L;w-0.15])  ,xF_s(3),0.15,0.3);
        carBox_dashed(xF + (RotF*[L;-w+0.15]) ,xF_s(3),0.15,0.3);
        carBox_dashed(xF + (RotF*[0; w-0.15]) ,xF_s(3), 0.15,0.3);
        carBox_dashed(xF + (RotF*[0;-w+0.15]) ,xF_s(3), 0.15,0.3);

        if i == N+1
            plot(xF(1),xF(2),"p");
            hold on
        end
    %     axis equal;


        rect = [40, 1, 540, 270];
        frame=getframe(h,rect);
        imind=frame2im(frame);
        [imind,cm] = rgb2ind(imind,256);
        if i==1
            imwrite(imind,cm,'optvideo.gif','gif', 'Loopcount',inf,'DelayTime',0.1*3);%第一次必须创建！
        else
            imwrite(imind,cm,'optvideo.gif','gif','WriteMode','append','DelayTime',0.1*3);
        end
        
            pause(Ts);
            
    end
end

function carBox(x0,phi,w,l)
    car1 = x0(1:2) + [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car2 = x0(1:2) + [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    car3 = x0(1:2) - [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car4 = x0(1:2) - [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    plot([car1(1),car2(1),car4(1),car3(1),car1(1)],[car1(2),car2(2),car4(2),car3(2),car1(2)],"k",'LineWidth',0.01);
    
end
function carBox_dashed(x0,phi,w,l)
    car1 = x0(1:2) + [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car2 = x0(1:2) + [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    car3 = x0(1:2) - [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car4 = x0(1:2) - [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    plot([car1(1),car2(1),car4(1),car3(1),car1(1)],[car1(2),car2(2),car4(2),car3(2),car1(2)],"k",'LineWidth',0.01);
    hold on
end