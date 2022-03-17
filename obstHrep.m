function [A_all, b_all] = obstHrep(nOb,vOb,lOb)
   if nOb ~= length(lOb)   %判断障碍物个数是否符合要求
       disp('error in number of obstacles');
   end
   
   % contain the H-rep
   A_all = zeros(sum(vOb)-nOb,2);   % 向量方向
   b_all = zeros(sum(vOb)-nOb,1);   % 起点
   
   lazyCounter = 1;                 %计数
   
   for i = 1:nOb
       A_i = zeros(vOb(i)-1,2);    %两个顶点构成一个半平面，3个则求得两个向量
       b_i = zeros(vOb(i)-1,1);
       % 取两个连续顶点, 计算超平面
       for j = 1:vOb(i)-1
           v1 = lOb{i}{j};    %第一个顶点 
           v2 = lOb{i}{j+1};  %第二个顶点
           %find hyperplane passing through v1, v2
           if v1(1) == v2(1)  %如果两个顶点x坐标相等，则平行于y轴
               if v2(2) < v1(2)    % （顺时针方向）向下
                   A_tmp = [1,0];  % 则法线方向指向（1，0）
                   b_tmp = v1(1);  % 分割面为 x = v1(1) = v2(1) 
               else
                   A_tmp = [-1,0]; 
                   b_tmp = -v1(1); % -1
               end
           elseif v1(2) == v2(2)  %两顶点y坐标相等，则平行于x轴 
               if v1(1) < v2(1)   %（顺时针），向右
                   A_tmp = [0,1]; % 法线（0，1）
                   b_tmp = v1(2);
               else
                   A_tmp = [0,-1]; %同理
                   b_tmp = -v1(2);
               end
           else    %对于一般平面
               ab = [v1(1),1;v2(1),1] \ [v1(2) ; v2(2)];
               a = ab(1);
               b = ab(2);
               if v1(1) < v2(1)
                   A_tmp = [-a,1];
                   b_tmp = b;
               else
                   A_tmp = [a,-1];
                   b_tmp = -b;
               end
           end
           %记录
           A_i(j,:) = A_tmp;
           b_i(j) = b_tmp;
       end
       A_all(lazyCounter:lazyCounter+vOb(i)-2,:) = A_i;
       b_all(lazyCounter:lazyCounter+vOb(i)-2) = b_i;
       
       lazyCounter = lazyCounter + vOb(i)-1;
   end 
end