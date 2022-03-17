function [v_barMM,a] = veloSmooth(v,amax,Ts)
   v_ex = zeros(length(v)+40,1);
   v_bar = zeros(4,length(v)+40);
   v_bar2 = zeros(4,length(v)+40);
   v_barMM = zeros(1,length(v));
   
   for i = 1:length(v)
       for j =1:4
           v_bar(j,i+19) = v(i);
           v_ex(i+19) = v(i);
       end
   end
   
   v_cut1 = 0.25*abs(v(1));
   v_cut2 = 0.25*abs(v(1))+abs(v(1));
   accPhase = round(abs(v(1))/amax/Ts);
   index1 = find((diff(v_ex)>v_cut1).*(diff(v_ex)<v_cut2));
   index2 = find(diff(v_ex)>v_cut2);

   index3 = find((diff(v_ex)<-v_cut1).*(diff(v_ex)>-v_cut2));
   index4 = find(diff(v_ex)<-v_cut2);
   
   if length(index1) >= 1 && index1(1) == 19
       index1(1) = index1(1)+1;
   end
   if length(index1) >= 1 && length(index3) >=1 && index1(1) == 19
       index3(1) = index3(1)+1;
   end
   
   for j = 1:length(index1)
       if v_ex(index1(j)) > v_cut1 || v_ex(index1(1)+1) > v_cut1
           v_bar(1,index1(j):index1(j)+accPhase) = linspace(0,abs(v(1)),accPhase+1);
       elseif v_ex(index1(j)) < -v_cut1 || v_ex(index1(1)+1) < -v_cut1
           v_bar(1,index1(j)-accPhase+1:index1(j)+1) = linspace(-abs(v(1)),0,accPhase+1);
       end
   end
   
   for j = 1:length(index3)
       if v_ex(index3(j)) > v_cut1 || v_ex(index3(j)+1) > v_cut1
          v_bar(2,index3(j)-accPhase+1:index3(j)+1) = linspace(abs(v(1)),0,accPhase+1);
       elseif v_ex(index3(j)) < -v_cut1 || v_ex(index3(j)+1) < -v_cut1
          v_bar(2,index3(j):index3(j)+accPhase) = linspace(0,abs(v(1)),accPhase+1);
       end
   end
   
   for j = 1:length(index2)
       v_bar(3,index2(j)-accPhase:index2(j)+accPhase) = linspace(-abs(v(1)),abs(v(1)),2*accPhase+1);
   end
   
   for j = 1:length(index4)
       v_bar(4,index4(j)-accPhase:index4(j)+accPhase) = linspace(abs(v(1)),-abs(v(1)),2*accPhase+1);
   end
   
   for i = 20:length(v)+19
       for j = 1:4
           if v_bar(j,i) == 0
               v_bar2(j,i) = v_bar(j,i);
           elseif sign(v_ex(i)) ~= sign(v_bar(j,i))
               v_bar2(j,i) = v_ex(i);
           else
               v_bar2(j,i) = v_bar(j,i);
           end
       end
   end
   
   for i = 20:length(v)+19
       if v_ex(i) > 0
          v_barMM(i-19) = min(v_bar2(:,i));
       else
          v_barMM(i-19) = max(v_bar2(:,i));
       end
   end
   
   a = diff(v_barMM')./Ts;  
   v_barMM = v_barMM';
end