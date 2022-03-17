classdef Node1          % 类名
    properties          % 类属性
        x = 0;
        y = 0;
        cost = 0;
        pind = 0;
    end
    methods            % 类方法
        function obj = Node1(x,y,cost,pind) % 构造函数，声明的时候就定义了
            obj.x = x;
            obj.y = y;
            obj.cost = cost;
            obj.pind = pind;

        end
    end
end