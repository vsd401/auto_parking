classdef Node           % 类名
    properties          % 类属性
        xind = 0;
        yind = 0;
        yawind = 0;
        direction = 0;
        x = [];
        y = [];
        yaw = [];
        steer = 0;
        cost = inf;
        pind = 0;
    end
    methods            % 类方法
        function obj = Node(xind,yind,yawind,direction,x,y,yaw,steer,cost,pind) % 构造函数，声明的时候就定义了
            obj.xind = xind;
            obj.yind = yind;
            obj.yawind = yawind;
            obj.direction = direction;
            obj.x = x;
            obj.y = y;
            obj.yaw = yaw;
            obj.steer = steer;
            obj.cost = cost;
            obj.pind = pind;

        end
    end
end