classdef Path           % 类名
    properties          % 类属性
        lengths = [];
        ctypes= [];
        L = 0;
        x = [];
        y = [];
        yaw = [];
        directions = [];
    end
    methods            % 类方法
        function obj = Path(lengths,ctypes,L,x,y,yaw,directions) % 构造函数，声明的时候就定义了
            obj.lengths = lengths;
            obj.ctypes = ctypes;
            obj.L = L;
            obj.x = x;
            obj.y = y;
            obj.yaw = yaw;
            obj.directions = directions;
        end
    end
end