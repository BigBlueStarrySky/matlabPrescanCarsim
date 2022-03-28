function [path_x_final_set_gcs,path_y_final_set_gcs] = QpSmoothPath(path_x_init_set_gcs,path_y_init_set_gcs,...
    w_smooth,w_length,w_ref,x_lb,x_ub,y_lb,y_ub)
%该函数将使用二次规划算法来平滑给定轨迹path
%二次规划通用形式 g(x) = 0.5 * x' * H * x + f' * x;约束条件 lb <= x <= ub 通过迭代求解x的最小值
% 输入 path_x_init_set_gcs,path_y_init_set_gcs 未平滑的给定轨迹点集合 
% w_smooth,w_length,w_ref 平滑代价，紧凑代价，几何相似代价权重系数 x_lb,x_ub,y_lb,y_ub 允许坐标x,坐标y变化的上下界
% 输出 path_x_final_set_gcs,path_y_final_set_gcs平滑后的给定轨迹

%simulink无法直接使用自带的二次规划函数，需要从外部引入
coder.extrinsic("quadprog");
% 二次规划形式  g(x) = 0.5 * x' * H * x + f' * x
%代价函数costfunction = x'*(w_smooth*A1'*A1+w_length*A2'*A2+w_ref*A3'*A3)*x +w_ref*h'*x
% 转换成二次规划形式 costfunction=0.5*x'*2[(w_smooth*A1'*A1+w_length*A2'*A2+w_ref*A3'*A3)]*x+ w_ref*h'*x
% 可以得到 H = 2[(w_smooth*A1'*A1+w_length*A2'*A2+w_ref*A3'*A3)] f'=w_ref*h' f = w_ref * h
% 首先计算出H和f  则需要写出A1 A2 A3  其中x=(x1,y1,x2,y3,x3,y3.....xn,yn)'  2n*1  xi,yi是平滑后轨迹点坐标
% 对参数进行初始化
% 计算平滑前轨迹点的个数
n = length(path_x_init_set_gcs);
% 对输出就行初始化
path_x_final_set_gcs = ones(n,1)*nan; path_y_final_set_gcs = ones(n,1)*nan; 
% 对权重矩阵进行初始化
A1 = zeros(2 * n - 4, 2 * n); A2 = zeros(2 * n - 2, 2 * n); A3 = eye(2 * n, 2 * n); h = ones(2*n,1)*nan;
% 对平滑后轨迹点(x,y)上下限初始化
lb = zeros(2 * n, 1);    %(x,y)坐标的下限
ub = zeros(2 * n, 1);    %(x,y)坐标的上限
% 对平滑矩阵赋值
for i = 1:(2*n-4)
    A1(i,i) = 1;
    A1(i,i+2) = -2;
    A1(i,i+4) = 1;
end
% 对紧凑矩阵赋值
for i = 1:(2*n-2)
    A2(i,i) = -1;
    A2(i,i+2) = 1;
end
% 对(x,y)上下限和与参考线距离向量赋值
for i = 1:n
    h(2*i-1) = -2 * path_x_init_set_gcs(i);
    h(2*i) = -2 * path_y_init_set_gcs(i);
    lb(2*i-1) = path_x_init_set_gcs(i) + x_lb;
    ub(2*i-1) = path_x_init_set_gcs(i) + x_ub;
    lb(2*i) = path_y_init_set_gcs(i) + y_lb;
    ub(2*i) = path_y_init_set_gcs(i) + y_ub;
end

% 计算二次规划参数
H = 2 * (w_smooth*(A1'*A1)+w_length*(A2'*A2)+w_ref*(A3'*A3));
f = w_ref * h;
%用 优化前path_init作为二次规划的迭代起点
X0 = zeros(2 * n, 1); %X0为迭代初值  初始化
for i = 1:n
    X0(2*i-1) = path_x_init_set_gcs(i);
    X0(2*i) = path_y_init_set_gcs(i);
end
% 调用二次规划模块求解
X = quadprog(H,f,[],[],[],[],lb,ub,X0); %迭代出平滑后的轨迹(x,y)坐标 X=[x1,y1,x2,y2,x3,y3,...xn,yn]
% 将优化后的结果赋值到新坐标中
for i = 1:n
    path_x_final_set_gcs(i) = X(2*i-1);
    path_y_final_set_gcs(i) = X(2*i);
end
end