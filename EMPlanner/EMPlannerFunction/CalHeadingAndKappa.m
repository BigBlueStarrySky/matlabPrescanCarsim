function [path_heading_set_gcs,path_kappa_set_gcs] = CalHeadingAndKappa(path_x_set_gcs,path_y_set_gcs)
% 该函数通过给定路径采样点的全局坐标系下的x,y坐标计算采样点的heading角和kappa
% 输入：path_x_set_gcs,path_y_set_gcs 给定轨迹坐标 都是列向量
% 输出：path_heading_set_gcs path_kappa_set_gcs 给定轨径的heading和曲率
% 采用差分法
% 原理 heading = arctan(dy/dx)    某一点的切向角近似等于tan[(y(n+1)-y(n))/(x(n+1)-x(n))] 
% kappa = dheading / ds
% ds = (dx^2 + dy^2)^0.5
% 计算给定路径采样点的个数
n = length(path_x_set_gcs);
% 对输出初始化
path_heading_set_gcs = ones(n,1)*nan;
path_kappa_set_gcs = ones(n,1)*nan; 
% 对采样点x,y坐标进行差分 相当于下一个值与当前值的差值作为元素
dx_set_gcs = diff(path_x_set_gcs);
dy_set_gcs = diff(path_y_set_gcs);
% 差分会导致少一个点 使用中点欧拉法补充该点
dx_forward_set_gcs = [dx_set_gcs(1);dx_set_gcs]; % 向前欧拉法  补充第一个点
dx_backward_set_gcs = [dx_set_gcs;dx_set_gcs(end)]; % 向后欧拉法  补充最后一个点
dx_mid_set_gcs = (dx_forward_set_gcs + dx_backward_set_gcs) ./ 2; % 中点欧拉法 相加除以2   ./表示向量里的元素相除
dy_forward_set_gcs = [dy_set_gcs(1);dy_set_gcs]; % 向前欧拉法  补充第一个点
dy_backward_set_gcs = [dy_set_gcs;dy_set_gcs(end)]; % 向后欧拉法  补充最后一个点
dy_mid_set_gcs = (dy_forward_set_gcs + dy_backward_set_gcs) ./ 2; % 中点欧拉法 相加除以2   ./表示向量里的元素相除

% 采用atan2可以避免分母为0的情况无法计算   并且heading的取值范围为[-pi,pi] 与atan2的取值范围相同
% atan2是一个函数，在C语言里返回的是指方位角 y和x的值的符号决定了正确的象限
path_heading_set_gcs = atan2(dy_mid_set_gcs, dx_mid_set_gcs);

% 对heading进行差分
dheading_set_gcs = diff(path_heading_set_gcs);
dheading_forward_set_gcs = [dheading_set_gcs(1);dheading_set_gcs]; % 向前欧拉法  补充第一个点
dheading_backward_set_gcs = [dheading_set_gcs;dheading_set_gcs(end)]; % 向后欧拉法  补充最后一个点
dheading_mid_set_gcs = (dheading_forward_set_gcs + dheading_backward_set_gcs) ./ 2; % 中点欧拉法 相加除以2   ./表示向量里的元素相除

% 计算ds = (dx^2 + dy^2)^0.5
ds_set_gcs = sqrt(dx_mid_set_gcs.^2 + dy_mid_set_gcs.^2); %向量间元素计算 使用点乘或点除 对向量开方使用sqrt或者.^0.5

%为了防止dheading出现多一个2pi的错误(特别是方向转到x轴负方向时)，一般情况下真实的dheading较小，用sin(dheading)近似dheading
path_kappa_set_gcs = sin(dheading_mid_set_gcs) ./ ds_set_gcs;
end