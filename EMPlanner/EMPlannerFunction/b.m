function [referenceline_x_init_set_gcs, referenceline_y_init_set_gcs] = fun(host_match_point_index, ...
    path_x_set_gcs, path_y_set_gcs)
% 该函数将在给定路径上提取初始参考线
%  host_match_point_index 自车在给定路径上匹配点编号
%  path_x_set_gcs, path_y_set_gcs 给定路径坐标信息
%  referenceline_x_init_set_gcs, referenceline_y_init_set_gcs 初始参考线信息
% 在给定路径上 由匹配点编号往前进方向取150个点  往后退方向取30个点  总共181个点  给定路径一般肯定会多于181个点
% 需要判断 前进方向是否有150个点  或者后退方向是否有30个点
n = 181;
% 输出初始化
referenceline_x_init_set_gcs = ones(n,1)*nan; referenceline_y_init_set_gcs = ones(n,1)*nan;

if host_match_point_index <= 30
    % 后退方向不够30个点
    start_index = 1; % 从起点开始取
    end_index = 181;
elseif host_match_point_index + 150 > length(path_x_set_gcs)
    % 前进方向不够150个点
    start_index = host_match_point_index - 30;
    end_index = length(path_x_set_gcs);
else
    % 前进和后退方向点的数量均满足要求
    start_index = host_match_point_index - 30;
    end_index = host_match_point_index + 150;
end

% 在给定路径上提取181个点作为初始参考线
referenceline_x_init_set_gcs = path_x_set_gcs(start_index:end_index);
referenceline_y_init_set_gcs = path_y_set_gcs(start_index:end_index);
end