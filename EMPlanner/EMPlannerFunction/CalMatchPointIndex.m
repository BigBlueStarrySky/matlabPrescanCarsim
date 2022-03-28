function match_point_index = CalMatchPointIndex(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,...
    path_heading_set_gcs,path_kappa_set_gcs,start_find_index,increase_count_limit, forward_stable_backward)
% 该函数将求待求点在给定路径采样点下的匹配点编号
%输入:x_gcs,y_gcs待求点信息
%path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs 给定路径采样点信息集合
% start_find_index 给定路径采样点开始寻找编号   亦即是上周期该待求点的匹配点编号
% increase_count_limit 距离增长上限
% forward_stable_backward 决定遍历方向
% 输出初始化
match_point_index = -1;

increase_count = 0; % 记录距离连续增长次数
min_distance_square = inf; %   初始化待求点到给定路径采样点最小距离的平方
if forward_stable_backward > 0.001 % 从开始寻找点往给定路径采样点编号增大方向遍历
    for i = start_find_index:length(path_x_set_gcs)
        distance_square = (x_gcs - path_x_set_gcs(i))^2 + (y_gcs - path_y_set_gcs(i))^2;
        if distance_square < min_distance_square
            min_distance_square = distance_square;
            match_point_index = i;  % 记录当前最小距离下标
            increase_count = 0;  
        else 
            increase_count = increase_count + 1;
        end 
        if increase_count > increase_count_limit
            break;   % 连续寻找increase_count_limit次都没找到比最小距离还小的值 退出循环
        end 
    end
elseif forward_stable_backward > 0.001 % 从开始寻找点往给定路径采样点编号减小方向遍历
    for i = start_find_index:-1:1
        distance_square = (x_gcs - path_x_set_gcs(i))^2 + (y_gcs - path_y_set_gcs(i))^2;
        if distance_square < min_distance_square
            min_distance_square = distance_square;
            match_point_index = i;  % 记录当前最小距离下标
            increase_count = 0;  
        else 
            increase_count = increase_count + 1;
        end 
        if increase_count > increase_count_limit
            break;   % 连续寻找increase_count_limit次都没找到比最小距离还小的值 退出循环
        end 
    end
else  % 取开始寻找点 亦即上周期的匹配点编号作为本周期的匹配点编号
    match_point_index = start_find_index;
end
end