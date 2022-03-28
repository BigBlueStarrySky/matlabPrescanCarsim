function [match_point_index_set,proj_point_x_set_gcs,proj_point_y_set_gcs,proj_point_heading_set_gcs,...
    proj_point_kappa_set_gcs] = fcn(x_set_gcs,y_set_gcs,path_x_set_gcs,path_y_set_gcs,...
    path_heading_set_gcs,path_kappa_set_gcs)
% 该函数将批量计算点在给定路径下的匹配点编号和投影点信息   
% x_set_gcs,y_set_gcs 输入点信息集合
% path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs 给定路径点信息集合
% match_point_index_set 匹配点编号集合
% proj_point_x_set_gcs,proj_point_y_set_gcs,proj_point_heading_set_gcs,proj_point_kappa_set_gcs 投影点信息集合

% 假设最多有181个点需要投影
n = 181;
% 对输出进行初始化
match_point_index_set = ones(n,1) *nan; 
proj_point_x_set_gcs = ones(n,1) *nan;  proj_point_y_set_gcs = ones(n,1) *nan;
proj_point_heading_set_gcs = ones(n,1) *nan;  proj_point_kappa_set_gcs = ones(n,1) *nan;

% 寻找匹配点时 需要利用上一个周期的结果 因此定义需要利用的静态变量
persistent is_first_run;
persistent pre_match_point_index_set;
persistent pre_path_x_set_gcs; persistent pre_path_y_set_gcs; % 通过上个周期匹配点位矢判断遍历方向
persistent pre_path_heading_set_gcs; persistent pre_path_kappa_set_gcs;
persistent pre_x_set_gcs; persistent pre_y_set_gcs; % 判断相邻帧检测的是否是同一个障碍物

if isempty(is_first_run)
    is_first_run = 0;
    % 首次运行 所有待求点都从给定路径路径第一个点开始找 往前遍历 且距离连续增长上限设为50
    for i = 1:length(x_set_gcs) % 存在的点都在前面 遍历到nan至后面的点都不存在
        if isnan(x_set_gcs(i))
            break;
        end
        start_find_index = 1; forward_stable_backward = 1; increase_count_limit = 50;
        % 计算匹配点编号
        match_point_index = CalMatchPointIndex(x_set_gcs(i),y_set_gcs(i),path_x_set_gcs,...
        path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,start_find_index,...
        increase_count_limit, forward_stable_backward);
        % 计算投影点信息
        [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
        CalProjPointInfo(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
        path_kappa_set_gcs,match_point_index);
        % 将每个点求解信息添加到集合
        match_point_index_set(i) = match_point_index;
        proj_point_x_set_gcs(i) = proj_point_x_gcs; proj_point_y_set_gcs(i) = proj_point_y_gcs;
        proj_point_heading_set_gcs(i) = proj_point_heading_gcs; 
        proj_point_kappa_set_gcs(i) = proj_point_kappa_gcs;
    end
    % 保存该周期信息 以便下周期使用
    pre_match_point_index_set = match_point_index_set;
    pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
    pre_path_heading_set_gcs = path_heading_set_gcs; pre_path_kappa_set_gcs = path_kappa_set_gcs;
    pre_x_set_gcs = x_set_gcs; pre_y_set_gcs = y_set_gcs;
else
    % 需要使用上周期信息
    for i = 1:length(x_set_gcs)
        if isnan(x_set_gcs(i))
            break;
        end
        % 检测相邻帧是否是同一物体求解
        dis = sqrt((x_set_gcs(i)-pre_x_set_gcs(i))^2 + (y_set_gcs(i)-pre_y_set_gcs(i))^2);
        if dis > 10  % 相邻帧(相隔0.1s)同一物体距离不可能这么大
            % 说明相邻帧不是同一物体 上周期该物体已经消失在车辆的检测范围内
            % 把上周期该物体匹配点编号信息清空
            pre_match_point_index_set(i) = nan;
        end
        if isnan(pre_match_point_index_set(i))
            % 说明该物体是第一次寻找匹配点 从头开始寻找 且往前遍历
            start_find_index = 1; forward_stable_backward = 1; increase_count_limit = 50;
        else
            % 说明该物体上周期已经寻找过匹配点  判断开始寻找编号 和遍历方向
            start_find_index = pre_match_point_index_set(i); increase_count_limit = 5;
            % 提取上周期匹配点信息
            pre_match_point_x = pre_path_x_set_gcs(pre_match_point_index_set(i));
            pre_match_point_y = pre_path_y_set_gcs(pre_match_point_index_set(i));
            pre_match_point_heading = pre_path_heading_set_gcs(pre_match_point_index_set(i));
            % 通过该物体上周期匹配点位置到本周期位置向量和上周期匹配点切向量的点乘来判断遍历方向
            vector_preMatch_current = [x_set_gcs(i),y_set_gcs(i)] - [pre_match_point_x, pre_match_point_y];
            vector_tor_preMatch = [cos(pre_match_point_heading), sin(pre_match_point_heading)];
            forward_stable_backward = dot(vector_preMatch_current, vector_tor_preMatch);
        end
        % 计算匹配点编号
        match_point_index = CalMatchPointIndex(x_set_gcs(i),y_set_gcs(i),path_x_set_gcs,...
        path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,start_find_index,...
        increase_count_limit, forward_stable_backward);
        % 计算投影点信息
        [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
        CalProjPointInfo(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
        path_kappa_set_gcs,match_point_index);
        % 将每个点求解信息添加到集合
        match_point_index_set(i) = match_point_index;
        proj_point_x_set_gcs(i) = proj_point_x_gcs; proj_point_y_set_gcs(i) = proj_point_y_gcs;
        proj_point_heading_set_gcs(i) = proj_point_heading_gcs; 
        proj_point_kappa_set_gcs(i) = proj_point_kappa_gcs;
    end
    % 保存该周期信息 以便下周期使用
    pre_match_point_index_set = match_point_index_set;
    pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
    pre_path_heading_set_gcs = path_heading_set_gcs; pre_path_kappa_set_gcs = path_kappa_set_gcs;
    pre_x_set_gcs = x_set_gcs; pre_y_set_gcs = y_set_gcs;
end
end




