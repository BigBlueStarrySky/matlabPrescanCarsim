function [path_x_set_gcs,path_y_set_gcs] = fun(path_x_init_set_gcs, path_y_init_set_gcs,w_smooth,w_length,w_ref,...
    x_lb,x_ub,y_lb,y_ub)
% 该函数将通过二次规划方法平滑给定路径
% path_x_init_set_gcs, path_x_init_set_gcs 待平滑轨迹点信息
% w_smooth,w_length,w_ref 二次规划权重 平滑权重 紧凑权重(平滑之后尽可能长度短) 相似权重(与之前路径尽可能接近)
% x_lb,x_ub,y_lb,y_ub 平滑之后x,y坐标上下限

% 需要利用上周期平滑前轨迹信息和平滑后轨迹信息  因此需要定义静态变量来保存
% 如果本周期平滑前轨迹信息和上周期平滑前轨迹一模一样 就无需平滑
persistent is_first_run;
persistent pre_path_x_init_set_gcs; persistent pre_path_y_init_set_gcs; 
persistent pre_path_x_set_gcs; persistent pre_path_y_set_gcs; 

if isempty(is_first_run)
    is_first_run = 0;
    % 第一次 无需判断 需要平滑 调用二次规划平滑路径函数
    [path_x_set_gcs,path_y_set_gcs] = QpSmoothPath(path_x_init_set_gcs,path_y_init_set_gcs,...
    w_smooth,w_length,w_ref,x_lb,x_ub,y_lb,y_ub);
    % 保存该周期结果 供下周期使用
    pre_path_x_init_set_gcs = path_x_init_set_gcs; pre_path_y_init_set_gcs = path_y_init_set_gcs;
    pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
else
    % 判断本周期平滑前轨迹信息和上周期平滑前轨迹是否一模一样 判断起点就可以 因为采样点往后都是一样的
    if pre_path_x_init_set_gcs(1) == path_x_init_set_gcs(1) && pre_path_x_init_set_gcs(2) == path_x_init_set_gcs(2)
        % 无需平滑 直接采用上周期平滑结果即可
        path_x_set_gcs = pre_path_x_set_gcs; path_y_set_gcs = pre_path_y_set_gcs;
    else
        % 重新平滑
        [path_x_set_gcs,path_y_set_gcs] = QpSmoothPath(path_x_init_set_gcs,path_y_init_set_gcs,...
                                                        w_smooth,w_length,w_ref,x_lb,x_ub,y_lb,y_ub);
    end
     % 保存该周期结果 供下周期使用
     pre_path_x_init_set_gcs = path_x_init_set_gcs; pre_path_y_init_set_gcs = path_y_init_set_gcs;
     pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
end
end

