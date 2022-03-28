function [path_length,path_s_set_fcs] = fcn(path_x_set_gcs,path_y_set_gcs)
% 该函数将用来计算给定路径的整个弧长 以及给定路径每个采样点到第一个采样点的弧长
n = length(path_x_set_gcs);
path_s_set_fcs = zeros(n,1);
flag = 1;
for i = 2:length(path_x_set_gcs)
    if isnan(path_x_set_gcs(i))
        flag = 0;
        break;
    end
    dis = sqrt((path_x_set_gcs(i)-path_x_set_gcs(i-1))^2 + (path_y_set_gcs(i)-path_y_set_gcs(i-1))^2);
    path_s_set_fcs(i) = path_s_set_fcs(i-1) + dis;
end

% 计算整个弧长 需要判断是遍历完推出 还是中途遇到不存在的点退出
if flag == 1
    % 正常退出
    path_length = path_s_set_fcs(end);
else
    path_length = path_s_set_fcs(i-1);
end
end

function [plan_startPoint_sdot,plan_startPoint_sdotdot] = fcn(plan_startPoint_vx_gcs,plan_startPoint_vy_gcs,...
    plan_startPoint_ax_gcs,plan_startstartPoint_ay_gcs,plan_startPoint_heading_gcs)
% 该函数计算速度规划的初始条件
vector_tStart = [cos(plan_start_heading_gcs),sin(plan_start_heading_gcs)];
vector_vStart = [plan_startPoint_vx_gcs,plan_startPoint_vy_gcs];
vector_aStart = [plan_startPoint_ax_gcs,plan_startPoint_ay_gcs];
v_t = tor'*[plan_start_vx;plan_start_vy];
a_t = tor'*[plan_start_ax;plan_start_ay];
plan_start_s_dot = v_t;
plan_start_s_dot2 = a_t;
