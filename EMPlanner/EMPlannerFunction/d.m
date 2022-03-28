function [plan_startPoint_x_gcs,plan_startPoint_y_gcs,plan_startPoint_heading_gcs,plan_startPoint_kappa_gcs, ...
    plan_startPoint_vx_gcs,plan_startPoint_vy_gcs,plan_startPoint_ax_gcs,plan_startPoint_ay_gcs,plan_startPoint_time,...
    stitchPath_x_set_gcs,stitchPath_y_set_gcs,stitchPath_heading_set_gcs,stitchPath_kappa_set_gcs,stitchPath_velocity_set_gcs,...
    stitchPath_acceleration_set_gcs,stitchPath_time_set] = fun(pre_planPath_x_set_gcs,pre_planPath_y_set_gcs,...
    pre_planPath_heading_set_gcs,pre_planPath_kappa_set_gcs,pre_planPath_velocity_set_gcs,pre_planPath_acceleration_set_gcs,...
    pre_planPath_time_set,current_time,host_x_gcs,host_y_gcs,host_heading_gcs,host_vx_vcs,host_vy_vcs,host_ax_vcs,host_ay_vcs,host_dheading_gcs)
% 该函数将计算规划起点和待拼接轨迹的信息
% plan_startPoint_x_gcs,plan_startPoint_y_gcs,plan_startPoint_heading_gcs,plan_startPoint_kappa_gcs,plan_startPoint_vx_gcs
% plan_startPoint_vy_gcs,plan_startPoint_ax_gcs,plan_startPoint_ay_gcs,plan_startPoint_time 规划起点信息
% stitchPath_x_set_gcs,stitchPath_y_set_gcs,stitchPath_heading_set_gcs,stitchPath_kappa_set_gcs,
% stitchPath_velocity_set_gcs,stitchPath_acceleration_set_gcs,stitchPath_time_set 待拼接轨迹信息
% pre_planPath_x_set_gcs,pre_planPath_y_set_gcs,pre_planPath_heading_set_gcs,pre_planPath_kappa_set_gcs,
% pre_planPath_velocity_set_gcs,pre_planPath_acceleration_set_gcs,pre_planPath_time_set 上个周期规划轨迹信息
% current_time 当前时间
% host_x_gcs,host_y_gcs,host_heading_gcs,host_vx_vcs,host_vy_vcs,host_ax_vcs,host_ay_vcs 自车当前时间信息

% 对输出进行初始化
% 假设取规划起点前面的20个点作为待拼接轨迹 对其初始化
n = 20;
stitchPath_x_set_gcs = zeros(n,1);stitchPath_y_set_gcs = zeros(n,1); stitchPath_heading_set_gcs = zeros(n,1);
stitchPath_kappa_set_gcs = zeros(n,1); stitchPath_velocity_set_gcs = zeros(n,1); stitchPath_acceleration_set_gcs = zeros(n,1);
stitchPath_time_set = ones(n,1) * -1; %时间为负代表没有对应的拼接轨迹
dt = 0.1; % 规划周期100ms 0.1s
persistent is_first_run;
% 得到车辆的x方向和y方向的速度和加速度
host_vx_gcs = host_vx_vcs * cos(host_heading_gcs) - host_vy_vcs * sin(host_heading_gcs);
host_vy_gcs = host_vx_vcs * sin(host_heading_gcs) + host_vy_vcs * cos(host_heading_gcs);
host_ax_gcs = host_ax_vcs * cos(host_heading_gcs) - host_ay_vcs * sin(host_heading_gcs);
host_ay_gcs = host_ax_vcs * sin(host_heading_gcs) + host_ay_vcs * cos(host_heading_gcs);
if isempty(is_first_run)
    is_first_run = 0;
    % 第一次运行 没有上周期轨迹  无需拼接  通过车辆动力学递推dt时间之后的信息作为规划起点 假设dt时间做匀加速运动
    plan_startPoint_x_gcs = host_x_gcs + host_vx_gcs * dt + 0.5 * host_ax_gcs * dt^2;
    plan_startPoint_y_gcs = host_y_gcs + host_vy_gcs * dt + 0.5 * host_ay_gcs * dt^2;
    plan_startPoint_heading_gcs = host_heading_gcs + host_dheading_gcs * dt;
    plan_startPoint_kappa_gcs = 0;
    plan_startPoint_vx_gcs = host_vx_gcs + host_ax_gcs * dt;
    plan_startPoint_vy_gcs = host_vy_gcs + host_ay_gcs * dt;
    plan_startPoint_ax_gcs = host_ax_gcs;
    plan_startPoint_ay_gcs = host_ay_gcs;
    plan_startPoint_time = current_time + dt;
else
    % 后续运行  有上周期轨迹 
    % 判断自身是否跟上上周期规划轨迹 如果没有跟上 无需拼接
    % 首先求出上周期自车应该在的位置
    for i = 1:length(pre_planPath_time_set)-1
        if current_time >= pre_planPath_time_set(i) && current_time < pre_planPath_time_set(i+1)
            break;
        end
    end
    % 当前时间current_time对应上周期规划轨迹i时刻位置
    % 提取上周期规划本车信息
    pre_host_x_gcs = pre_planPath_x_set_gcs(i); pre_host_y_gcs = pre_planPath_y_set_gcs(i);
    pre_host_heading_gcs = pre_planPath_heading_set_gcs(i);
    % 计算本周期本车信息和上周期规划本车信息的横向误差和纵向误差
    vector_preHost_curHost = [host_x_gcs,host_y_gcs] - [pre_host_x_gcs,pre_host_y_gcs];
    vector_tor_preHost = [cos(pre_host_heading_gcs),sin(pre_host_heading_gcs)];
    vector_nor_preHost = [-sin(pre_host_heading_gcs),cos(pre_host_heading_gcs)];
    lat_er = dot(vector_preMatch_current,vector_tor_preHost);
    long_er = dot(vector_preHost_curHost,vector_nor_preHost);
    if abs(lat_er) > 0.5 || abs(long_er) > 2.5
        % 如果横向误差大于0.5或纵向误差大于2.5 视为车辆控制没有跟上规划
        % 无需拼接轨迹  通过车辆动力学递推dt时间之后的信息作为规划起点 假设dt时间做匀加速运动
        plan_startPoint_x_gcs = host_x_gcs + host_vx_gcs * dt + 0.5 * host_ax_gcs * dt^2;
        plan_startPoint_y_gcs = host_y_gcs + host_vy_gcs * dt + 0.5 * host_ay_gcs * dt^2;
        plan_startPoint_heading_gcs = host_heading_gcs + host_dheading_gcs * dt;
        plan_startPoint_kappa_gcs = 0;
        plan_startPoint_vx_gcs = host_vx_gcs + host_ax_gcs * dt;
        plan_startPoint_vy_gcs = host_vy_gcs + host_ay_gcs * dt;
        plan_startPoint_ax_gcs = host_ax_gcs;
        plan_startPoint_ay_gcs = host_ay_gcs;
        plan_startPoint_time = current_time + dt;
    else
        % 车辆控制跟上规划 需要拼接轨迹
        % 首先计算current_time+dt时刻对应上周期规划轨迹j时刻位置 作为规划起点
        for j = i:length(pre_planPath_time_set)-1
            if current_time+dt >= pre_planPath_time_set(j) && current_time+dt < pre_planPath_time_set(j+1)
                break;
            end
        end
        % 提取上周期规划轨迹j时刻位置信息作为规划起点信息
        plan_startPoint_x_gcs = pre_planPath_x_set_gcs(j);
        plan_startPoint_y_gcs = pre_planPath_y_set_gcs(j);
        plan_startPoint_heading_gcs = pre_planPath_heading_set_gcs(j);
        plan_startPoint_kappa_gcs = pre_planPath_kappa_set_gcs(j);
        % 需要注意的是pre_planPath_velocity_set_gcs(j) 规划路径的速度为沿路径的切向速度 需要分解
        plan_startPoint_vx_gcs = pre_planPath_velocity_set_gcs(j) * cos(pre_planPath_heading_set_gcs(j));
        plan_startPoint_vy_gcs = pre_planPath_velocity_set_gcs(j) * sin(pre_planPath_heading_set_gcs(j));
        % 需要注意的是pre_planPath_acceleration_set_gcs(j) 规划路径的加速度为沿路径的切向加速度 实际上还有法向加速度 
        % 需要先求出法向加速度 再进行求解  an = v^2/R = v^2*k
        % 将加速度转化成向量 这样可以直接包括方向
        tor = [cos(pre_planPath_heading_set_gcs(j)),sin(pre_planPath_heading_set_gcs(j))];
        nor = [-sin(pre_planPath_heading_set_gcs(j)),cos(pre_planPath_heading_set_gcs(j))];
        a_tor = pre_planPath_acceleration_set_gcs(j) * tor;
        a_nor = pre_planPath_velocity_set_gcs(j)^2 * pre_planPath_kappa_set_gcs(j) * nor;
        % 通过向量进行分解 向量都可以看成是绝对x方向和绝对y方向组成 
        plan_startPoint_ax_gcs = a_tor(1) + a_nor(1);
        plan_startPoint_ay_gcs = a_tor(2) + a_nor(2);
        plan_startPoint_time = pre_planPath_time_set(j);
        % 选取规划起点前面的20个点作为待拼接轨迹
        if j > 20
            % 第一种情况 规划起点前面的点够20个
            start_index = j - 20;
            stitchPath_x_set_gcs(1:n) = pre_planPath_x_set_gcs(start_index:start_index+19);
            stitchPath_y_set_gcs(1:n) = pre_planPath_y_set_gcs(start_index:start_index+19);
            stitchPath_heading_set_gcs(1:n) = pre_planPath_heading_set_gcs(start_index:start_index+19);
            stitchPath_kappa_set_gcs(1:n) = pre_planPath_kappa_set_gcs(start_index:start_index+19);
            stitchPath_velocity_set_gcs(1:n) = pre_planPath_velocity_set_gcs(start_index:start_index+19);
            stitchPath_acceleration_set_gcs(1:n) = pre_planPath_acceleration_set_gcs(start_index:start_index+19);
            stitchPath_time_set(1:n) = pre_planPath_time_set(start_index:start_index+19);
        else
            % 第二种情况 规划起点前面的点不够20个
            start_index = 1;
            % 优先填充后面的点
            stitchPath_x_set_gcs(n-(j-2):n) = pre_planPath_x_set_gcs(start_index:j-1);
            stitchPath_y_set_gcs(n-(j-2):n) = pre_planPath_y_set_gcs(start_index:j-1);
            stitchPath_heading_set_gcs(n-(j-2):n) = pre_planPath_heading_set_gcs(start_index:j-1);
            stitchPath_kappa_set_gcs(n-(j-2):n) = pre_planPath_kappa_set_gcs(start_index:j-1);
            stitchPath_velocity_set_gcs(n-(j-2):n) = pre_planPath_velocity_set_gcs(start_index:j-1);
            stitchPath_acceleration_set_gcs(n-(j-2):n) = pre_planPath_acceleration_set_gcs(start_index:j-1);
            stitchPath_time_set(n-(j-2):n) = pre_planPath_time_set(start_index:j-1);
        end
    end
end
end



