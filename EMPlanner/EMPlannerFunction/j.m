function [static_obs_x_set_gcs,static_obs_y_set_gcs,dynamic_obs_x_set_gcs,dynamic_obs_y_set_gcs,...
    dynamic_obs_vx_set_gcs,dynamic_obs_vy_set_gcs] = fcn(obs_x_set_gcs,obs_y_set_gcs,obs_velocity_set_gcs,obs_velocity_heading_set_gcs)

% 该函数将分类静态障碍物和动态障碍物
% 输出初始化
n = 181;
static_obs_x_set_gcs = ones(n,1)*nan; static_obs_y_set_gcs = ones(n,1)*nan;
dynamic_obs_x_set_gcs = ones(n,1)*nan; dynamic_obs_y_set_gcs = ones(n,1)*nan;
dynamic_obs_vx_set_gcs = ones(n,1)*nan; dynamic_obs_vy_set_gcs = ones(n,1)*nan;
count_static = 1;
count_dynamic = 1;

for i = 1:length(obs_x_set_gcs)
    if isnan(obs_x_set_gcs(i))
        break;
    end
    if abs(obs_velocity_set_gcs(i)) < 0.1
        % 速度小于0.1m/s 视为静态障碍物
        static_obs_x_set_gcs(count_static) = obs_x_set_gcs(i);
        static_obs_y_set_gcs(count_static) = obs_y_set_gcs(i);
        count_static = count_static + 1;
    else
        dynamic_obs_x_set_gcs(count_dynamic) = obs_x_set_gcs(i);
        dynamic_obs_y_set_gcs(count_dynamic) = obs_y_set_gcs(i);
        dynamic_obs_vx_set_gcs(count_dynamic) = obs_velocity_set_gcs(i) * cos(obs_velocity_heading_set_gcs(i));
        dynamic_obs_vy_set_gcs(count_dynamic) = obs_velocity_set_gcs(i) * sin(obs_velocity_heading_set_gcs(i));
        count_dynamic = count_dynamic + 1;
    end
end
end