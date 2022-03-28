function [obs_x_set_final_gcs,obs_y_set_final_gcs,obs_velocity_set_final_gcs,obs_velocity_heading_set_final_gcs] = ...
    fcn(host_x_gcs,host_y_gcs,host_heading_gcs,obs_x_set_gcs,obs_y_set_gcs,obs_velocity_set_gcs,obs_velocity_heading_set_gcs)
% 该函数将筛选障碍物，纵向[-10,60] 横向[-10,10]的障碍物才会被考虑
% 该函数只是一种单车道的临时办法，考虑到多车道情况，即使障碍物距离较远也应该考虑
% EM Planner完全体是多车道并行计算的，每个车道都生成参考线然后并行计算出多条轨迹，再选择最优的轨迹作为输出

% 假设最多检测到181个障碍物
n = 181;
% 输出初始化
obs_x_set_final_gcs = ones(n,1)*nan; obs_y_set_final_gcs = ones(n,1)*nan;
obs_velocity_set_final_gcs = ones(n,1)*nan; obs_velocity_heading_set_final_gcs = ones(n,1)*nan;

count = 1; % 记录符合要求障碍物个数
for i = 1:length(obs_x_set_gcs)
    if isnan(obs_x_set_gcs(i))
        break;
    end
    vector_host_obs = [host_x_gcs,host_y_gcs] - [obs_x_set_gcs(i),obs_y_set_gcs(i)];
    vector_th = [cos(host_heading_gcs),sin(host_heading_gcs)];
    vector_nh = [-sin(host_heading_gcs),cos(host_heading_gcs)];
    long_er = dot(vector_host_obs,vector_th);
    lat_er = dot(vector_host_obs,vector_nh);
    if long_er >= -10 && long_er <= 60 && abs(lat_er) <= 10
        obs_x_set_final_gcs(count) = obs_x_set_gcs(i);
        obs_y_set_final_gcs(count) = obs_y_set_gcs(i);
        obs_velocity_set_final_gcs(count) = obs_velocity_set_gcs(i);
        obs_velocity_heading_set_final_gcs(count) = obs_velocity_heading_set_gcs(i);
        count = count + 1;
    end
end
end