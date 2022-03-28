function [obs_x_set_final_gcs,obs_y_set_final_gcs,obs_velocity_set_final_gcs,obs_velocity_heading_set_final_gcs] = ...
    fcn(host_x_gcs,host_y_gcs,host_heading_gcs,obs_x_set_gcs,obs_y_set_gcs,obs_velocity_set_gcs,obs_velocity_heading_set_gcs)
% �ú�����ɸѡ�ϰ������[-10,60] ����[-10,10]���ϰ���Żᱻ����
% �ú���ֻ��һ�ֵ���������ʱ�취�����ǵ��೵���������ʹ�ϰ�������ԶҲӦ�ÿ���
% EM Planner��ȫ���Ƕ೵�����м���ģ�ÿ�����������ɲο���Ȼ���м���������켣����ѡ�����ŵĹ켣��Ϊ���

% ��������⵽181���ϰ���
n = 181;
% �����ʼ��
obs_x_set_final_gcs = ones(n,1)*nan; obs_y_set_final_gcs = ones(n,1)*nan;
obs_velocity_set_final_gcs = ones(n,1)*nan; obs_velocity_heading_set_final_gcs = ones(n,1)*nan;

count = 1; % ��¼����Ҫ���ϰ������
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