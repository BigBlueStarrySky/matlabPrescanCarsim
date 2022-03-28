function [plan_startPoint_x_gcs,plan_startPoint_y_gcs,plan_startPoint_heading_gcs,plan_startPoint_kappa_gcs, ...
    plan_startPoint_vx_gcs,plan_startPoint_vy_gcs,plan_startPoint_ax_gcs,plan_startPoint_ay_gcs,plan_startPoint_time,...
    stitchPath_x_set_gcs,stitchPath_y_set_gcs,stitchPath_heading_set_gcs,stitchPath_kappa_set_gcs,stitchPath_velocity_set_gcs,...
    stitchPath_acceleration_set_gcs,stitchPath_time_set] = fun(pre_planPath_x_set_gcs,pre_planPath_y_set_gcs,...
    pre_planPath_heading_set_gcs,pre_planPath_kappa_set_gcs,pre_planPath_velocity_set_gcs,pre_planPath_acceleration_set_gcs,...
    pre_planPath_time_set,current_time,host_x_gcs,host_y_gcs,host_heading_gcs,host_vx_vcs,host_vy_vcs,host_ax_vcs,host_ay_vcs,host_dheading_gcs)
% �ú���������滮���ʹ�ƴ�ӹ켣����Ϣ
% plan_startPoint_x_gcs,plan_startPoint_y_gcs,plan_startPoint_heading_gcs,plan_startPoint_kappa_gcs,plan_startPoint_vx_gcs
% plan_startPoint_vy_gcs,plan_startPoint_ax_gcs,plan_startPoint_ay_gcs,plan_startPoint_time �滮�����Ϣ
% stitchPath_x_set_gcs,stitchPath_y_set_gcs,stitchPath_heading_set_gcs,stitchPath_kappa_set_gcs,
% stitchPath_velocity_set_gcs,stitchPath_acceleration_set_gcs,stitchPath_time_set ��ƴ�ӹ켣��Ϣ
% pre_planPath_x_set_gcs,pre_planPath_y_set_gcs,pre_planPath_heading_set_gcs,pre_planPath_kappa_set_gcs,
% pre_planPath_velocity_set_gcs,pre_planPath_acceleration_set_gcs,pre_planPath_time_set �ϸ����ڹ滮�켣��Ϣ
% current_time ��ǰʱ��
% host_x_gcs,host_y_gcs,host_heading_gcs,host_vx_vcs,host_vy_vcs,host_ax_vcs,host_ay_vcs �Գ���ǰʱ����Ϣ

% ��������г�ʼ��
% ����ȡ�滮���ǰ���20������Ϊ��ƴ�ӹ켣 �����ʼ��
n = 20;
stitchPath_x_set_gcs = zeros(n,1);stitchPath_y_set_gcs = zeros(n,1); stitchPath_heading_set_gcs = zeros(n,1);
stitchPath_kappa_set_gcs = zeros(n,1); stitchPath_velocity_set_gcs = zeros(n,1); stitchPath_acceleration_set_gcs = zeros(n,1);
stitchPath_time_set = ones(n,1) * -1; %ʱ��Ϊ������û�ж�Ӧ��ƴ�ӹ켣
dt = 0.1; % �滮����100ms 0.1s
persistent is_first_run;
% �õ�������x�����y������ٶȺͼ��ٶ�
host_vx_gcs = host_vx_vcs * cos(host_heading_gcs) - host_vy_vcs * sin(host_heading_gcs);
host_vy_gcs = host_vx_vcs * sin(host_heading_gcs) + host_vy_vcs * cos(host_heading_gcs);
host_ax_gcs = host_ax_vcs * cos(host_heading_gcs) - host_ay_vcs * sin(host_heading_gcs);
host_ay_gcs = host_ax_vcs * sin(host_heading_gcs) + host_ay_vcs * cos(host_heading_gcs);
if isempty(is_first_run)
    is_first_run = 0;
    % ��һ������ û�������ڹ켣  ����ƴ��  ͨ����������ѧ����dtʱ��֮�����Ϣ��Ϊ�滮��� ����dtʱ�����ȼ����˶�
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
    % ��������  �������ڹ켣 
    % �ж������Ƿ���������ڹ滮�켣 ���û�и��� ����ƴ��
    % ��������������Գ�Ӧ���ڵ�λ��
    for i = 1:length(pre_planPath_time_set)-1
        if current_time >= pre_planPath_time_set(i) && current_time < pre_planPath_time_set(i+1)
            break;
        end
    end
    % ��ǰʱ��current_time��Ӧ�����ڹ滮�켣iʱ��λ��
    % ��ȡ�����ڹ滮������Ϣ
    pre_host_x_gcs = pre_planPath_x_set_gcs(i); pre_host_y_gcs = pre_planPath_y_set_gcs(i);
    pre_host_heading_gcs = pre_planPath_heading_set_gcs(i);
    % ���㱾���ڱ�����Ϣ�������ڹ滮������Ϣ�ĺ��������������
    vector_preHost_curHost = [host_x_gcs,host_y_gcs] - [pre_host_x_gcs,pre_host_y_gcs];
    vector_tor_preHost = [cos(pre_host_heading_gcs),sin(pre_host_heading_gcs)];
    vector_nor_preHost = [-sin(pre_host_heading_gcs),cos(pre_host_heading_gcs)];
    lat_er = dot(vector_preMatch_current,vector_tor_preHost);
    long_er = dot(vector_preHost_curHost,vector_nor_preHost);
    if abs(lat_er) > 0.5 || abs(long_er) > 2.5
        % �������������0.5������������2.5 ��Ϊ��������û�и��Ϲ滮
        % ����ƴ�ӹ켣  ͨ����������ѧ����dtʱ��֮�����Ϣ��Ϊ�滮��� ����dtʱ�����ȼ����˶�
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
        % �������Ƹ��Ϲ滮 ��Ҫƴ�ӹ켣
        % ���ȼ���current_time+dtʱ�̶�Ӧ�����ڹ滮�켣jʱ��λ�� ��Ϊ�滮���
        for j = i:length(pre_planPath_time_set)-1
            if current_time+dt >= pre_planPath_time_set(j) && current_time+dt < pre_planPath_time_set(j+1)
                break;
            end
        end
        % ��ȡ�����ڹ滮�켣jʱ��λ����Ϣ��Ϊ�滮�����Ϣ
        plan_startPoint_x_gcs = pre_planPath_x_set_gcs(j);
        plan_startPoint_y_gcs = pre_planPath_y_set_gcs(j);
        plan_startPoint_heading_gcs = pre_planPath_heading_set_gcs(j);
        plan_startPoint_kappa_gcs = pre_planPath_kappa_set_gcs(j);
        % ��Ҫע�����pre_planPath_velocity_set_gcs(j) �滮·�����ٶ�Ϊ��·���������ٶ� ��Ҫ�ֽ�
        plan_startPoint_vx_gcs = pre_planPath_velocity_set_gcs(j) * cos(pre_planPath_heading_set_gcs(j));
        plan_startPoint_vy_gcs = pre_planPath_velocity_set_gcs(j) * sin(pre_planPath_heading_set_gcs(j));
        % ��Ҫע�����pre_planPath_acceleration_set_gcs(j) �滮·���ļ��ٶ�Ϊ��·����������ٶ� ʵ���ϻ��з�����ٶ� 
        % ��Ҫ�����������ٶ� �ٽ������  an = v^2/R = v^2*k
        % �����ٶ�ת�������� ��������ֱ�Ӱ�������
        tor = [cos(pre_planPath_heading_set_gcs(j)),sin(pre_planPath_heading_set_gcs(j))];
        nor = [-sin(pre_planPath_heading_set_gcs(j)),cos(pre_planPath_heading_set_gcs(j))];
        a_tor = pre_planPath_acceleration_set_gcs(j) * tor;
        a_nor = pre_planPath_velocity_set_gcs(j)^2 * pre_planPath_kappa_set_gcs(j) * nor;
        % ͨ���������зֽ� ���������Կ����Ǿ���x����;���y������� 
        plan_startPoint_ax_gcs = a_tor(1) + a_nor(1);
        plan_startPoint_ay_gcs = a_tor(2) + a_nor(2);
        plan_startPoint_time = pre_planPath_time_set(j);
        % ѡȡ�滮���ǰ���20������Ϊ��ƴ�ӹ켣
        if j > 20
            % ��һ����� �滮���ǰ��ĵ㹻20��
            start_index = j - 20;
            stitchPath_x_set_gcs(1:n) = pre_planPath_x_set_gcs(start_index:start_index+19);
            stitchPath_y_set_gcs(1:n) = pre_planPath_y_set_gcs(start_index:start_index+19);
            stitchPath_heading_set_gcs(1:n) = pre_planPath_heading_set_gcs(start_index:start_index+19);
            stitchPath_kappa_set_gcs(1:n) = pre_planPath_kappa_set_gcs(start_index:start_index+19);
            stitchPath_velocity_set_gcs(1:n) = pre_planPath_velocity_set_gcs(start_index:start_index+19);
            stitchPath_acceleration_set_gcs(1:n) = pre_planPath_acceleration_set_gcs(start_index:start_index+19);
            stitchPath_time_set(1:n) = pre_planPath_time_set(start_index:start_index+19);
        else
            % �ڶ������ �滮���ǰ��ĵ㲻��20��
            start_index = 1;
            % ����������ĵ�
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



