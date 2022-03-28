function [match_point_index_set,proj_point_x_set_gcs,proj_point_y_set_gcs,proj_point_heading_set_gcs,...
    proj_point_kappa_set_gcs] = fcn(x_set_gcs,y_set_gcs,path_x_set_gcs,path_y_set_gcs,...
    path_heading_set_gcs,path_kappa_set_gcs)
% �ú���������������ڸ���·���µ�ƥ����ź�ͶӰ����Ϣ   
% x_set_gcs,y_set_gcs �������Ϣ����
% path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs ����·������Ϣ����
% match_point_index_set ƥ����ż���
% proj_point_x_set_gcs,proj_point_y_set_gcs,proj_point_heading_set_gcs,proj_point_kappa_set_gcs ͶӰ����Ϣ����

% ���������181������ҪͶӰ
n = 181;
% ��������г�ʼ��
match_point_index_set = ones(n,1) *nan; 
proj_point_x_set_gcs = ones(n,1) *nan;  proj_point_y_set_gcs = ones(n,1) *nan;
proj_point_heading_set_gcs = ones(n,1) *nan;  proj_point_kappa_set_gcs = ones(n,1) *nan;

% Ѱ��ƥ���ʱ ��Ҫ������һ�����ڵĽ�� ��˶�����Ҫ���õľ�̬����
persistent is_first_run;
persistent pre_match_point_index_set;
persistent pre_path_x_set_gcs; persistent pre_path_y_set_gcs; % ͨ���ϸ�����ƥ���λʸ�жϱ�������
persistent pre_path_heading_set_gcs; persistent pre_path_kappa_set_gcs;
persistent pre_x_set_gcs; persistent pre_y_set_gcs; % �ж�����֡�����Ƿ���ͬһ���ϰ���

if isempty(is_first_run)
    is_first_run = 0;
    % �״����� ���д���㶼�Ӹ���·��·����һ���㿪ʼ�� ��ǰ���� �Ҿ�����������������Ϊ50
    for i = 1:length(x_set_gcs) % ���ڵĵ㶼��ǰ�� ������nan������ĵ㶼������
        if isnan(x_set_gcs(i))
            break;
        end
        start_find_index = 1; forward_stable_backward = 1; increase_count_limit = 50;
        % ����ƥ�����
        match_point_index = CalMatchPointIndex(x_set_gcs(i),y_set_gcs(i),path_x_set_gcs,...
        path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,start_find_index,...
        increase_count_limit, forward_stable_backward);
        % ����ͶӰ����Ϣ
        [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
        CalProjPointInfo(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
        path_kappa_set_gcs,match_point_index);
        % ��ÿ���������Ϣ��ӵ�����
        match_point_index_set(i) = match_point_index;
        proj_point_x_set_gcs(i) = proj_point_x_gcs; proj_point_y_set_gcs(i) = proj_point_y_gcs;
        proj_point_heading_set_gcs(i) = proj_point_heading_gcs; 
        proj_point_kappa_set_gcs(i) = proj_point_kappa_gcs;
    end
    % �����������Ϣ �Ա�������ʹ��
    pre_match_point_index_set = match_point_index_set;
    pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
    pre_path_heading_set_gcs = path_heading_set_gcs; pre_path_kappa_set_gcs = path_kappa_set_gcs;
    pre_x_set_gcs = x_set_gcs; pre_y_set_gcs = y_set_gcs;
else
    % ��Ҫʹ����������Ϣ
    for i = 1:length(x_set_gcs)
        if isnan(x_set_gcs(i))
            break;
        end
        % �������֡�Ƿ���ͬһ�������
        dis = sqrt((x_set_gcs(i)-pre_x_set_gcs(i))^2 + (y_set_gcs(i)-pre_y_set_gcs(i))^2);
        if dis > 10  % ����֡(���0.1s)ͬһ������벻������ô��
            % ˵������֡����ͬһ���� �����ڸ������Ѿ���ʧ�ڳ����ļ�ⷶΧ��
            % �������ڸ�����ƥ�������Ϣ���
            pre_match_point_index_set(i) = nan;
        end
        if isnan(pre_match_point_index_set(i))
            % ˵���������ǵ�һ��Ѱ��ƥ��� ��ͷ��ʼѰ�� ����ǰ����
            start_find_index = 1; forward_stable_backward = 1; increase_count_limit = 50;
        else
            % ˵���������������Ѿ�Ѱ�ҹ�ƥ���  �жϿ�ʼѰ�ұ�� �ͱ�������
            start_find_index = pre_match_point_index_set(i); increase_count_limit = 5;
            % ��ȡ������ƥ�����Ϣ
            pre_match_point_x = pre_path_x_set_gcs(pre_match_point_index_set(i));
            pre_match_point_y = pre_path_y_set_gcs(pre_match_point_index_set(i));
            pre_match_point_heading = pre_path_heading_set_gcs(pre_match_point_index_set(i));
            % ͨ��������������ƥ���λ�õ�������λ��������������ƥ����������ĵ�����жϱ�������
            vector_preMatch_current = [x_set_gcs(i),y_set_gcs(i)] - [pre_match_point_x, pre_match_point_y];
            vector_tor_preMatch = [cos(pre_match_point_heading), sin(pre_match_point_heading)];
            forward_stable_backward = dot(vector_preMatch_current, vector_tor_preMatch);
        end
        % ����ƥ�����
        match_point_index = CalMatchPointIndex(x_set_gcs(i),y_set_gcs(i),path_x_set_gcs,...
        path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,start_find_index,...
        increase_count_limit, forward_stable_backward);
        % ����ͶӰ����Ϣ
        [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
        CalProjPointInfo(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
        path_kappa_set_gcs,match_point_index);
        % ��ÿ���������Ϣ��ӵ�����
        match_point_index_set(i) = match_point_index;
        proj_point_x_set_gcs(i) = proj_point_x_gcs; proj_point_y_set_gcs(i) = proj_point_y_gcs;
        proj_point_heading_set_gcs(i) = proj_point_heading_gcs; 
        proj_point_kappa_set_gcs(i) = proj_point_kappa_gcs;
    end
    % �����������Ϣ �Ա�������ʹ��
    pre_match_point_index_set = match_point_index_set;
    pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
    pre_path_heading_set_gcs = path_heading_set_gcs; pre_path_kappa_set_gcs = path_kappa_set_gcs;
    pre_x_set_gcs = x_set_gcs; pre_y_set_gcs = y_set_gcs;
end
end




