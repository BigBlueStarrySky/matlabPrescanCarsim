function [referenceline_x_init_set_gcs, referenceline_y_init_set_gcs] = fun(host_match_point_index, ...
    path_x_set_gcs, path_y_set_gcs)
% �ú������ڸ���·������ȡ��ʼ�ο���
%  host_match_point_index �Գ��ڸ���·����ƥ�����
%  path_x_set_gcs, path_y_set_gcs ����·��������Ϣ
%  referenceline_x_init_set_gcs, referenceline_y_init_set_gcs ��ʼ�ο�����Ϣ
% �ڸ���·���� ��ƥ�������ǰ������ȡ150����  �����˷���ȡ30����  �ܹ�181����  ����·��һ��϶������181����
% ��Ҫ�ж� ǰ�������Ƿ���150����  ���ߺ��˷����Ƿ���30����
n = 181;
% �����ʼ��
referenceline_x_init_set_gcs = ones(n,1)*nan; referenceline_y_init_set_gcs = ones(n,1)*nan;

if host_match_point_index <= 30
    % ���˷��򲻹�30����
    start_index = 1; % ����㿪ʼȡ
    end_index = 181;
elseif host_match_point_index + 150 > length(path_x_set_gcs)
    % ǰ�����򲻹�150����
    start_index = host_match_point_index - 30;
    end_index = length(path_x_set_gcs);
else
    % ǰ���ͺ��˷���������������Ҫ��
    start_index = host_match_point_index - 30;
    end_index = host_match_point_index + 150;
end

% �ڸ���·������ȡ181������Ϊ��ʼ�ο���
referenceline_x_init_set_gcs = path_x_set_gcs(start_index:end_index);
referenceline_y_init_set_gcs = path_y_set_gcs(start_index:end_index);
end