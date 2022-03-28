function [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
    CalProjPointInfo(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,match_point_index)
% �ú�����ͨ��������ڸ���·���������µ�ƥ����������Ӧ��ͶӰ����Ϣ
%����:x_gcs,y_gcs�������Ϣ
%path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs ����·����������Ϣ����
% match_point_index ������ڸ���·���������µ�ƥ�����
% ���:proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs ������ڸ���·����������ͶӰ����Ϣ

% ��ȡƥ�����Ϣ
match_point_x_gcs = path_x_set_gcs(match_point_index); match_point_y_gcs = path_y_set_gcs(match_point_index);
match_point_heading_gcs = path_heading_set_gcs(match_point_index); match_point_kappa_gcs = path_kappa_set_gcs(match_point_index);

vector_match_point = [x_gcs - match_point_x_gcs, y_gcs - match_point_y_gcs]; % ƥ��㵽����������
t_match = [cos(match_point_heading_gcs), sin(match_point_heading_gcs)]; % ƥ����ظ���·���ĵ�λ������

% ����ͶӰ����Ϣ
proj_point_x_gcs = match_point_x_gcs + dot(vector_match_point, t_match) * cos(match_point_heading_gcs);
proj_point_y_gcs = match_point_y_gcs + dot(vector_match_point, t_match) * sin(match_point_heading_gcs);
proj_point_heading_gcs = match_point_heading_gcs + match_point_kappa_gcs * dot(vector_match_point, t_match);
proj_point_kappa_gcs = match_point_kappa_gcs;
end