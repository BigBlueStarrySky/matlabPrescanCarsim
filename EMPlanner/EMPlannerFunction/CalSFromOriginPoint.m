function s = CalSFromOriginPoint(index_to_s_set,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
    proj_point_x_gcs,proj_point_y_gcs,match_point_index)
% �ú�������������ĳ����ͶӰ�㵽����·������ԭ��Ļ���  �༴����õ�ͶӰ��s����
s_match_origin = index_to_s_set(match_point_index); % �õ�ƥ��㵽s������ԭ��Ļ���
% ƥ��㵥λ������
match_point_tor = [cos(path_heading_set_gcs(match_point_index)),sin(path_heading_set_gcs(match_point_index))];
% ƥ��㵽ͶӰ�������
vector_match_proj = [proj_point_x_gcs - path_x_set_gcs(match_point_index),proj_point_y_gcs - path_y_set_gcs(match_point_index)]; 
% ����ƥ��㵽ͶӰ��Ļ���  ����������������Ѿ�������ǰ������Ľ��
s_match_proj = dot(vector_match_proj, match_point_tor);  % �����������������
% ����ͶӰ�㵽�켣ԭ��Ļ���
s_proj_origin = s_match_origin + s_match_proj;
s = s_proj_origin;
end