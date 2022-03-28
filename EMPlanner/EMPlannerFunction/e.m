function matchPoint_index_s_set_fcs = fun(path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
    origin_match_point_index,origin_point_x_gcs,origin_point_y_gcs)
% �ú������Ը���·��������Ϊs�� ȡ�����ڸ���·����ͶӰ����Ϊ����ԭ�� �����ÿ���������źͶ�Ӧs����ļ���
% path_x_set_gcs,path_y_set_gcs ����·�������㼯��
% origin_match_point_index,origin_point_x_gcs,origin_point_y_gcs   ����ԭ����Ϣ
% �����ʼ��
matchPoint_index_s_set_fcs = zeros(length(path_x_set_gcs),1);
% ���ȸ���·��������ÿ���㵽��һ��������Ļ���
for i = 2:length(path_x_set_gcs)
    dis = sqrt((path_x_set_gcs(i)-path_x_set_gcs(i-1))^2+(path_y_set_gcs(i)-path_y_set_gcs(i-1))^2); % ��ֱ�ߴ������ڵ㻡��
    matchPoint_index_s_set_fcs(i) = dis + matchPoint_index_s_set_fcs(i-1); % �ۼӵõ�
end
%���������ԭ�㵽��һ��������Ļ���
s = CalSFromOriginPoint(matchPoint_index_s_set_fcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
                            origin_point_x_gcs,origin_point_y_gcs,origin_match_point_index);
% �õ�ÿ�������㵽����ԭ��Ļ�����Ӧ��ϵ �༴������s�����Ӧ��Ź�ϵ
matchPoint_index_s_set_fcs = matchPoint_index_s_set_fcs - ones(length(path_x_set_gcs),1)*s;
end