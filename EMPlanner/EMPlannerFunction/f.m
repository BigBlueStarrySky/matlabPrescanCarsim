function [s_set_fcs,l_set_fcs] = fun(x_set_gcs,y_set_gcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,match_point_index_set,...
    proj_point_x_set_gcs,proj_point_y_set_gcs,proj_point_heading_set_gcs,matchPoint_index_s_set_fcs)
% �������������������(x,y)ת����(s,l)
% ���������181������Ҫת������
n = 181;
% �����ʼ��
s_set_fcs = ones(n,1)*nan; l_set_fcs = ones(n,1)*nan;
for i = 1:length(x_set_gcs)
    if isnan(x_set_gcs(i))
        break;
    end
    s_set_fcs(i) = CalSFromOriginPoint(matchPoint_index_s_set_fcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
                                proj_point_x_set_gcs(i),proj_point_y_set_gcs(i),match_point_index_set(i));
    vector_rh = [x_set_gcs(i),y_set_gcs(i)];
    vector_rp = [proj_point_x_gcs(i),proj_point_y_gcs(i)];
    vector_np = [-sin(proj_point_heading_set_gcs(i)),cos(proj_point_heading_set_gcs(i))];
    l_set_fcs(i) = dot(vector_rh-vector_rp, vector_np);
end
end
