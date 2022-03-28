function [x_set_gcs,y_set_gcs,heading_set_gcs,kappa_set_gcs] = fcn(s_set_fcs,l_set_fcs,dl_set_fcs,ddl_set_fcs,...
    path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,matchPoint_index_s_set_fcs)
% �ú�����������(s,l) ת���� (x,y)
% s_set_fcs,l_set_fcs,dl_set_fcs,ddl_set_fcs ת���� ��Ȼ����ϵ��Ϣ
% path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs ����·����������Ϣ
% matchPoint_index_s_set_fcs ����·��������s���꼯��
% ���������181������Ҫת��
n = 181;
% �����ʼ��
x_set_gcs = ones(n,1)*nan; y_set_gcs = ones(n,1)*nan; heading_set_gcs = ones(n,1)*nan; kappa_set_gcs = ones(n,1)*nan;
for i = 1:length(s_set_fcs)
    if isnan(s_set_fcs(i))
        break;
    end
    % ����õ��ڸ���·���������� ͶӰ�����Ϣ
    [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
                CalProjPointInfoFromFrenet(s_set_fcs(i),path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
                                                path_kappa_set_gcs,matchPoint_index_s_set_fcs);
    x_set_gcs(i) = proj_point_x_gcs - l_set_fcs(i) * sin(proj_point_heading_gcs);
    y_set_gcs(i) = proj_point_y_gcs + l_set_fcs(i) * cos(proj_point_heading_gcs);
    heading_set_gcs(i) = proj_point_heading_gcs + atan(dl_set_fcs(i)/(1-proj_point_kappa_gcs*l_set_fcs(i)));
    k1 = ddl_set_fcs(i) + proj_point_kappa_gcs * dl_set_fcs(i) * tan(heading_set_gcs(i)-proj_point_heading_gcs);
    k2 = (cos(heading_set_gcs(i)-proj_point_heading_gcs))^2 / (1 - proj_point_kappa_gcs*l_set_fcs(i));
    k3 = cos(heading_set_gcs(i)-proj_point_heading_gcs) / (1 - proj_point_kappa_gcs*l_set_fcs(i));
    kappa_set_gcs(i) = (k1 * k2 + proj_point_kappa_gcs) * k3;
end
end