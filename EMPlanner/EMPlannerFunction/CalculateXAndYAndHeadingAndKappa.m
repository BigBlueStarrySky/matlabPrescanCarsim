function point_info_set_gcs = CalculateXAndYAndHeadingAndKappa(point_info_set_fcs, path_info_set_gcs,index_to_s_set)
% �ú�������Ȼ����ϵ�µĵ���Ϣ(s,l,dl,ddl) ת��Ϊ��������ϵ�µĵ���Ϣ(x,y,heading,kappa)
% ����: point_info_set_fcs ��Ȼ����ϵ�µĵ���Ϣ(s,l,dl,ddl)
% path_info_set_gcs ����·������Ϣ  
% index_to_s_set ���Գ�ͶӰ����Ϊs��ԭ��  ��ת�����ڸ���·����ƥ����±���s������Ķ�Ӧ��ϵ
% ��� point_info_set_gcs ��������ϵ�µĵ���Ϣ(x,y,heading,kappa)
% ��ȡ��Ȼ����ϵ�µĵ���Ϣ
s_set_fcs = point_info_set_fcs(:,1); l_set_fcs = point_info_set_fcs(:,2);
dl_set_fcs = point_info_set_fcs(:,3); ddl_set_fcs = point_info_set_fcs(:,4);
% ��ȡ����·����Ϣ
path_x_set_gcs = path_info_set_gcs(:,1); path_y_set_gcs = path_info_set_gcs(:,2);
path_heading_set_gcs = path_info_set_gcs(:,3); path_kappa_set_gcs = path_info_set_gcs(:,4);
% ���ȼ���Ҫת���ĵ�ĸ���
n = length(s_set_fcs);
% ��������ϵ�µĵ���Ϣ��ʼ��
x_set_gcs = ones(n,1)*nan; y_set_gcs = ones(n,1)*nan; heading_set_gcs = ones(n,1)*nan; kappa_set_gcs = ones(n,1)*nan;
for i = 1:n
    if isnan(s_set_fcs(i))
        break;
    end
    % ���㵱ǰ��Ȼ����ϵ�µĵ�(s,l,dl,ddl)��·����������ͶӰ����Ϣ(ֱ������ϵ��Ϣ)
    [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
    CalculateProjPointInfoFromFrenet(s_set_fcs(i),path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,index_to_s_set);
    % ͨ��ͶӰ����Ϣ�����ת���ĵ��ھ�������ϵ�µĵ���Ϣ
    x_set_gcs(i) = proj_point_x_gcs - l_set_fcs(i) * sin(proj_point_heading_gcs);
    y_set_gcs(i) = proj_point_y_gcs + l_set_fcs(i) * cos(proj_point_heading_gcs);
    heading_set_gcs(i) = proj_point_heading_gcs + atan(dl_set_fcs(i)/(1-proj_point_kappa_gcs*l_set_fcs(i)));
    % ������Ϊ kappa' = 0
    k1 = ddl_set_fcs(i) + proj_point_kappa_gcs * dl_set_fcs(i) * tan(heading_set_gcs(i) - proj_point_heading_gcs);
    k2 = (cos(heading_set_gcs(i) - proj_point_heading_gcs))^2 / (1 - proj_point_kappa_gcs * l_set_fcs(i));
    k3 = cos(heading_set_gcs(i) - proj_point_heading_gcs) / (1 - proj_point_kappa_gcs * l_set_fcs(i));
    kappa_set_gcs(i) = (k1 * k2 + proj_point_kappa_gcs) * k3;  
end
% �������
point_info_set_gcs = [x_set_gcs,y_set_gcs,heading_set_gcs,kappa_set_gcs];
end