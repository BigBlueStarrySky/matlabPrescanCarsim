function [sdotdot_set_fcs, ldotdot_set_fcs, ddl_set_fcs] = fun(ax_set_gcs,ay_set_gcs,proj_point_heading_set_gcs,proj_point_kappa_set_gcs,...
    l_set_fcs, sdot_set_fcs,dl_set_fcs)
% �ú�������ת������õ�    sdotdot_set_fcs, ldotdot_set_fcs, ddl_set_fcs
% ���������181������Ҫת������
n = 181;
% �����ʼ��
sdotdot_set_fcs = ones(n,1)*nan; ldotdot_set_fcs = ones(n,1)*nan; ddl_set_fcs = ones(n,1)*nan;

for i = 1:length(ax_set_gcs)
    if isnan(ax_set_gcs(i))
        break;
    end
    vector_ah = [ax_set_gcs(i),ay_set_gcs(i)];
    vector_tp = [cos(proj_point_heading_set_gcs(i)),sin(proj_point_heading_set_gcs(i))];
    vector_np = [-sin(proj_point_heading_set_gcs(i)),cos(proj_point_heading_set_gcs(i))];
    sdotdot_set_fcs(i) = (dot(vector_ah,vector_tp) + 2*proj_point_kappa_set_gcs(i)*dl_set_fcs(i)*sdot_set_fcs(i)^2) / (1-proj_point_kappa_set_gcs(i)*l_set_fcs(i));
    ldotdot_set_fcs(i) = dot(vector_ah,vector_np) - proj_point_kappa_set_gcs(i) * (1-proj_point_kappa_set_gcs(i)*l_set_fcs(i)) * sdot_set_fcs(i)^2;
    if abs(sdotdot_set_fcs(i)) < 1e-6
        ddl_set_fcs(i) = 0;
    elsel
        ddl_set_fcs(i) = (ldotdot_set_fcs(i) - dl_set_fcs(i) * sdotdot_set_fcs(i)) / sdot_set_fcs(i)^2;
    end
end
end