function [sdot_set_fcs, ldot_set_fcs, dl_set_fcs] = fun(l_set_fcs,vx_set_gcs,vy_set_gcs,proj_point_heading_set_gcs,proj_point_kappa_set_gcs)
% 该函数批量转化坐标得到    sdot_set_fcs, ldot_set_fcs, dl_set_fcs
% 假设最多有181个点需要转换坐标
n = 181;
% 输出初始化
sdot_set_fcs = ones(n,1)*nan; ldot_set_fcs = ones(n,1)*nan; dl_set_fcs = ones(n,1)*nan;

for i = 1:length(vx_set_gcs)
    if isnan(vx_set_gcs(i))
        break;
    end
    vector_vh = [vx_set_gcs(i),vy_set_gcs(i)];
    vector_tp = [cos(proj_point_heading_set_gcs(i)),sin(proj_point_heading_set_gcs(i))];
    vector_np = [-sin(proj_point_heading_set_gcs(i)),cos(proj_point_heading_set_gcs(i))];
    sdot_set_fcs(i) = (dot(vector_vh,vector_tp)) / (1-proj_point_kappa_set_gcs(i)*l_set_fcs(i));
    ldot_set_fcs(i) = dot(vector_vh,vector_np);
    if abs(sdot_set_fcs(i)) < 1e-6
        dl_set_fcs(i) = 0;
    else
        dl_set_fcs(i) = ldot_set_fcs(i) / sdot_set_fcs(i);
    end
end
end