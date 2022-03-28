function point_info_set_gcs = CalculateXAndYAndHeadingAndKappa(point_info_set_fcs, path_info_set_gcs,index_to_s_set)
% 该函数将自然坐标系下的点信息(s,l,dl,ddl) 转化为绝对坐标系下的点信息(x,y,heading,kappa)
% 输入: point_info_set_fcs 自然坐标系下的点信息(s,l,dl,ddl)
% path_info_set_gcs 给定路径的信息  
% index_to_s_set 以自车投影点作为s轴原点  待转换点在给定路径上匹配点下标与s轴坐标的对应关系
% 输出 point_info_set_gcs 绝对坐标系下的点信息(x,y,heading,kappa)
% 读取自然坐标系下的点信息
s_set_fcs = point_info_set_fcs(:,1); l_set_fcs = point_info_set_fcs(:,2);
dl_set_fcs = point_info_set_fcs(:,3); ddl_set_fcs = point_info_set_fcs(:,4);
% 读取给定路径信息
path_x_set_gcs = path_info_set_gcs(:,1); path_y_set_gcs = path_info_set_gcs(:,2);
path_heading_set_gcs = path_info_set_gcs(:,3); path_kappa_set_gcs = path_info_set_gcs(:,4);
% 首先计算要转换的点的个数
n = length(s_set_fcs);
% 绝对坐标系下的点信息初始化
x_set_gcs = ones(n,1)*nan; y_set_gcs = ones(n,1)*nan; heading_set_gcs = ones(n,1)*nan; kappa_set_gcs = ones(n,1)*nan;
for i = 1:n
    if isnan(s_set_fcs(i))
        break;
    end
    % 计算当前自然坐标系下的点(s,l,dl,ddl)在路径坐标轴上投影点信息(直角坐标系信息)
    [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
    CalculateProjPointInfoFromFrenet(s_set_fcs(i),path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,index_to_s_set);
    % 通过投影点信息计算待转换的点在绝对坐标系下的点信息
    x_set_gcs(i) = proj_point_x_gcs - l_set_fcs(i) * sin(proj_point_heading_gcs);
    y_set_gcs(i) = proj_point_y_gcs + l_set_fcs(i) * cos(proj_point_heading_gcs);
    heading_set_gcs(i) = proj_point_heading_gcs + atan(dl_set_fcs(i)/(1-proj_point_kappa_gcs*l_set_fcs(i)));
    % 近似认为 kappa' = 0
    k1 = ddl_set_fcs(i) + proj_point_kappa_gcs * dl_set_fcs(i) * tan(heading_set_gcs(i) - proj_point_heading_gcs);
    k2 = (cos(heading_set_gcs(i) - proj_point_heading_gcs))^2 / (1 - proj_point_kappa_gcs * l_set_fcs(i));
    k3 = cos(heading_set_gcs(i) - proj_point_heading_gcs) / (1 - proj_point_kappa_gcs * l_set_fcs(i));
    kappa_set_gcs(i) = (k1 * k2 + proj_point_kappa_gcs) * k3;  
end
% 整理输出
point_info_set_gcs = [x_set_gcs,y_set_gcs,heading_set_gcs,kappa_set_gcs];
end