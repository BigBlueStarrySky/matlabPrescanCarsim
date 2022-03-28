function [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
    CalProjPointInfoFromFrenet(s_fcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,index_to_s_set)
% 该函数用来计算 自然坐标系的点(s,l)在路径坐标轴上投影点的直角坐标信息
% 设投影点 路径坐标轴上前一个点为pre_index 路径坐标轴上后一个点为behind_index
% 如果投影点直接就是坐标轴上离散的点 则直接取对应点信息即可
% 输入:
% index_to_s_set 以自车投影点作为s轴原点  待转换点在给定路径上匹配点下标与s轴坐标的对应关系
% s坐标  相当于是计算当前转换点投影点到s轴坐标原点的弧长
% 有两种情况 一种就是待转换点的s坐标刚好与index_to_s_set某个s轴坐标相等 说明此时投影点刚好与匹配点重合 
% 第二种待转换点的s坐标介于index_to_s_set介于某两个s轴坐标之间
% 输出初始化
proj_point_x_gcs = -1; proj_point_y_gcs = -1; 
proj_point_heading_gcs = -1; proj_point_kappa_gcs = -1;

if s_fcs >= index_to_s_set(end)
% 取最后一个匹配点的信息作为投影点信息即可
    proj_point_x_gcs = path_x_set_gcs(end); proj_point_y_gcs = path_y_set_gcs(end);
    proj_point_heading_gcs = path_heading_set_gcs(end); proj_point_kappa_gcs = path_kappa_set_gcs(end);
else	
    for i = 1:length(index_to_s_set)-1
        % 第一种情况
        if s_fcs == index_to_s_set(i)
            % 取匹配点的信息作为投影点信息即可
            proj_point_x_gcs = path_x_set_gcs(i); proj_point_y_gcs = path_y_set_gcs(i);
            proj_point_heading_gcs = path_heading_set_gcs(i); proj_point_kappa_gcs = path_kappa_set_gcs(i);
            break;
        % 第二种情况 取投影点坐标轴上前一个离散点和后一个离散点进行求解
        elseif s_fcs > index_to_s_set(i) && s_fcs < index_to_s_set(i+1)
            % 选取前一个离散点和后一个离散点heading角的平均值
            avg_heading = (path_heading_set_gcs(i) + path_heading_set_gcs(i+1)) / 2;
            ds = s_fcs - index_to_s_set(i); %待转换点投影点到前一个离散点的弧长 可以近似为直线距离
            proj_point_x_gcs = path_x_set_gcs(i) + ds * cos(avg_heading);
            proj_point_y_gcs = path_y_set_gcs(i) + ds * sin(avg_heading);
            % dtheta = km * ds
            proj_point_heading_gcs = path_heading_set_gcs(i) + path_kappa_set_gcs(i) * ds;
            % 选取前一个离散点和后一个离散点kappa的平均值作为待转换点投影点的kappa
            proj_point_kappa_gcs = (path_kappa_set_gcs(i) + path_kappa_set_gcs(i+1)) / 2;
            break;
        end
    end
end
end