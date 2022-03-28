function [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
    CalProjPointInfo(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,match_point_index)
% 该函数将通过待求点在给定路径采样点下的匹配点编号求解相应的投影点信息
%输入:x_gcs,y_gcs待求点信息
%path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs 给定路径采样点信息集合
% match_point_index 待求点在给定路径采样点下的匹配点编号
% 输出:proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs 待求点在给定路径采样点下投影点信息

% 读取匹配点信息
match_point_x_gcs = path_x_set_gcs(match_point_index); match_point_y_gcs = path_y_set_gcs(match_point_index);
match_point_heading_gcs = path_heading_set_gcs(match_point_index); match_point_kappa_gcs = path_kappa_set_gcs(match_point_index);

vector_match_point = [x_gcs - match_point_x_gcs, y_gcs - match_point_y_gcs]; % 匹配点到待求点的向量
t_match = [cos(match_point_heading_gcs), sin(match_point_heading_gcs)]; % 匹配点沿给定路径的单位切向量

% 计算投影点信息
proj_point_x_gcs = match_point_x_gcs + dot(vector_match_point, t_match) * cos(match_point_heading_gcs);
proj_point_y_gcs = match_point_y_gcs + dot(vector_match_point, t_match) * sin(match_point_heading_gcs);
proj_point_heading_gcs = match_point_heading_gcs + match_point_kappa_gcs * dot(vector_match_point, t_match);
proj_point_kappa_gcs = match_point_kappa_gcs;
end