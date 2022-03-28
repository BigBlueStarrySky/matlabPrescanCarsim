function s = CalSFromOriginPoint(index_to_s_set,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
    proj_point_x_gcs,proj_point_y_gcs,match_point_index)
% 该函数将用来计算某个点投影点到给定路径坐标原点的弧长  亦即是求该点投影点s坐标
s_match_origin = index_to_s_set(match_point_index); % 该点匹配点到s轴坐标原点的弧长
% 匹配点单位切向量
match_point_tor = [cos(path_heading_set_gcs(match_point_index)),sin(path_heading_set_gcs(match_point_index))];
% 匹配点到投影点的向量
vector_match_proj = [proj_point_x_gcs - path_x_set_gcs(match_point_index),proj_point_y_gcs - path_y_set_gcs(match_point_index)]; 
% 计算匹配点到投影点的弧长  向量法计算出来的已经包括了前面或后面的结果
s_match_proj = dot(vector_match_proj, match_point_tor);  % 包括了正负两种情况
% 计算投影点到轨迹原点的弧长
s_proj_origin = s_match_origin + s_match_proj;
s = s_proj_origin;
end