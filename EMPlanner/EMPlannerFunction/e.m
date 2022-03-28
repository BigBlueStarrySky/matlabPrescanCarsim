function matchPoint_index_s_set_fcs = fun(path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
    origin_match_point_index,origin_point_x_gcs,origin_point_y_gcs)
% 该函数将以给定路径采样点为s轴 取自身在给定路径上投影点作为坐标原点 来求出每个采样点编号和对应s坐标的集合
% path_x_set_gcs,path_y_set_gcs 给定路径采样点集合
% origin_match_point_index,origin_point_x_gcs,origin_point_y_gcs   坐标原点信息
% 输出初始化
matchPoint_index_s_set_fcs = zeros(length(path_x_set_gcs),1);
% 首先给定路径采样点每个点到第一个采样点的弧长
for i = 2:length(path_x_set_gcs)
    dis = sqrt((path_x_set_gcs(i)-path_x_set_gcs(i-1))^2+(path_y_set_gcs(i)-path_y_set_gcs(i-1))^2); % 以直线代替相邻点弧长
    matchPoint_index_s_set_fcs(i) = dis + matchPoint_index_s_set_fcs(i-1); % 累加得到
end
%再求出坐标原点到第一个采样点的弧长
s = CalSFromOriginPoint(matchPoint_index_s_set_fcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,...
                            origin_point_x_gcs,origin_point_y_gcs,origin_match_point_index);
% 得到每个采样点到坐标原点的弧长对应关系 亦即真正的s坐标对应编号关系
matchPoint_index_s_set_fcs = matchPoint_index_s_set_fcs - ones(length(path_x_set_gcs),1)*s;
end