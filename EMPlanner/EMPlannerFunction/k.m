function [dpPath_s_init_set_fcs,dpPath_l_init_set_fcs] = fcn(obs_s_set_fcs,obs_l_set_fcs,plan_startPoint_s_fcs,...
    plan_startPoint_l_fcs,plan_startPoint_dl_fcs,plan_startPoint_ddl_fcs,...
         w_obs,w_smooth_dl,w_smooth_ddl,w_smooth_dddl,w_ref,row,col,sample_s,sample_l)
% 该函数将通过动态规划出一条初始路径来实现避障
% obs_s_set_fcs,obs_l_set_fcs 障碍物坐标信息
% plan_startPoint_s_fcs,plan_startPoint_l_fcs,plan_startPoint_dl_fcs,plan_startPoint_ddl_fcs 规划起点信息
%  w_obs,w_smooth_dl,w_smooth_ddl,w_smooth_dddl,w_ref 动态规划权重参数
% 采样点的行 列 行间距 列间距  行数必须为技术  且中间行的点需要刚好在s轴上
mid_row = (row+1)/2;
% dpPath_s_init_set_fcs,dpPath_l_init_set_fcs  暂时不包括规划起点

node_minCost_set = ones(row,col) * inf; % 用来记录自车从规划起点到每个采样点的最小代价
preCol_bestRow_set = zeros(row, col); % 用来记录到达该采样点最优路径前一列的行号   第一列采样点前一列最优行号视为0

% 首先计算规划起点到第一列所有采样点的代价
for i = 1:row
    pre_col_node_s_fcs = plan_startPoint_s_fcs; pre_col_node_l_fcs = plan_startPoint_l_fcs;
    pre_col_node_dl_fcs = plan_startPoint_dl_fcs; pre_col_node_ddl_fcs = plan_startPoint_ddl_fcs;
    cur_col_node_s_fcs = plan_startPoint_s_fcs + sample_s;
    cur_col_node_l_fcs = (mid_row - i) * sample_l;
    cur_col_node_dl_fcs = 0; cur_col_node_ddl_fcs = 0;
    node_minCost_set(i,1) = CalNeighbourColPointCost(pre_col_node_s_fcs,pre_col_node_l_fcs,pre_col_node_dl_fcs,pre_col_node_ddl_fcs,...
    cur_col_node_s_fcs, cur_col_node_l_fcs,cur_col_node_dl_fcs,cur_col_node_ddl_fcs, w_obs,w_smooth_dl,w_smooth_ddl,...
    w_smooth_dddl,w_ref,obs_s_set_fcs,obs_l_set_fcs,sample_s);
end

% 再计算规划起点到后续列所有采样点的最小代价 以及最优轨迹行数
% 一列一列的往后遍历
for j = 2:col
    for i = 1:row
        %遍历当前列所有行
        cur_col = j;
        cur_col_node_s_fcs = plan_startPoint_s_fcs + cur_col * sample_s;
        cur_col_node_l_fcs = (mid_row - i) * sample_l;
        cur_col_node_dl_fcs = 0; cur_col_node_ddl_fcs = 0;
        % 求解前一列所有行采样点到当前列当前行采样点的代价
        for k = 1:row
            % 遍历前一列所有行
            pre_col = j - 1;
            pre_col_node_s_fcs = plan_startPoint_s_fcs + pre_col * sample_s;
            pre_col_node_l_fcs = (mid_row - k) * sample_l;
            pre_col_node_dl_fcs = 0; pre_col_node_ddl_fcs = 0;
            % 前一列当前行采样点到当前列当前行采样点的代价
            cost = CalNeighbourColPointCost(pre_col_node_s_fcs,pre_col_node_l_fcs,pre_col_node_dl_fcs,pre_col_node_ddl_fcs,...
                cur_col_node_s_fcs, cur_col_node_l_fcs,cur_col_node_dl_fcs,cur_col_node_ddl_fcs, w_obs,w_smooth_dl,w_smooth_ddl,...
                w_smooth_dddl,w_ref,obs_s_set_fcs,obs_l_set_fcs,sample_s);
            cost = cost + node_minCost_set(k,pre_col);
            % 通过比较 最终将最小代价填充进数组
            if cost < node_minCost_set(i,cur_col)
                node_minCost_set(i,cur_col) = cost;
                preCol_bestRow_set(i,cur_col) = k;
            end
        end
    end
end

% 计算最后一列最优路径点是哪个  亦即是最终目标点
% 即计算数组node_minCost_set最后一列对应行数
lastCol_bestRow = 0;
minCost = inf;
for i = 1:row
    if node_minCost_set(i,end) < minCost
        lastCol_bestRow = i;
        minCost = node_minCost_set(i,end);
    end
end
best_row_set = zeros(col,1); best_row_set(end) = lastCol_bestRow;
best_row = lastCol_bestRow;
% 再计算到达最后一列最优路径点 前面所经历过的轨迹行数
for j = col:-1:2
    best_row = preCol_bestRow_set(best_row,j);
    best_row_set(j-1) = best_row;
end

% 根据最优路径行号 求解最优路径采样点坐标
for j = 1:col
    dpPath_s_init_set_fcs = plan_startPoint_s_fcs + j * sample_s;
    dpPath_l_init_set_fcs = (mid_row - best_row_set(j)) * sample_l;
end
end





