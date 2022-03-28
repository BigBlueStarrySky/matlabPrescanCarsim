function cost= CalNeighbourColPointCost(pre_col_node_s_fcs,pre_col_node_l_fcs,pre_col_node_dl_fcs,pre_col_node_ddl_fcs,...
    cur_col_node_s_fcs, cur_col_node_l_fcs,cur_col_node_dl_fcs,cur_col_node_ddl_fcs, w_obs,w_smooth_dl,w_smooth_ddl,...
    w_smooth_dddl,w_ref,obs_s_set_fcs,obs_l_set_fcs,sample_s)
% 该函数将用来计算相邻列点之间的路径代价 平滑代价 障碍物代价  参考线距离代价

% 计算连接两个点五次多项式的系数
[a0,a1,a2,a3,a4,a5] = CalQuinticCoeffient(pre_col_node_s_fcs,pre_col_node_l_fcs,pre_col_node_dl_fcs,...
pre_col_node_ddl_fcs,cur_col_node_s_fcs,cur_col_node_l_fcs,cur_col_node_dl_fcs,cur_col_node_ddl_fcs);

% 在曲线上等分n个点 并计算每个点的信息 起点为pre_col_node_s_fcs 终点为cur_col_node_s_fcs
n = 10; % 假设10
% 对等分点信息进行初始化
s = zeros(n,1); l = zeros(n,1); dl = zeros(n,1); ddl = zeros(n,1); dddl = zeros(n,1);

% 求解出等分点信息
for i = 1:n
    % 通过起点把每一个s求解出来
    s(i) = pre_col_node_s_fcs + (i-1) * sample_s / n;
    % 再通过五次多项式求解出l,dl,ddl
    % l = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
    % dl =      a1  + 2*a2*s + 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4
    % ddl =           2*a2 +  6*a3*s  +  12*a4*s^2  + 20*a5*s^3
    % dddl =                   6*a3   + 24*a4*s   + 60*a5*s^2 
    l(i) = a0 + a1*s(i) + a2*s(i)^2 + a3*s(i)^3 + a4*s(i)^4 + a5*s(i)^5;
    dl(i) =      a1  + 2*a2*s(i) + 3*a3*s(i)^2 + 4*a4*s(i)^3 + 5*a5*s(i)^4;
    ddl(i) =           2*a2 +  6*a3*s(i)  +  12*a4*s(i)^2  + 20*a5*s(i)^3;
    dddl(i) =                   6*a3   + 24*a4*s(i)   + 60*a5*s(i)^2;
end

% 计算总的平滑代价
% cost = cost_smooth + cost_ref + cost_obs
% 计算出每个等分点的代价 累计即为前列点到当前列点的代价
cost_smooth = 0;  cost_ref = 0; cost_obs = 0; % 将三种代价初始化为0
for i = 1:n
     % 单个等分点的平滑代价
    cost_smooth_once = w_smooth_dl*(dl(i)^2) + w_smooth_ddl*(ddl(i)^2) + w_smooth_dddl*(dddl(i)^2);
    cost_smooth = cost_smooth + cost_smooth_once;  % 累加得到总的平滑代价
    % 单个等分点的参考线代价
    cost_ref_once = w_ref * l(i)^2;
    cost_ref = cost_ref + cost_ref_once; % 累加得到总的参考线代价
    % 单个等分点到所有障碍物的代价
    for j = 1:length(obs_s_set_fcs)  % 遍历所有障碍物
        if isnan(obs_s_set_fcs(j))
            break;
        end
        % 计算当前等分点到某个障碍物的横向距离和纵向距离
        dlong = s(i) - obs_s_set_fcs(j);  % 当前等分点到某个障碍物的横向距离
        dlat = l(i) - obs_l_set_fcs(j);  % 当前等分点到某个障碍物的纵向距离
        % 将障碍物简化为一个质点  当前采样点离某个障碍物的距离的平方为   这里是约等于 使用直角坐标系的计算公式
        dis_square = dlong^2 + dlat^2;  % 可以更精细化
        % de离某个障碍物的距离超过5m时 cost为0 在4到5米时 cost为1000/dis_square 在小于3米时cost为w_obs
        if dis_square > 25
            cost_obs_once = 0;
        elseif (dis_square <= 25 && dis_square >= 16)
            cost_obs_once = 1000 / dis_square;
        else
            cost_obs_once = w_obs;
        end
        cost_obs = cost_obs + cost_obs_once; % 累加得到总的障碍物代价
    end
end
% 将三种代价累加得到总的代价
cost = cost_smooth + cost_ref + cost_obs;
end 