function [dp_path_s_final_set_fcs,dp_path_l_final_set_fcs,dp_path_dl_final_set_fcs,dp_path_ddl_final_set_fcs] = ...
    fcn(plan_start_point_s_fcs,plan_start_point_l_fcs,plan_start_point_dl_fcs,plan_start_point_ddl_fcs,dp_path_s_init_set_fcs,dp_path_l_init_set_fcs)
% 原有的动态规划的路径点太少  该函数用来增密路径点
% 输入:plan_start_point_info_fcs 规划起点信息 dp_path_info_init_set_fcs 动态规划初始点集合信息
% 输出:dp_path_info_final_set_fcs 动态规划最终点集合信息
% 原有的动态规划的路径点太少  该函数用来增密原有的动态规划得到的初始路径点
% 通过原有的规划点进行增密
% 假设增密之后横向每隔一米踩一个点
ds = 1;
% 原来路径为 规划起点-> 动态规划点1 -> 动态规划点2 -> 动态规划点3 -> 动态规划点4 -> 动态规划点5
% 增密后路径为 规划起点->(新增点->新增点->新增点)->动态规划点1->(新增点->新增点->新增点)->动态规划点2->...
% 如 规划起点-> 动态规划点1 为一个五次多项式  通过ds取多项式多个点 即为新增的点

% 假设增密之后最多200个点
n = 200;
% 输出初始化
dp_path_s_final_set_fcs = ones(n,1)*nan;dp_path_l_final_set_fcs = ones(n,1)*nan;
dp_path_dl_final_set_fcs = ones(n,1)*nan; dp_path_ddl_final_set_fcs = ones(n,1)*nan;

% 第一个五次多项式的起点为规划起点
start_s = plan_start_point_s_fcs; start_l = plan_start_point_l_fcs; 
start_dl = plan_start_point_dl_fcs; start_ddl = plan_start_point_ddl_fcs;

s_cur = []; l_cur = []; dl_cur = []; ddl_cur = []; %开辟动态数组存储所有采样点

for i = 1:length(dp_path_s_init_set_fcs)
    if isnan(dp_path_s_init_set_fcs(i))
        break;   %点不存在 退出
    end
    s_temp = []; l_temp = []; dl_temp = []; ddl_temp = []; %开辟临时动态数组存储当前循环采样点
    % 对规划起点-> 动态规划点1 -> 动态规划点2 -> 动态规划点3 -> 动态规划点4 -> 动态规划点5->...进行采样
    for j = 1:10000  % 取个较大的循环 下面通过条件break退出循环就
        s_node = start_s + ds * (j-1);   %增密后各个采样点s坐标  l,dl,ddl需要通过五次多项式计算
        if s_node < dp_path_s_init_set_fcs(i)
            s_temp = [s_temp, s_node]; % 将满足条件的增密点添加进数组
        else
            break;
        end
    end
    % % 求出五次多项式终点的边界条件
    end_s = dp_path_s_init_set_fcs(i); end_l = dp_path_l_init_set_fcs(i); end_dl = 0; end_ddl = 0;
    [a0,a1,a2,a3,a4,a5] = CalQuinticCoeffient(start_s,start_l,start_dl,start_ddl,end_s,end_l,end_dl,end_ddl);
    % 通过五次多项式求出每个采样点的l,dl,ddl
    for k = 1:length(s_temp)
        l_node = a0 + a1 * s_temp(k) + a2 * s_temp(k)^2+ a3 * s_temp(k)^3 + a4 * s_temp(k)^4 + a5 * s_temp(k)^5;
        l_temp = [l_temp,l_node];
        dl_node = a1 + 2 * a2 * s_temp(k) + 3 * a3 * s_temp(k)^2 + 4 * a4 * s_temp(k)^3 + 5 * a5 * s_temp(k)^4;
        dl_temp = [dl_temp,dl_node];
        ddl_node = 2 * a2 + 6 * a3 * s_temp(k) + 12 * a4 * s_temp(k)^2 + 20 * a5 * s_temp(k)^3;
        ddl_temp = [ddl_temp, ddl_node];
    end 
    % 将当前循环记录的采样点信息记录到总的动态数组里
    s_cur = [s_cur, s_temp]; l_cur = [l_cur, l_temp]; dl_cur = [dl_cur, dl_temp]; ddl_cur = [ddl_cur, ddl_temp];           
     % end的值赋值给start 做下一步循环
    start_s = end_s;start_l = end_l;start_dl = end_dl;start_ddl = end_ddl;
end

for i = 1:length(s_cur)
    % 防止增密之后点的个数超过最大容量
    if i > n
        break;
    end
    dp_path_s_final_set_fcs(i) = s_cur(i);
    dp_path_s_final_set_fcs(i) = plan_start_point_s_fcs + ds * (i - 1);
    dp_path_l_final_set_fcs(i) = l_cur(i);
    dp_path_dl_final_set_fcs(i) = dl_cur(i);
    dp_path_ddl_final_set_fcs(i) = ddl_cur(i);
end
end