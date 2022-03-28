function [dpPath_s_init_set_fcs,dpPath_l_init_set_fcs] = fcn(obs_s_set_fcs,obs_l_set_fcs,plan_startPoint_s_fcs,...
    plan_startPoint_l_fcs,plan_startPoint_dl_fcs,plan_startPoint_ddl_fcs,...
         w_obs,w_smooth_dl,w_smooth_ddl,w_smooth_dddl,w_ref,row,col,sample_s,sample_l)
% �ú�����ͨ����̬�滮��һ����ʼ·����ʵ�ֱ���
% obs_s_set_fcs,obs_l_set_fcs �ϰ���������Ϣ
% plan_startPoint_s_fcs,plan_startPoint_l_fcs,plan_startPoint_dl_fcs,plan_startPoint_ddl_fcs �滮�����Ϣ
%  w_obs,w_smooth_dl,w_smooth_ddl,w_smooth_dddl,w_ref ��̬�滮Ȩ�ز���
% ��������� �� �м�� �м��  ��������Ϊ����  ���м��еĵ���Ҫ�պ���s����
mid_row = (row+1)/2;
% dpPath_s_init_set_fcs,dpPath_l_init_set_fcs  ��ʱ�������滮���

node_minCost_set = ones(row,col) * inf; % ������¼�Գ��ӹ滮��㵽ÿ�����������С����
preCol_bestRow_set = zeros(row, col); % ������¼����ò���������·��ǰһ�е��к�   ��һ�в�����ǰһ�������к���Ϊ0

% ���ȼ���滮��㵽��һ�����в�����Ĵ���
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

% �ټ���滮��㵽���������в��������С���� �Լ����Ź켣����
% һ��һ�е��������
for j = 2:col
    for i = 1:row
        %������ǰ��������
        cur_col = j;
        cur_col_node_s_fcs = plan_startPoint_s_fcs + cur_col * sample_s;
        cur_col_node_l_fcs = (mid_row - i) * sample_l;
        cur_col_node_dl_fcs = 0; cur_col_node_ddl_fcs = 0;
        % ���ǰһ�������в����㵽��ǰ�е�ǰ�в�����Ĵ���
        for k = 1:row
            % ����ǰһ��������
            pre_col = j - 1;
            pre_col_node_s_fcs = plan_startPoint_s_fcs + pre_col * sample_s;
            pre_col_node_l_fcs = (mid_row - k) * sample_l;
            pre_col_node_dl_fcs = 0; pre_col_node_ddl_fcs = 0;
            % ǰһ�е�ǰ�в����㵽��ǰ�е�ǰ�в�����Ĵ���
            cost = CalNeighbourColPointCost(pre_col_node_s_fcs,pre_col_node_l_fcs,pre_col_node_dl_fcs,pre_col_node_ddl_fcs,...
                cur_col_node_s_fcs, cur_col_node_l_fcs,cur_col_node_dl_fcs,cur_col_node_ddl_fcs, w_obs,w_smooth_dl,w_smooth_ddl,...
                w_smooth_dddl,w_ref,obs_s_set_fcs,obs_l_set_fcs,sample_s);
            cost = cost + node_minCost_set(k,pre_col);
            % ͨ���Ƚ� ���ս���С������������
            if cost < node_minCost_set(i,cur_col)
                node_minCost_set(i,cur_col) = cost;
                preCol_bestRow_set(i,cur_col) = k;
            end
        end
    end
end

% �������һ������·�������ĸ�  �༴������Ŀ���
% ����������node_minCost_set���һ�ж�Ӧ����
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
% �ټ��㵽�����һ������·���� ǰ�����������Ĺ켣����
for j = col:-1:2
    best_row = preCol_bestRow_set(best_row,j);
    best_row_set(j-1) = best_row;
end

% ��������·���к� �������·������������
for j = 1:col
    dpPath_s_init_set_fcs = plan_startPoint_s_fcs + j * sample_s;
    dpPath_l_init_set_fcs = (mid_row - best_row_set(j)) * sample_l;
end
end





