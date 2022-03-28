function cost= CalNeighbourColPointCost(pre_col_node_s_fcs,pre_col_node_l_fcs,pre_col_node_dl_fcs,pre_col_node_ddl_fcs,...
    cur_col_node_s_fcs, cur_col_node_l_fcs,cur_col_node_dl_fcs,cur_col_node_ddl_fcs, w_obs,w_smooth_dl,w_smooth_ddl,...
    w_smooth_dddl,w_ref,obs_s_set_fcs,obs_l_set_fcs,sample_s)
% �ú������������������е�֮���·������ ƽ������ �ϰ������  �ο��߾������

% ����������������ζ���ʽ��ϵ��
[a0,a1,a2,a3,a4,a5] = CalQuinticCoeffient(pre_col_node_s_fcs,pre_col_node_l_fcs,pre_col_node_dl_fcs,...
pre_col_node_ddl_fcs,cur_col_node_s_fcs,cur_col_node_l_fcs,cur_col_node_dl_fcs,cur_col_node_ddl_fcs);

% �������ϵȷ�n���� ������ÿ�������Ϣ ���Ϊpre_col_node_s_fcs �յ�Ϊcur_col_node_s_fcs
n = 10; % ����10
% �Եȷֵ���Ϣ���г�ʼ��
s = zeros(n,1); l = zeros(n,1); dl = zeros(n,1); ddl = zeros(n,1); dddl = zeros(n,1);

% �����ȷֵ���Ϣ
for i = 1:n
    % ͨ������ÿһ��s������
    s(i) = pre_col_node_s_fcs + (i-1) * sample_s / n;
    % ��ͨ����ζ���ʽ����l,dl,ddl
    % l = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
    % dl =      a1  + 2*a2*s + 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4
    % ddl =           2*a2 +  6*a3*s  +  12*a4*s^2  + 20*a5*s^3
    % dddl =                   6*a3   + 24*a4*s   + 60*a5*s^2 
    l(i) = a0 + a1*s(i) + a2*s(i)^2 + a3*s(i)^3 + a4*s(i)^4 + a5*s(i)^5;
    dl(i) =      a1  + 2*a2*s(i) + 3*a3*s(i)^2 + 4*a4*s(i)^3 + 5*a5*s(i)^4;
    ddl(i) =           2*a2 +  6*a3*s(i)  +  12*a4*s(i)^2  + 20*a5*s(i)^3;
    dddl(i) =                   6*a3   + 24*a4*s(i)   + 60*a5*s(i)^2;
end

% �����ܵ�ƽ������
% cost = cost_smooth + cost_ref + cost_obs
% �����ÿ���ȷֵ�Ĵ��� �ۼƼ�Ϊǰ�е㵽��ǰ�е�Ĵ���
cost_smooth = 0;  cost_ref = 0; cost_obs = 0; % �����ִ��۳�ʼ��Ϊ0
for i = 1:n
     % �����ȷֵ��ƽ������
    cost_smooth_once = w_smooth_dl*(dl(i)^2) + w_smooth_ddl*(ddl(i)^2) + w_smooth_dddl*(dddl(i)^2);
    cost_smooth = cost_smooth + cost_smooth_once;  % �ۼӵõ��ܵ�ƽ������
    % �����ȷֵ�Ĳο��ߴ���
    cost_ref_once = w_ref * l(i)^2;
    cost_ref = cost_ref + cost_ref_once; % �ۼӵõ��ܵĲο��ߴ���
    % �����ȷֵ㵽�����ϰ���Ĵ���
    for j = 1:length(obs_s_set_fcs)  % ���������ϰ���
        if isnan(obs_s_set_fcs(j))
            break;
        end
        % ���㵱ǰ�ȷֵ㵽ĳ���ϰ���ĺ��������������
        dlong = s(i) - obs_s_set_fcs(j);  % ��ǰ�ȷֵ㵽ĳ���ϰ���ĺ������
        dlat = l(i) - obs_l_set_fcs(j);  % ��ǰ�ȷֵ㵽ĳ���ϰ�����������
        % ���ϰ����Ϊһ���ʵ�  ��ǰ��������ĳ���ϰ���ľ����ƽ��Ϊ   ������Լ���� ʹ��ֱ������ϵ�ļ��㹫ʽ
        dis_square = dlong^2 + dlat^2;  % ���Ը���ϸ��
        % de��ĳ���ϰ���ľ��볬��5mʱ costΪ0 ��4��5��ʱ costΪ1000/dis_square ��С��3��ʱcostΪw_obs
        if dis_square > 25
            cost_obs_once = 0;
        elseif (dis_square <= 25 && dis_square >= 16)
            cost_obs_once = 1000 / dis_square;
        else
            cost_obs_once = w_obs;
        end
        cost_obs = cost_obs + cost_obs_once; % �ۼӵõ��ܵ��ϰ������
    end
end
% �����ִ����ۼӵõ��ܵĴ���
cost = cost_smooth + cost_ref + cost_obs;
end 