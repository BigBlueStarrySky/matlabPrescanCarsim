function [dp_path_s_final_set_fcs,dp_path_l_final_set_fcs,dp_path_dl_final_set_fcs,dp_path_ddl_final_set_fcs] = ...
    fcn(plan_start_point_s_fcs,plan_start_point_l_fcs,plan_start_point_dl_fcs,plan_start_point_ddl_fcs,dp_path_s_init_set_fcs,dp_path_l_init_set_fcs)
% ԭ�еĶ�̬�滮��·����̫��  �ú�����������·����
% ����:plan_start_point_info_fcs �滮�����Ϣ dp_path_info_init_set_fcs ��̬�滮��ʼ�㼯����Ϣ
% ���:dp_path_info_final_set_fcs ��̬�滮���յ㼯����Ϣ
% ԭ�еĶ�̬�滮��·����̫��  �ú�����������ԭ�еĶ�̬�滮�õ��ĳ�ʼ·����
% ͨ��ԭ�еĹ滮���������
% ��������֮�����ÿ��һ�ײ�һ����
ds = 1;
% ԭ��·��Ϊ �滮���-> ��̬�滮��1 -> ��̬�滮��2 -> ��̬�滮��3 -> ��̬�滮��4 -> ��̬�滮��5
% ���ܺ�·��Ϊ �滮���->(������->������->������)->��̬�滮��1->(������->������->������)->��̬�滮��2->...
% �� �滮���-> ��̬�滮��1 Ϊһ����ζ���ʽ  ͨ��dsȡ����ʽ����� ��Ϊ�����ĵ�

% ��������֮�����200����
n = 200;
% �����ʼ��
dp_path_s_final_set_fcs = ones(n,1)*nan;dp_path_l_final_set_fcs = ones(n,1)*nan;
dp_path_dl_final_set_fcs = ones(n,1)*nan; dp_path_ddl_final_set_fcs = ones(n,1)*nan;

% ��һ����ζ���ʽ�����Ϊ�滮���
start_s = plan_start_point_s_fcs; start_l = plan_start_point_l_fcs; 
start_dl = plan_start_point_dl_fcs; start_ddl = plan_start_point_ddl_fcs;

s_cur = []; l_cur = []; dl_cur = []; ddl_cur = []; %���ٶ�̬����洢���в�����

for i = 1:length(dp_path_s_init_set_fcs)
    if isnan(dp_path_s_init_set_fcs(i))
        break;   %�㲻���� �˳�
    end
    s_temp = []; l_temp = []; dl_temp = []; ddl_temp = []; %������ʱ��̬����洢��ǰѭ��������
    % �Թ滮���-> ��̬�滮��1 -> ��̬�滮��2 -> ��̬�滮��3 -> ��̬�滮��4 -> ��̬�滮��5->...���в���
    for j = 1:10000  % ȡ���ϴ��ѭ�� ����ͨ������break�˳�ѭ����
        s_node = start_s + ds * (j-1);   %���ܺ����������s����  l,dl,ddl��Ҫͨ����ζ���ʽ����
        if s_node < dp_path_s_init_set_fcs(i)
            s_temp = [s_temp, s_node]; % ���������������ܵ���ӽ�����
        else
            break;
        end
    end
    % % �����ζ���ʽ�յ�ı߽�����
    end_s = dp_path_s_init_set_fcs(i); end_l = dp_path_l_init_set_fcs(i); end_dl = 0; end_ddl = 0;
    [a0,a1,a2,a3,a4,a5] = CalQuinticCoeffient(start_s,start_l,start_dl,start_ddl,end_s,end_l,end_dl,end_ddl);
    % ͨ����ζ���ʽ���ÿ���������l,dl,ddl
    for k = 1:length(s_temp)
        l_node = a0 + a1 * s_temp(k) + a2 * s_temp(k)^2+ a3 * s_temp(k)^3 + a4 * s_temp(k)^4 + a5 * s_temp(k)^5;
        l_temp = [l_temp,l_node];
        dl_node = a1 + 2 * a2 * s_temp(k) + 3 * a3 * s_temp(k)^2 + 4 * a4 * s_temp(k)^3 + 5 * a5 * s_temp(k)^4;
        dl_temp = [dl_temp,dl_node];
        ddl_node = 2 * a2 + 6 * a3 * s_temp(k) + 12 * a4 * s_temp(k)^2 + 20 * a5 * s_temp(k)^3;
        ddl_temp = [ddl_temp, ddl_node];
    end 
    % ����ǰѭ����¼�Ĳ�������Ϣ��¼���ܵĶ�̬������
    s_cur = [s_cur, s_temp]; l_cur = [l_cur, l_temp]; dl_cur = [dl_cur, dl_temp]; ddl_cur = [ddl_cur, ddl_temp];           
     % end��ֵ��ֵ��start ����һ��ѭ��
    start_s = end_s;start_l = end_l;start_dl = end_dl;start_ddl = end_ddl;
end

for i = 1:length(s_cur)
    % ��ֹ����֮���ĸ��������������
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