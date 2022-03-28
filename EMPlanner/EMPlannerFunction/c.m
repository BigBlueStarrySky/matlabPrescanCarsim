function [path_x_set_gcs,path_y_set_gcs] = fun(path_x_init_set_gcs, path_y_init_set_gcs,w_smooth,w_length,w_ref,...
    x_lb,x_ub,y_lb,y_ub)
% �ú�����ͨ�����ι滮����ƽ������·��
% path_x_init_set_gcs, path_x_init_set_gcs ��ƽ���켣����Ϣ
% w_smooth,w_length,w_ref ���ι滮Ȩ�� ƽ��Ȩ�� ����Ȩ��(ƽ��֮�󾡿��ܳ��ȶ�) ����Ȩ��(��֮ǰ·�������ܽӽ�)
% x_lb,x_ub,y_lb,y_ub ƽ��֮��x,y����������

% ��Ҫ����������ƽ��ǰ�켣��Ϣ��ƽ����켣��Ϣ  �����Ҫ���徲̬����������
% ���������ƽ��ǰ�켣��Ϣ��������ƽ��ǰ�켣һģһ�� ������ƽ��
persistent is_first_run;
persistent pre_path_x_init_set_gcs; persistent pre_path_y_init_set_gcs; 
persistent pre_path_x_set_gcs; persistent pre_path_y_set_gcs; 

if isempty(is_first_run)
    is_first_run = 0;
    % ��һ�� �����ж� ��Ҫƽ�� ���ö��ι滮ƽ��·������
    [path_x_set_gcs,path_y_set_gcs] = QpSmoothPath(path_x_init_set_gcs,path_y_init_set_gcs,...
    w_smooth,w_length,w_ref,x_lb,x_ub,y_lb,y_ub);
    % ��������ڽ�� ��������ʹ��
    pre_path_x_init_set_gcs = path_x_init_set_gcs; pre_path_y_init_set_gcs = path_y_init_set_gcs;
    pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
else
    % �жϱ�����ƽ��ǰ�켣��Ϣ��������ƽ��ǰ�켣�Ƿ�һģһ�� �ж����Ϳ��� ��Ϊ������������һ����
    if pre_path_x_init_set_gcs(1) == path_x_init_set_gcs(1) && pre_path_x_init_set_gcs(2) == path_x_init_set_gcs(2)
        % ����ƽ�� ֱ�Ӳ���������ƽ���������
        path_x_set_gcs = pre_path_x_set_gcs; path_y_set_gcs = pre_path_y_set_gcs;
    else
        % ����ƽ��
        [path_x_set_gcs,path_y_set_gcs] = QpSmoothPath(path_x_init_set_gcs,path_y_init_set_gcs,...
                                                        w_smooth,w_length,w_ref,x_lb,x_ub,y_lb,y_ub);
    end
     % ��������ڽ�� ��������ʹ��
     pre_path_x_init_set_gcs = path_x_init_set_gcs; pre_path_y_init_set_gcs = path_y_init_set_gcs;
     pre_path_x_set_gcs = path_x_set_gcs; pre_path_y_set_gcs = path_y_set_gcs;
end
end

