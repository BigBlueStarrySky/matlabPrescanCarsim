function [proj_point_x_gcs,proj_point_y_gcs,proj_point_heading_gcs,proj_point_kappa_gcs] = ...
    CalProjPointInfoFromFrenet(s_fcs,path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs,index_to_s_set)
% �ú����������� ��Ȼ����ϵ�ĵ�(s,l)��·����������ͶӰ���ֱ��������Ϣ
% ��ͶӰ�� ·����������ǰһ����Ϊpre_index ·���������Ϻ�һ����Ϊbehind_index
% ���ͶӰ��ֱ�Ӿ�������������ɢ�ĵ� ��ֱ��ȡ��Ӧ����Ϣ����
% ����:
% index_to_s_set ���Գ�ͶӰ����Ϊs��ԭ��  ��ת�����ڸ���·����ƥ����±���s������Ķ�Ӧ��ϵ
% s����  �൱���Ǽ��㵱ǰת����ͶӰ�㵽s������ԭ��Ļ���
% ��������� һ�־��Ǵ�ת�����s����պ���index_to_s_setĳ��s��������� ˵����ʱͶӰ��պ���ƥ����غ� 
% �ڶ��ִ�ת�����s�������index_to_s_set����ĳ����s������֮��
% �����ʼ��
proj_point_x_gcs = -1; proj_point_y_gcs = -1; 
proj_point_heading_gcs = -1; proj_point_kappa_gcs = -1;

if s_fcs >= index_to_s_set(end)
% ȡ���һ��ƥ������Ϣ��ΪͶӰ����Ϣ����
    proj_point_x_gcs = path_x_set_gcs(end); proj_point_y_gcs = path_y_set_gcs(end);
    proj_point_heading_gcs = path_heading_set_gcs(end); proj_point_kappa_gcs = path_kappa_set_gcs(end);
else	
    for i = 1:length(index_to_s_set)-1
        % ��һ�����
        if s_fcs == index_to_s_set(i)
            % ȡƥ������Ϣ��ΪͶӰ����Ϣ����
            proj_point_x_gcs = path_x_set_gcs(i); proj_point_y_gcs = path_y_set_gcs(i);
            proj_point_heading_gcs = path_heading_set_gcs(i); proj_point_kappa_gcs = path_kappa_set_gcs(i);
            break;
        % �ڶ������ ȡͶӰ����������ǰһ����ɢ��ͺ�һ����ɢ��������
        elseif s_fcs > index_to_s_set(i) && s_fcs < index_to_s_set(i+1)
            % ѡȡǰһ����ɢ��ͺ�һ����ɢ��heading�ǵ�ƽ��ֵ
            avg_heading = (path_heading_set_gcs(i) + path_heading_set_gcs(i+1)) / 2;
            ds = s_fcs - index_to_s_set(i); %��ת����ͶӰ�㵽ǰһ����ɢ��Ļ��� ���Խ���Ϊֱ�߾���
            proj_point_x_gcs = path_x_set_gcs(i) + ds * cos(avg_heading);
            proj_point_y_gcs = path_y_set_gcs(i) + ds * sin(avg_heading);
            % dtheta = km * ds
            proj_point_heading_gcs = path_heading_set_gcs(i) + path_kappa_set_gcs(i) * ds;
            % ѡȡǰһ����ɢ��ͺ�һ����ɢ��kappa��ƽ��ֵ��Ϊ��ת����ͶӰ���kappa
            proj_point_kappa_gcs = (path_kappa_set_gcs(i) + path_kappa_set_gcs(i+1)) / 2;
            break;
        end
    end
end
end