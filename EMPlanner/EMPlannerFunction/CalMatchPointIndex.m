function match_point_index = CalMatchPointIndex(x_gcs,y_gcs,path_x_set_gcs,path_y_set_gcs,...
    path_heading_set_gcs,path_kappa_set_gcs,start_find_index,increase_count_limit, forward_stable_backward)
% �ú������������ڸ���·���������µ�ƥ�����
%����:x_gcs,y_gcs�������Ϣ
%path_x_set_gcs,path_y_set_gcs,path_heading_set_gcs,path_kappa_set_gcs ����·����������Ϣ����
% start_find_index ����·�������㿪ʼѰ�ұ��   �༴�������ڸô�����ƥ�����
% increase_count_limit ������������
% forward_stable_backward ������������
% �����ʼ��
match_point_index = -1;

increase_count = 0; % ��¼����������������
min_distance_square = inf; %   ��ʼ������㵽����·����������С�����ƽ��
if forward_stable_backward > 0.001 % �ӿ�ʼѰ�ҵ�������·�������������������
    for i = start_find_index:length(path_x_set_gcs)
        distance_square = (x_gcs - path_x_set_gcs(i))^2 + (y_gcs - path_y_set_gcs(i))^2;
        if distance_square < min_distance_square
            min_distance_square = distance_square;
            match_point_index = i;  % ��¼��ǰ��С�����±�
            increase_count = 0;  
        else 
            increase_count = increase_count + 1;
        end 
        if increase_count > increase_count_limit
            break;   % ����Ѱ��increase_count_limit�ζ�û�ҵ�����С���뻹С��ֵ �˳�ѭ��
        end 
    end
elseif forward_stable_backward > 0.001 % �ӿ�ʼѰ�ҵ�������·���������ż�С�������
    for i = start_find_index:-1:1
        distance_square = (x_gcs - path_x_set_gcs(i))^2 + (y_gcs - path_y_set_gcs(i))^2;
        if distance_square < min_distance_square
            min_distance_square = distance_square;
            match_point_index = i;  % ��¼��ǰ��С�����±�
            increase_count = 0;  
        else 
            increase_count = increase_count + 1;
        end 
        if increase_count > increase_count_limit
            break;   % ����Ѱ��increase_count_limit�ζ�û�ҵ�����С���뻹С��ֵ �˳�ѭ��
        end 
    end
else  % ȡ��ʼѰ�ҵ� �༴�����ڵ�ƥ�������Ϊ�����ڵ�ƥ�����
    match_point_index = start_find_index;
end
end