function [path_heading_set_gcs,path_kappa_set_gcs] = CalHeadingAndKappa(path_x_set_gcs,path_y_set_gcs)
% �ú���ͨ������·���������ȫ������ϵ�µ�x,y�������������heading�Ǻ�kappa
% ���룺path_x_set_gcs,path_y_set_gcs �����켣���� ����������
% �����path_heading_set_gcs path_kappa_set_gcs �����쾶��heading������
% ���ò�ַ�
% ԭ�� heading = arctan(dy/dx)    ĳһ�������ǽ��Ƶ���tan[(y(n+1)-y(n))/(x(n+1)-x(n))] 
% kappa = dheading / ds
% ds = (dx^2 + dy^2)^0.5
% �������·��������ĸ���
n = length(path_x_set_gcs);
% �������ʼ��
path_heading_set_gcs = ones(n,1)*nan;
path_kappa_set_gcs = ones(n,1)*nan; 
% �Բ�����x,y������в�� �൱����һ��ֵ�뵱ǰֵ�Ĳ�ֵ��ΪԪ��
dx_set_gcs = diff(path_x_set_gcs);
dy_set_gcs = diff(path_y_set_gcs);
% ��ֻᵼ����һ���� ʹ���е�ŷ��������õ�
dx_forward_set_gcs = [dx_set_gcs(1);dx_set_gcs]; % ��ǰŷ����  �����һ����
dx_backward_set_gcs = [dx_set_gcs;dx_set_gcs(end)]; % ���ŷ����  �������һ����
dx_mid_set_gcs = (dx_forward_set_gcs + dx_backward_set_gcs) ./ 2; % �е�ŷ���� ��ӳ���2   ./��ʾ�������Ԫ�����
dy_forward_set_gcs = [dy_set_gcs(1);dy_set_gcs]; % ��ǰŷ����  �����һ����
dy_backward_set_gcs = [dy_set_gcs;dy_set_gcs(end)]; % ���ŷ����  �������һ����
dy_mid_set_gcs = (dy_forward_set_gcs + dy_backward_set_gcs) ./ 2; % �е�ŷ���� ��ӳ���2   ./��ʾ�������Ԫ�����

% ����atan2���Ա����ĸΪ0������޷�����   ����heading��ȡֵ��ΧΪ[-pi,pi] ��atan2��ȡֵ��Χ��ͬ
% atan2��һ����������C�����ﷵ�ص���ָ��λ�� y��x��ֵ�ķ��ž�������ȷ������
path_heading_set_gcs = atan2(dy_mid_set_gcs, dx_mid_set_gcs);

% ��heading���в��
dheading_set_gcs = diff(path_heading_set_gcs);
dheading_forward_set_gcs = [dheading_set_gcs(1);dheading_set_gcs]; % ��ǰŷ����  �����һ����
dheading_backward_set_gcs = [dheading_set_gcs;dheading_set_gcs(end)]; % ���ŷ����  �������һ����
dheading_mid_set_gcs = (dheading_forward_set_gcs + dheading_backward_set_gcs) ./ 2; % �е�ŷ���� ��ӳ���2   ./��ʾ�������Ԫ�����

% ����ds = (dx^2 + dy^2)^0.5
ds_set_gcs = sqrt(dx_mid_set_gcs.^2 + dy_mid_set_gcs.^2); %������Ԫ�ؼ��� ʹ�õ�˻��� ����������ʹ��sqrt����.^0.5

%Ϊ�˷�ֹdheading���ֶ�һ��2pi�Ĵ���(�ر��Ƿ���ת��x�Ḻ����ʱ)��һ���������ʵ��dheading��С����sin(dheading)����dheading
path_kappa_set_gcs = sin(dheading_mid_set_gcs) ./ ds_set_gcs;
end