function [path_x_final_set_gcs,path_y_final_set_gcs] = QpSmoothPath(path_x_init_set_gcs,path_y_init_set_gcs,...
    w_smooth,w_length,w_ref,x_lb,x_ub,y_lb,y_ub)
%�ú�����ʹ�ö��ι滮�㷨��ƽ�������켣path
%���ι滮ͨ����ʽ g(x) = 0.5 * x' * H * x + f' * x;Լ������ lb <= x <= ub ͨ���������x����Сֵ
% ���� path_x_init_set_gcs,path_y_init_set_gcs δƽ���ĸ����켣�㼯�� 
% w_smooth,w_length,w_ref ƽ�����ۣ����մ��ۣ��������ƴ���Ȩ��ϵ�� x_lb,x_ub,y_lb,y_ub ��������x,����y�仯�����½�
% ��� path_x_final_set_gcs,path_y_final_set_gcsƽ����ĸ����켣

%simulink�޷�ֱ��ʹ���Դ��Ķ��ι滮��������Ҫ���ⲿ����
coder.extrinsic("quadprog");
% ���ι滮��ʽ  g(x) = 0.5 * x' * H * x + f' * x
%���ۺ���costfunction = x'*(w_smooth*A1'*A1+w_length*A2'*A2+w_ref*A3'*A3)*x +w_ref*h'*x
% ת���ɶ��ι滮��ʽ costfunction=0.5*x'*2[(w_smooth*A1'*A1+w_length*A2'*A2+w_ref*A3'*A3)]*x+ w_ref*h'*x
% ���Եõ� H = 2[(w_smooth*A1'*A1+w_length*A2'*A2+w_ref*A3'*A3)] f'=w_ref*h' f = w_ref * h
% ���ȼ����H��f  ����Ҫд��A1 A2 A3  ����x=(x1,y1,x2,y3,x3,y3.....xn,yn)'  2n*1  xi,yi��ƽ����켣������
% �Բ������г�ʼ��
% ����ƽ��ǰ�켣��ĸ���
n = length(path_x_init_set_gcs);
% ��������г�ʼ��
path_x_final_set_gcs = ones(n,1)*nan; path_y_final_set_gcs = ones(n,1)*nan; 
% ��Ȩ�ؾ�����г�ʼ��
A1 = zeros(2 * n - 4, 2 * n); A2 = zeros(2 * n - 2, 2 * n); A3 = eye(2 * n, 2 * n); h = ones(2*n,1)*nan;
% ��ƽ����켣��(x,y)�����޳�ʼ��
lb = zeros(2 * n, 1);    %(x,y)���������
ub = zeros(2 * n, 1);    %(x,y)���������
% ��ƽ������ֵ
for i = 1:(2*n-4)
    A1(i,i) = 1;
    A1(i,i+2) = -2;
    A1(i,i+4) = 1;
end
% �Խ��վ���ֵ
for i = 1:(2*n-2)
    A2(i,i) = -1;
    A2(i,i+2) = 1;
end
% ��(x,y)�����޺���ο��߾���������ֵ
for i = 1:n
    h(2*i-1) = -2 * path_x_init_set_gcs(i);
    h(2*i) = -2 * path_y_init_set_gcs(i);
    lb(2*i-1) = path_x_init_set_gcs(i) + x_lb;
    ub(2*i-1) = path_x_init_set_gcs(i) + x_ub;
    lb(2*i) = path_y_init_set_gcs(i) + y_lb;
    ub(2*i) = path_y_init_set_gcs(i) + y_ub;
end

% ������ι滮����
H = 2 * (w_smooth*(A1'*A1)+w_length*(A2'*A2)+w_ref*(A3'*A3));
f = w_ref * h;
%�� �Ż�ǰpath_init��Ϊ���ι滮�ĵ������
X0 = zeros(2 * n, 1); %X0Ϊ������ֵ  ��ʼ��
for i = 1:n
    X0(2*i-1) = path_x_init_set_gcs(i);
    X0(2*i) = path_y_init_set_gcs(i);
end
% ���ö��ι滮ģ�����
X = quadprog(H,f,[],[],[],[],lb,ub,X0); %������ƽ����Ĺ켣(x,y)���� X=[x1,y1,x2,y2,x3,y3,...xn,yn]
% ���Ż���Ľ����ֵ����������
for i = 1:n
    path_x_final_set_gcs(i) = X(2*i-1);
    path_y_final_set_gcs(i) = X(2*i);
end
end