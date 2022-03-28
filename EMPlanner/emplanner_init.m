%%%%%EM PLANNER初始化与配置文件，主要是加载全局路径，加载油门刹车标定表，设置一些规划和控制参数等等

%%%%加载油门刹车标定表
load('table_calibration.mat')
%%%%加载全局路径
load('global_path.mat');
vs_state = -1;
StopMode = -1;

%%%%%前轮转角与方向盘转角的映射关系
right_wheel_ground=[-70 ,-67.2 , -64.4 , -61.6 , -58.8 , -56 , -53.2 , -50.4 , -47.6 , -44.8 , -42 , ...
    -39.2 , -36.4 , -33.6 , -30.8 , -28 , -25.2 , -22.4 , -19.6 , -16.8 , -14 , -11.2 , -8.4 , -5.6 , ...
    -2.8 , 0 , 2.8 , 5.6 , 8.4 , 11.2 , 14 , 16.8 , 19.6 , 22.4 , 25.2 , 28 , 30.8 , 33.6 , 36.4 , ...
    39.2 , 42 , 44.8 , 47.6 , 50.4 , 53.2 , 56 , 58.8 , 61.6 , 64.4 , 67.2 , 70 ];

rack_displacement=[-39.14 , -37.2 , -35.29 , -33.43 , -31.6 , -29.81 , -28.06 , -26.34 , -24.66 , ...
    -23.01 , -21.38 , -19.79 , -18.23 , -16.69 , -15.18 , -13.7 , -12.23 , -10.8 , -9.38 , -7.98 ,...
    -6.61 , -5.25 , -3.91 , -2.59 , -1.29 , 0 , 1.27 , 2.54 , 3.78 , 5.02 , 6.24 , 7.46 , 8.66 , ...
    9.86 , 11.05 , 12.24 , 13.41 , 14.59 , 15.76 , 16.92 , 18.09 , 19.25 , 20.42 , 21.59 , 22.76 , ...
    23.93 , 25.11 , 26.3 , 27.5 , 28.71 , 29.94];

%转向系统C特性
c_factor=43.75;%%单位: mm/rev
%%%%%
%%%参数设置%%%%%%
deg2rad=pi/180;
rad2deg=180/pi;
%%%%整车参数%%%%%
cf=-175016;
cr=-130634;
m=2020;
Iz=4095.0;
la=1.265;
lb=2.947-1.265;
wheel_distance = 1.624;

% 决策规划模块参数设置
% 二次规划平滑参考线权重参数
qp_referenceline_w_smooth = 1;
qp_referenceline_w_length = 2;
qp_referenceline_w_ref = 3;
qp_referenceline_x_lb = -0.2;
qp_referenceline_x_ub = 0.2;
qp_referenceline_y_lb = -0.2;
qp_referenceline_y_ub = 0.2;

% 动态规划求出粗接最优路径参数
dp_w_obs = 1e8;
dp_w_smooth_dl = 300; dp_w_smooth_ddl = 2000; dp_w_smooth_dddl = 10000;
dp_w_ref = 20;
dp_row = 9; dp_col = 4;
dp_sample_s = 15; dp_sample_l = 1;


%%%控制模块参数
control_predict_time = 0.01;   % 控制会有延迟  用0.01s车辆之后的状态来进行LQR控制
%%%%%%%横向LQR参数
LQR_Q1=25;
LQR_Q2=3;
LQR_Q3=40;
LQR_Q4=4;
LQR_R=5;
%%%%纵向双PID参数
KP_PID_distance=0.5;
KI_PID_distance=0.0;
KD_PID_distance=0.0;
KP_PID_speed=1.8;
KI_PID_speed=0;
KD_PID_speed=0;
%%%%%LQR_OFFLINE
k=zeros(5000,4);
vx_break_point=zeros(1,5000);
for i=1:5000
    vx_break_point(i)=0.01*i;
    
    A=[0,1,0,0;
        0,(cf+cr)/(m*vx_break_point(i)),-(cf+cr)/m,(la*cf-lb*cr)/(m*vx_break_point(i));
        0,0,0,1;
        0,(la*cf-lb*cr)/(Iz*vx_break_point(i)),-(la*cf-lb*cr)/Iz,(la*la*cf+lb*lb*cr)/(Iz*vx_break_point(i))];
    B=[0;
        -cf/m;
        0;
        -la*cf/Iz];
LQR_Q=1*[LQR_Q1,0,0,0;
        0,LQR_Q2,0,0;
        0,0,LQR_Q3,0;
        0,0,0,LQR_Q4];
   k(i,:)=lqr(A,B,LQR_Q,LQR_R);
   %LQR中参数Q 4阶单位矩阵  Q越大 算法性能越好 但导致u较大 稳定性变差
    %R=100; %LQR中参数R    R越大 控制过程越平缓 u变化相对较慢 舒适性高 跟踪效果相对差
    %k(i,:)=dlqr(A,B,Q,R); %lqr每次求解出来的K是1*4行向量  每次循环取出k的第i行进行赋值 每一行k对应每一个vx
    % 2020a版本必须使用lqr 不能使用dlqr
end
LQR_K1=k(:,1)';
LQR_K2=k(:,2)';
LQR_K3=k(:,3)';
LQR_K4=k(:,4)';
%%%%车辆初始位置
host_x_init=0; 
host_y_init=0;


