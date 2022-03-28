function [a0,a1,a2,a3,a4,a5] = CalQuinticCoeffient(start_s,start_l,start_dl,start_ddl,end_s,end_l,end_dl,end_ddl)
    %通过五次多项式起点和终点的六个边界条件求出五次多项式系数
    % 写出五次多项式的系数矩阵 CalculateQuinticCoeffient
A = [1, start_s, start_s^2,  start_s^3,   start_s^4,   start_s^5;
        0, 1,  2*start_s,  3*start_s^2, 4*start_s^3, 5*start_s^4;
        0, 0,   2,     6*start_s, 12*start_s^2, 20*start_s^3;
        1, end_s, end_s^2,  end_s^3,   end_s^4,   end_s^5;
        0, 1,  2*end_s,  3*end_s^2, 4*end_s^3, 5*end_s^4;
        0, 0,   2,     6*end_s, 12*end_s^2, 20*end_s^3];
B = [start_l,start_dl,start_ddl,end_l,end_dl,end_ddl]';
coeff = A \ B;
% 取出五次多项式的六个系数 l = f(s) = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
a0 = coeff(1); a1 = coeff(2); a2 = coeff(3); a3 = coeff(4); a4 = coeff(5); a5 = coeff(6);
end