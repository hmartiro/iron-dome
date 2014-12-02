clear all;
clc;
syms R x0 y0 z0 vx vy vz t g;
initialCondition = [1.4 -1 -1 -0.5 2 2 2 -9.81];
x = x0 + vx*t;
y = y0 + vy*t;
z = z0 + vz*t + 1/2*g*t^2;

longeq = solve(x^2+y^2+z^2==R^2,t,'MaxDegree',4);
root = subs(longeq,[R x0 y0 z0 vx vy vz g],...
    [initialCondition]);
tIntersect = vpa(root(imag(vpa(root))==0))



