clc;
prompt = '1st point, Kinect frame: ';
a1 = input(prompt);
% prompt = '1st point, Kuka frame: ';
% a2 = input(prompt);
prompt = '2nd point, Kinect frame: ';
b1 = input(prompt);
% prompt = '2nd point, Kuka frame: ';
% b2 = input(prompt);
prompt = '3rd point, Kinect frame: ';
c1 = input(prompt);
% prompt = '3rd point, Kuka frame: ';
% c2 = input(prompt);
prompt = '4th point, Kinect frame: ';
d1 = input(prompt);
% prompt = '4th point, Kuka frame: ';
% d2 = input(prompt);
%% preset points
% a2 = [2-.27623,1.08204-1.7729,1.4873];
% b2 = a2 + [+.381,0,0]; % 15 inches
% c2 = a2 + [0,+0.381,0];
% d2 = a2 +[0 0 -0.5];
a2 = [2.1048    0.3836    0.9873];
b2 = [2.1048   -0.6909    1.4884];
c2 = [2.1048    0.3836    1.9177];
d2 = [3.7520        0    1.36906];

%% Use this section with calibration_data.mat

p = zeros(size(d2));

P = [(b1(1)-a1(1)) (b1(2)-a1(2)) (b1(3)-a1(3)) ;...
    (c1(1)-a1(1)) (c1(2)-a1(2)) (c1(3)-a1(3));...
   (d1(1)-a1(1)) (d1(2)-a1(2)) (d1(3)-a1(3)) ];
Q = [(b2(1)-a2(1)) (b2(2)-a2(2)) (b2(3)-a2(3));...
    (c2(1)-a2(1)) (c2(2)-a2(2)) (c2(3)-a2(3));...  
    (d2(1)-a2(1)) (d2(2)-a2(2)) (d2(3)-a2(3))];

R = inv(P)*Q;
R = R/norm(R);
p = a2- a1*R;
% homogeneous transformation matrix
T = [inv(R) p';0 0 0 1]

