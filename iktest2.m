clear;clc

xyz = [0 7 1];
lengths = [5.75 7.25 5];



    total_length = abs(xyz);
    pangles1 = 0:.01:pi;
    pangles2 = 0:.01:(3*pi/4);
    lengths = [5.75 7.25 5];

    if total_length > abs(lengths)
        fprintf('Outside of Arm Range')
    end
    
    o = [0 0 0];
    c = xyz;
    b = c + [0 0 lengths(1,3)];
    r1 = sqrt(c(1,1)^2 + c(1,2)^2); % X-Y endeffrctor projection
    r2 = sqrt(r1^2 + c(1,3)^2); % R-Z endeffrctor projection
    rzb = [r1 b(1,3)];
    arm1r = lengths(1,1)*cos(pangles1); % R-Z projection for arm 1
    arm1z = lengths(1,1)*sin(pangles1); % R-Z projection for arm 1
    arm1 = [arm1r; arm1z];
    arm2r = rzb(1,1) - lengths(1,2)*cos(pangles1); % R-Z projection for arm 1
    arm2z = rzb(1,2) - lengths(1,2)*sin(pangles1); % R-Z projection for arm 1
    arm2 = [arm2r; arm2z];

    matching = arm1 == arm2;
    why = sqrt(sum(arm2.^2,1))
    pos = find(abs(why - 5.75) < .1)
    pos = pos(1,1)
    theta1 = atan2(c(1,1),c(1,2));
    angle2 = atan2(b(1,3),r2);
    theta2 = atan2(arm2(2,pos),arm2(1,pos))
    a = rzb - [arm2(1,pos) arm2(2,pos)]
    angle3 = atan2(a(1,2),a(1,1))
    angle4 = -pi;
    theta3 = angle3 - theta2
    theta4 = angle4 - theta3
j = [theta1 theta2 theta3 theta4];j

    
    
    