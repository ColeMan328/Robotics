% Cole Stoumbaugh
clear;clc

% Check to ensure com ports are closed to avoid "re-open" error
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% Opening serial port and setting it equal to variable 's'
s = serial('COM3');
set(s,'Terminator','CR');
set(s,'BaudRate',9600);
fopen(s)

% moving arm to 'home' position
homePosition(s)

% start continuous function for moving arm
runningCode(s)

% continuous function for moving arm
function runningCode(s)
    while 1==1
        lengths = [5.75 7.25 5];    % measured arm lengths
        xyz = input('Input xyz destination \n');    % input desired position in xyz frame
        if xyz == 1
            homePosition(s)
        elseif xyz == 0
            quit
        else
            j = InverseKin(xyz, lengths)
            P1 = angleToMV(j(1,1));
            P2 = angleToMV2(j(1,2));
            P3 = angleToMV2(j(1,3));
            P4 = angleToMV3(j(1,4));
            
            fprintf(s, sprintf('#0 P%.0f #1 P%.0f #2 P%.0f #3 P%.0f T6000' ,P1,(P2+200),P3,P4))
        end
    end
end

function j = InverseKin(xyz, lengths)

    total_length = abs(xyz);
    pangles1 = 0:.0001:pi;
    pangles2 = 0:.0001:(3*pi/4);
    lengths = [5.75 7.25 5];

    if total_length > abs(lengths)
        fprintf('Outside of Arm Range')
    end
    
    o = [0 0 2.75];
    c = xyz;
    r1 = sqrt(c(1,1)^2 + c(1,2)^2) % X-Y endeffrctor projection
    r2 = r1 - lengths(1,3) % R-Z endeffrctor projection
    rzb = [r1 c(1,3)]
    arm1r = lengths(1,1)*cos(pangles1); % R-Z projection for arm 1
    arm1z = lengths(1,1)*sin(pangles1); % R-Z projection for arm 1
    arm1 = [arm1r; arm1z];
    arm2r = rzb(1,1) - lengths(1,2)*cos(pangles1); % R-Z projection for arm 1
    arm2z = rzb(1,2) - lengths(1,2)*sin(pangles1); % R-Z projection for arm 1
    arm2 = [arm2r; arm2z];

    matching = arm1 == arm2;
    why = sqrt(sum(arm2.^2,1));
    pos = find(abs(why - 5.75) < .1);
    pos = pos(1,1);
    theta1 = atan2(c(1,1),c(1,2));
    b = c - [cos(lengths(1,1)) sin(lengths(1,1)) 0];
    angle2 = atan2(b(1,3),r2);
    theta2 = atan2(arm2(2,pos),arm2(1,pos));
    a = rzb - [arm2(1,pos) arm2(2,pos)];
    angle3 = atan2(a(1,2),a(1,1));
    angle4 = -pi;
    theta3 = angle3 - theta2;
    theta4 = angle4 - theta3;
    j = [theta1 theta2 theta3 theta4];
end
%{
function joint_angles = inverse_kinematics(end_effector_pos, arm_lengths)
    % Computes the joint angles of a robot arm using inverse kinematics.

    % Input:
    % - end_effector_pos: A 3D array representing the desired end effector position [x, y, z].
    % - arm_lengths: A vector of the lengths of the robot arm segments.

    % Output:
    % - joint_angles: A vector of the joint angles [theta1, theta2, theta3, ..., thetaN] in radians.

    % Extract the end effector position coordinates
    x = end_effector_pos(1,1);
    y = end_effector_pos(1,2);
    z = end_effector_pos(1,3);

    % Compute the distance from the base of the robot arm to the end effector in the XY plane
    r = sqrt(x^2 + y^2);

    % Compute the joint angles
    theta1 = atan2(y, x);
    theta2 = atan2(z - arm_lengths(1), r);

    % Compute the distance from the second joint to the end effector in the XY plane
    r2 = sqrt(r^2 + (z - arm_lengths(1))^2);

    % Use the law of cosines to compute the third joint angle
    a = arm_lengths(2);
    b = arm_lengths(3);
    c = r2;
    cos_theta3 = (a^2 + b^2 - c^2) / (2 * a * b);
    sin_theta3 = sqrt(1 - cos_theta3^2);
    theta3 = atan2(sin_theta3, cos_theta3);

    % Return the joint angles
    joint_angles = [theta1, theta2, theta3];
end
%}
function homePosition(s)
    % Home positionzxcv
    % 0=base 1=arm1 2=arm2 3=clawtheta 4=clawrot 5=clawclamp
    % L1=6 L2=7.5 L3=5
    
    fprintf(s, '#0 P1450 #1 P1600 #2 P1500 #3 P1500 #4 P1500 #5 P1500 T6000')
end

function P = angleToMV(theta)
    global P
    if theta>0 || theta==0
        P = (1800/pi)*theta + 550
    else
        P = abs((1800/pi)*theta - 550) + 900
    end
end

function P = angleToMV2(theta)
    global P
    P = (1800/pi)*abs(theta) + 550
end

function P = angleToMV3(theta)
    global P
    P = (1800/pi)*((pi/2)-abs(theta)) + 550
end
