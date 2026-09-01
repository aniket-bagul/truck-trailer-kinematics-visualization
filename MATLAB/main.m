clc; clf; clear;

%% Global controls
global quit_sim velocity steering_angle;
quit_sim = false;
velocity = 0;
steering_angle = 0;

%% Parameters
% Parameters matching VRML model (used also for 2D sim)
wheelbase_front = 12;      % Half truck length (truck box size is 16 in VRML)
total_length = wheelbase_front;
truck_width = 11.6;        % Truck width from VRML (size 9.6 in Z dir)

wheel_radius = 4.4;       % Truck and trailer wheel radius from VRML

trailer_length = 24.4;    % Trailer total length from VRML
trailer_wheelbase = 13.2; % Approx half trailer length, for wheelbase
trailer_width = 11.6;      % Trailer width from VRML

hitch_length = 10;         % Connecting rod length from VRML

% Clearance offset to avoid visual merging (adjustable)
clearance_offset = 3.5;
dt = 0.1;

%% Initial states
x = 0; y = 0; theta = 0;
trailer_x = x - (hitch_length + clearance_offset);
trailer_y = y;
trailer_theta = theta;

% Wheel tracking arrays
front_left_track = [];
front_right_track = [];

%% Setup figure
figure('KeyPressFcn', @keyboard_control, 'KeyReleaseFcn', @key_release);
hold on; axis equal;
xlim([-50 140]); ylim([-50 140]);
title('Truck-Trailer Simulation');
xlabel('X (m)'); ylabel('Y (m)');

% Load and show VRML world
world = vrworld('../VRML/test.wrl');
open(world);
fig = vrfigure(world);

% Get handles to VRML nodes
truckNode = vrnode(world, 'TruckInstance');
trailerNode = vrnode(world, 'TrailerInstance');
%connectingNode = vrnode(world, 'ConnectingInstance');

while ~quit_sim
    %% Update truck pose
    x = x + velocity * cos(theta) * dt;
    y = y + velocity * sin(theta) * dt;
    theta = theta + (velocity * tan(steering_angle) / total_length) * dt;

    %% Axle positions
    front_x = x + wheelbase_front * cos(theta);
    front_y = y + wheelbase_front * sin(theta);
    half_width = truck_width / 2;

    truck_front_left = [front_x - half_width * sin(theta), front_y + half_width * cos(theta)];
    truck_front_right = [front_x + half_width * sin(theta), front_y - half_width * cos(theta)];

    % Store wheel paths
    front_left_track(end+1, :) = truck_front_left;
    front_right_track(end+1, :) = truck_front_right;

    %% Trailer kinematics
    hitch_x = x - wheelbase_front * cos(theta);
    hitch_y = y - wheelbase_front * sin(theta);

    trailer_front_x = trailer_x + (trailer_wheelbase/2) * cos(trailer_theta);
    trailer_front_y = trailer_y + (trailer_wheelbase/2) * sin(trailer_theta);

    if velocity ~= 0
        delta_x = hitch_x - trailer_front_x;
        delta_y = hitch_y - trailer_front_y;
        angle_to_hitch = atan2(delta_y, delta_x);
        angle_diff = angle_to_hitch - trailer_theta;

        if abs(angle_diff) > pi
            angle_diff = angle_diff - sign(angle_diff) * 2 * pi;
        end
        trailer_theta = trailer_theta + angle_diff * 0.2;
    else
        trailer_theta = theta;
    end

    % Shift trailer back to add gap
    trailer_x = hitch_x - hitch_length * cos(trailer_theta);
    trailer_y = hitch_y - hitch_length * sin(trailer_theta);

    %% Trailer wheels
    trailer_front_x = trailer_x + (trailer_wheelbase/2) * cos(trailer_theta);
    trailer_front_y = trailer_y + (trailer_wheelbase/2) * sin(trailer_theta);
    trailer_rear_x = trailer_x - (trailer_wheelbase/2) * cos(trailer_theta);
    trailer_rear_y = trailer_y - (trailer_wheelbase/2) * sin(trailer_theta);

    trailer_front_left = [trailer_front_x - trailer_width/2 * sin(trailer_theta), trailer_front_y + trailer_width/2 * cos(trailer_theta)];
    trailer_front_right = [trailer_front_x + trailer_width/2 * sin(trailer_theta), trailer_front_y - trailer_width/2 * cos(trailer_theta)];
    trailer_rear_left = [trailer_rear_x - trailer_width/2 * sin(trailer_theta), trailer_rear_y + trailer_width/2 * cos(trailer_theta)];
    trailer_rear_right = [trailer_rear_x + trailer_width/2 * sin(trailer_theta), trailer_rear_y - trailer_width/2 * cos(trailer_theta)];

    %% Update VRML objects
    truckNode.translation = [x, 0, y];
    truckNode.rotation = [0, 1, 0, -theta];

    trailerNode.translation = [trailer_x, 0, trailer_y];
    trailerNode.rotation = [0, 1, 0, -trailer_theta];

    % === NEW: Update connecting rod position and rotation ===
    %truck_rear_x = x - wheelbase_front * cos(theta);
    %truck_rear_y = y - wheelbase_front * sin(theta);

    %rod_mid_x = (truck_rear_x + trailer_front_x) / 2;
    %rod_mid_y = (truck_rear_y + trailer_front_y) / 2;

    %delta_x = trailer_front_x - truck_rear_x;
    %delta_y = trailer_front_y - truck_rear_y;
    %rod_angle = atan2(delta_y, delta_x);

    %connectingNode.translation = [rod_mid_x, 0, rod_mid_y];
    %connectingNode.rotation = [0, 1, 0, -rod_angle];
    % =======================================================

    %% Visualization
    cla;

    % Draw wheel tracks
    if size(front_left_track,1) > 1
        plot(front_left_track(:,1), front_left_track(:,2), 'r:', 'LineWidth', 1.5);
        plot(front_right_track(:,1), front_right_track(:,2), 'g:', 'LineWidth', 1.5);
    end

    % Truck body
    truck_shape = [0, -truck_width/2;
                   wheelbase_front, -truck_width/2;
                   wheelbase_front, truck_width/2;
                   0, truck_width/2];
    R_truck = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    truck_body = (R_truck * truck_shape')' + [x, y];
    fill(truck_body(:,1), truck_body(:,2), 'y', 'EdgeColor', 'k');

    % Front wheels
    plot_steering_wheel(truck_front_left, wheel_radius, steering_angle, theta, 'r');
    plot_steering_wheel(truck_front_right, wheel_radius, steering_angle, theta, 'g');

    % Trailer body
    draw_basic_trailer(trailer_x, trailer_y, trailer_theta, trailer_length, trailer_width);

    % Trailer wheels
    plot_fixed_wheel(trailer_front_left, wheel_radius, trailer_theta, 'k');
    plot_fixed_wheel(trailer_front_right, wheel_radius, trailer_theta, 'k');
    plot_fixed_wheel(trailer_rear_left, wheel_radius, trailer_theta, 'k');
    plot_fixed_wheel(trailer_rear_right, wheel_radius, trailer_theta, 'k');

    % Hitch line
    plot([x, trailer_front_x], [y, trailer_front_y], 'k--', 'LineWidth', 1.5);

    pause(dt);
end

disp("Simulation ended.");

%% --- Helper Functions ---
function plot_steering_wheel(center, radius, steer_angle, truck_theta, color)
    length = 2 * radius;
    width = 0.6 * radius;
    shape = [-length/2, -width/2;
              length/2, -width/2;
              length/2,  width/2;
             -length/2,  width/2];
    R_steer = [cos(steer_angle), -sin(steer_angle); sin(steer_angle), cos(steer_angle)];
    R_truck = [cos(truck_theta), -sin(truck_theta); sin(truck_theta), cos(truck_theta)];
    R_total = R_truck * R_steer;
    wheel = (R_total * shape')' + center;
    fill(wheel(:,1), wheel(:,2), color, 'EdgeColor', 'k');
end

function draw_basic_trailer(x, y, theta, length, width)
    shape = [-length/2, -width/2;
              length/2, -width/2;
              length/2,  width/2;
             -length/2,  width/2];
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    box = (R * shape')' + [x, y];
    fill(box(:,1), box(:,2), [0.7 0.7 1], 'EdgeColor', 'k');
end

function plot_fixed_wheel(center, radius, theta, color)
    length = 2 * radius;
    width = 0.6 * radius;
    shape = [-length/2, -width/2;
              length/2, -width/2;
              length/2,  width/2;
             -length/2,  width/2];
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    wheel = (R * shape')' + center;
    fill(wheel(:,1), wheel(:,2), color, 'EdgeColor', 'k');
end

function keyboard_control(~, event)
    global velocity steering_angle;
    steer_limit = deg2rad(15);
    switch event.Key
        case {'w', 'uparrow'}
            velocity = velocity + 1;
        case {'s', 'downarrow'}
            velocity = velocity - 1;
        case 'space'
            velocity = 0;
        case {'a', 'leftarrow'}
            steering_angle = steer_limit;
        case {'d', 'rightarrow'}
            steering_angle = -steer_limit;
        case 'q'
            global quit_sim;
            quit_sim = true;
    end
end

function key_release(~, event)
    global steering_angle;
    if ismember(event.Key, {'a', 'd', 'leftarrow', 'rightarrow'})
        steering_angle = 0;
    end
end
