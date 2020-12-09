% MATLAB controller for Webots
% File:          rocket2.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

front_motor2=wb_robot_get_device('front_motor2');
rear_motor2=wb_robot_get_device('rear_motor2');
wb_motor_set_position(front_motor2,inf);
wb_motor_set_position(rear_motor2,inf);
wb_motor_set_velocity(front_motor2,-1);
wb_motor_set_velocity(rear_motor2,-1);

ds21=wb_robot_get_device('ds21');
wb_distance_sensor_enable(ds21,TIME_STEP);
ds22 =wb_robot_get_device('ds22');
wb_distance_sensor_enable(ds22,TIME_STEP);

lidar2=wb_robot_get_device('lidar2');
wb_lidar_enable(lidar2,TIME_STEP);
wb_lidar_enable_point_cloud(lidar2);




% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1

  wall_proximity21=wb_distance_sensor_get_value(ds21);
  wall_proximity22=wb_distance_sensor_get_value(ds22);
  
  if wall_proximity21<75 | wall_proximity22<75
    wb_motor_set_velocity(front_motor2,0);
    wb_motor_set_velocity(rear_motor2,0);
  end

  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
