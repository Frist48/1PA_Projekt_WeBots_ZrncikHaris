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
wb_motor_set_acceleration(front_motor2, 50);
wb_motor_set_acceleration(rear_motor2, 50);

ds21=wb_robot_get_device('ds21');
wb_distance_sensor_enable(ds21,TIME_STEP);
ds22 =wb_robot_get_device('ds22');
wb_distance_sensor_enable(ds22,TIME_STEP);

lidar2=wb_robot_get_device('lidar2');
wb_lidar_enable(lidar2,TIME_STEP);
wb_lidar_enable_point_cloud(lidar2);
offset = 0;




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
  
  %if wall_proximity21<75 | wall_proximity22<75
  wb_motor_set_velocity(front_motor2,0);
  wb_motor_set_velocity(rear_motor2,0);
  %end
  
    
  %if wall_proximity22 > 127
  %  offset = 0
  %elseif wall_proximity22 < 128 & wall_proximity22 > 91
  %  offset =  -4
  %elseif wall_proximity22 < 92 & wall_proximity22 > 54
  %  offset =  -5
  %elseif wall_proximity22 < 55
  %  offset =  -6
  %end
  
  %if wall_proximity21 > 128
  %  offset = 0
  %elseif wall_proximity21 < 128 & wall_proximity21 > 91
  %  offset =  4
  %elseif wall_proximity21 < 92 & wall_proximity21 > 54
  %  offset =  5
  %elseif wall_proximity21 < 55
  %  offset =  6
  %end
  

  %image = wb_lidar_get_range_image(lidar2)
  %i = 64 + offset;
  %c = 1;
  %buff(c) = image(i-1);
  %c = 2;
  %while i < (94 + offset)
  %  buff(c) = image(i);
  %  delta_buff(c) = abs(buff(c) - buff(c - 1));
  %  c=c+1;
  %  i=i+1;
  %end
  
  image = wb_lidar_get_range_image(lidar2)
  i = 60;
  c = 1;
  buff(c) = image(i-1);
  c = 2;
  while i < (98)
    buff(c) = image(i);
    delta_buff(c) = abs(buff(c) - buff(c - 1));
    c=c+1;
    i=i+1;
  end
  
  buff
  %min_value = min(buff)
  max_delta = max(delta_buff)
  %ball_pos = find(abs(buff-min_value) < 0.001)
  ball_pos2 = find(abs(delta_buff-max_delta) < 0.001)
  
  
  if ball_pos2 < 19 & wall_proximity21>40
    wb_motor_set_velocity(front_motor2,-8);
    wb_motor_set_velocity(rear_motor2,-8);
  elseif ball_pos2 > 19 & wall_proximity22>50
    wb_motor_set_velocity(front_motor2,8);
    wb_motor_set_velocity(rear_motor2,8);
  elseif ball_pos2 == 19
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
