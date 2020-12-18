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
wb_motor_set_acceleration(front_motor2, 50);
wb_motor_set_acceleration(rear_motor2, 50);

ds21=wb_robot_get_device('ds21');
wb_distance_sensor_enable(ds21,TIME_STEP);
ds22 =wb_robot_get_device('ds22');
wb_distance_sensor_enable(ds22,TIME_STEP);

lidar2=wb_robot_get_device('lidar2');
wb_lidar_enable(lidar2,TIME_STEP);
wb_lidar_enable_point_cloud(lidar2);

while wb_robot_step(TIME_STEP) ~= -1

  wall_proximity21=wb_distance_sensor_get_value(ds21);
  wall_proximity22=wb_distance_sensor_get_value(ds22);
  
  wb_motor_set_velocity(front_motor2,0);
  wb_motor_set_velocity(rear_motor2,0);
  
  image = wb_lidar_get_range_image(lidar2);
  i = 60;
  c = 1;
  buff(c) = image(i-1);
  c = 2;
  
  while i < (98)
    buff(c) = image(i);
    delta_buff(c) = abs(buff(c) - buff(c - 1));
    c = c+1;
    i = i+1;
  end
  
  max_delta = max(delta_buff);
  ball_pos2 = find(abs(delta_buff-max_delta) < 0.001);
  
  if ball_pos2 < 19 & wall_proximity21 > 30
    wb_motor_set_velocity(front_motor2,-10);
    wb_motor_set_velocity(rear_motor2,-10);
  elseif ball_pos2 > 19 & wall_proximity22 > 40
    wb_motor_set_velocity(front_motor2,10);
    wb_motor_set_velocity(rear_motor2,10);
  elseif ball_pos2 == 19
    wb_motor_set_velocity(front_motor2,0);
    wb_motor_set_velocity(rear_motor2,0);
  end

  drawnow;

end

