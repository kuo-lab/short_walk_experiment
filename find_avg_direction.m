function [mean_step_direction] = find_avg_direction(steps_x,steps_y)
% find_avg_direction(steps_x, steps_y) determines the walking bout
% direction, based on the covariance of the steps

c = cov(steps_x,steps_y);
[V,D] = eig(c);
[d,ind] = sort(diag(D));
Ds = D(ind,ind);
Vs = V(:,ind);
x = Vs(2,1);
y = Vs(2,2);
mean_step_direction = atan2(y,x);
direction_check = atan2(steps_y(end)-steps_y(1),steps_x(end)-steps_x(1));
if (abs(direction_check-mean_step_direction) > pi/2 && abs(direction_check-mean_step_direction-2*pi) > pi/2 && abs(direction_check-mean_step_direction+2*pi) > pi/2)
   mean_step_direction = mean_step_direction + pi; 
end