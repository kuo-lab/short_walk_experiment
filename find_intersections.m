function P = find_intersections(l1,l2,T_FF,A_FF)
%    find_intersections(l1, l2, T_FF, A_FF)
% find points of intersection in two lines l1 and l2
% A_FF is the distance threshold for intersection
% intersections must be separated by at least T_FF

if(~exist('T_FF','var') || isempty(T_FF))
  T_FF = floor(0.2/0.0078);
end

if(~exist('A_FF','var') || isempty(A_FF))
  A_FF = 0.02;
end

if length(l1) ~= length(l2)
    disp('Warning: trying to find intersection points of lines of different lengths')
end

near = find(abs(l1-l2) < A_FF);
if size(near,1) > size(near,2)
    near = near';
end
diff_near = diff(near);
valid_int = find(diff_near>T_FF);

start_int = near([1,valid_int+1]);  % size = 1 x number of intersections
end_int = near([valid_int,size(near,2)]);

P = zeros(2,length(start_int));
for i = 1:length(start_int)
   [~,intersect_ind] = min(abs(l1(start_int(i):end_int(i))-l2(start_int(i):end_int(i))));
   P(1,i) = intersect_ind+start_int(i)-1;
   P(2,i) = 0.5*(l1(P(1,i))+l2(P(1,i)));
   % sometimes the lines are close together for a long time at the end
   % because of both feet ending together at the end of the walk
   % take the first intersection point if this happens
   if i==length(start_int) && end_int(i)-start_int(i) > 10
       P(1,i) = start_int(i);
       P(2,i) = 0.5*(l1(P(1,i))+l2(P(1,i)));
   end
end

