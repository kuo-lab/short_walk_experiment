function [ff_dif] = time_shift(P1,P2,FF1,FF2,x0,PERIOD)

x0 = round(x0/PERIOD);
P2_shift = zeros(size(P2));
if x0 >= 0
    P2_shift(x0+1:end) = P2(1:end-x0);
else
   P2_shift(1:end-abs(x0)) = P2(abs(x0)+1:end); 
end

t=(0:length(P1)-1);
intersections = InterX([t; P1'],[t;P2_shift']);
%only one intersection after last footfall allowed
last_ff = find(intersections(1,:) > FF1(end));
if length(last_ff) > 1
    intersections(:,last_ff(2:end)) = [];
end
last_ff = find(intersections(1,:) > FF2(end));
if length(last_ff) > 1
    intersections(:,last_ff(2:end)) = [];
end
% only one intersection before second foot moves
first_ff = find(intersections(1,:) < FF2(1));
if length(first_ff) > 1
    intersections(:,first_ff(1:end-1)) = [];
end
%intersections(:,find(diff(intersections(1,:)) < 0.2)) = [];

dif_int = diff(intersections(1,:));

if length(dif_int) > 3
    count1=0;
    total_ff_dif1 = 0;
    for i=2:2:length(dif_int)-1
      total_ff_dif1 = total_ff_dif1+dif_int(i);
      count1 = count1+1;
    end
    ff_dif1 = total_ff_dif1/count1;
    
    count2=0;
    total_ff_dif2 = 0;
    for i=3:2:length(dif_int)-1
      total_ff_dif2 = total_ff_dif2+dif_int(i);
      count2 = count2+1;
    end
    ff_dif2 = total_ff_dif2/count2;   
    ff_dif = (ff_dif1-ff_dif2)^2;
    
else 
    % there should be at least one intersection for every step
 if length(dif_int) > length(FF1) || length(dif_int) > length(FF2)
     ff_dif = 0;
 else
     ff_dif = Inf;
 end
 
     
end %if length(dif_int) > 3
    
end


