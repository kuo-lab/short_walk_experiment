%% Short walking experiment, Carlisle & Kuo
%% Preliminary initialization
subjects = 1:10;
distances = 1:10;
section = [1 3];

% distances in meters
actual_dist = [1.1 1.7 2.2 2.8 3.3 3.8 5.1 7 9.1 12.7];
PERIOD = 0.0078;
trials=['a', 'b']; % 2 trials of each distance in each section
colors = jet(length(actual_dist));

x_save = zeros(800,5);
% subject, distance, whether optimization was used to rescale/shift,
% rescale factor, shift
xcount = 1;

peak_speed = zeros(10,length(actual_dist),length(trials)*length(section)); 
walk_duration = zeros(10,length(actual_dist),length(trials)*length(section));
speed_all_sub = {};
times_all_sub = {};
length_all_sub = {};
freq_all_sub = {};
rise_times_all_sub = zeros( length(subjects), 3, length(distances), length(section)*length(trials));
speed_dist_fig = figure;
%% Loop through subjects
% load in all walking bouts, plot speed trajectory vs time, 
% peak speed vs distance, and duration vs distance
for isubject=subjects
  fprintf(1, '\nLoading subject %d distances ', isubject);
  % this is where figures should get saved
  save_dir = ['results/subject' int2str(isubject) '/'];
  if ~exist('results', 'dir')
       mkdir('results')
  end
  if ~exist(save_dir, 'dir')
       mkdir(save_dir)
  end
  step_speed_all_dist = {};
  step_ff_all_dist = {};
  step_length_all_dist = {};
  step_freq_all_dist = {};
  
% Loop through the different walking distances
  for idistance = distances
    fprintf(1,'%d ', idistance);
    step_speed_all = {};
    step_speed_ff_all = {};
    step_length_all = {};
    step_freq_all = {};
    count = 1;
    for isection=section % 4 sections
        if isubject == 3 && isection == 3
           isection = 4; % subject 3 sections are out of order
        end
      for itrial=1:length(trials) % 2 trials per section
    
      % file where walk infos are saved
      saved_file = ['saved_walk_info/new/subject' num2str(isubject) 'distance' num2str(idistance) 'section' num2str(isection) 'trial' num2str(itrial) '.mat'];
      load(saved_file)
      
      % check which foot takes the first step
      if right_walk_info.FFstart(1) < left_walk_info.FFstart(1)
        FF1 = [right_walk_info.FFstart right_walk_info.FFend(end)];
        FF2 = [left_walk_info.FFstart left_walk_info.FFend(end)];
        walk_info1 = right_walk_info;
        walk_info2 = left_walk_info;
        label1 = 'Right foot';
        label2 = 'Left foot';
        color1 = '#1D3557';
        color2 = '#E63946';
      else
        FF1 = [left_walk_info.FFstart left_walk_info.FFend(end)];
        FF2 = [right_walk_info.FFstart right_walk_info.FFend(end)];
        walk_info1 = left_walk_info;
        walk_info2 = right_walk_info;
        label1 = 'Left foot';
        label2 = 'Right foot';
        color1 = '#E63946';
        color2 = '#1D3557';
      end

      if FF1(end) > FF2(end) % check which foot fall is last
        ffend = FF1(end);
      else
        ffend = FF2(end);
      end

      % rotate the foot paths so that they align with the x-axis
      % because IMUs do not know heading direction
      ang2 = find_avg_direction(walk_info2.P(FF1(1):ffend,1),walk_info2.P(FF1(1):ffend,2));
      Prot2 = rotateP(walk_info2.P(FF1(1):ffend,:),ang2,'z'); 
      P2 = Prot2(:,1); 
      % P2, P1 start at first footfall, end at last footfall
      ang1 = find_avg_direction(walk_info1.P(FF1(1):ffend,1),walk_info1.P(FF1(1):ffend,2));
      Prot1 = rotateP(walk_info1.P(FF1(1):ffend,:),ang1,'z');
      P1 = Prot1(:,1);
      
      % adjust the footfall indices
      offset = FF1(1);
      FF1 = FF1-offset+1; 
      FF2 = FF2-offset+1;
      
      % P2 gets shifted and rescaled to line up with P1
      P2 = P2-P2(FF2(1));
      P1 = P1-P1(FF1(1));
      start_scale = (P1(FF1(end))-P1(FF1(1)))/(P2(FF2(end))-P2(FF2(1)));
      if length(FF1) > 4
        % if there's more than one footfall before the second foot moves
        % only use the last one
        ff1 = FF1;
        ff1ind = find(FF1 < FF2(1));
        if length(ff1ind) > 1
          ff1(ff1ind(1:end-1)) = [];  
        end
        halfp1 = P1(ff1(2:end-2))+0.5*diff(P1(ff1(2:end-1)));
        start_shift = mean(halfp1-P2(FF2(2:length(halfp1)+1)));
        f = @(x)scale_and_shift(P1,P2,ff1,FF2,x); % the imus don't exactly match in distance traveled
        x0 = [start_scale; start_shift]; % so find the best scaling between them
        [x,fval,exitflag,output] = fminunc(f,x0); % to enable consistent step length measures
        P2_shift = (P2*x(1))+x(2);
        % save the x for later
        x_save(xcount,3) = 1; % whether or not optimization was used
        x_save(xcount,4) = x(1);
        x_save(xcount,5) = x(2);
        
        % shift in time
        f2 = @(x)time_shift(P1,P2_shift,ff1,FF2,x,PERIOD);
        %x0 = 0;
        %[xt,fval,exitflag,output] = fmincon(f2,x0,1,0.5);
        
        xsearch=-0.5:0.01:0.5;
        ysearch = zeros(size(xsearch));
        for i=1:length(xsearch)
          ysearch(i) = f2(xsearch(i));
        end 
        [m,i] = min(ysearch);
        xt = round(xsearch(i)/PERIOD);
        P2_shift_new = zeros(size(P2_shift));
        
        if xt > 0
          P2_shift_new(xt+1:end) = P2_shift(1:end-xt);
          P2_shift_new(1:xt) = P2_shift(xt+1);
        else
          P2_shift_new(1:end-abs(xt)) = P2_shift(abs(xt)+1:end);
          P2_shift_new(end-abs(xt)+1:end) = P2_shift_new(end-abs(xt));
        end
        P2_shift = P2_shift_new;
        
      else
          % for short walks, rescale but don't shift 
          P2_shift = P2*start_scale;
          x_save(xcount,4) = start_scale;
      end % if length(FF1) > 4
      
      x_save(xcount,1) = isubject;
      x_save(xcount,2) = idistance;
      xcount=xcount+1;
      
%       intersect_inds = find(abs(P2_shift-P1) < 0.05);
%       inds2 = find(diff(intersect_inds)>1);
%       
%       if isempty(inds2)
%           disp('bad')
%       end
%       
%       inds2 = [max([0 inds2(1)-3]); inds2; min([length(intersect_inds) inds2(end)+3])];
%       intersect_points = zeros(length(inds2)-1,1);
%       for i=2:length(inds2)
%         [m,mi] = min(abs(P2_shift(intersect_inds(inds2(i-1)+1:inds2(i)))-P1(intersect_inds(inds2(i-1)+1:inds2(i)))));
%         intersect_points(i-1)=intersect_inds(inds2(i-1)+mi);
%       end
%       
%       t=(0:length(P1)-1)*PERIOD;
%       step_speed1 = diff(P1(intersect_points)) ./ diff(t(intersect_points)');
%       step_speed_ff1 = t(intersect_points);
%       %make speed start and end at 0
%       if step_speed_ff1(1) > 0
%       step_speed = zeros(length(step_speed1)+2,1);
%       step_speed_ff = zeros(length(step_speed_ff1)+1,1);
%       step_speed(2:end-1) = step_speed1;
%       step_speed_ff(2:end) = step_speed_ff1;
%       else
%           step_speed_ff = step_speed_ff1;
%           step_speed = step_speed1;
%           step_speed(end+1) = 0;
%       end
%       step_length = zeros(length(step_speed),1);
%       step_length(2:end-1) = diff(P1(intersect_points));
%       step_freq = zeros(length(step_speed),1);
%       step_freq(2:end-1) = ones(length(step_speed)-2,1)./diff(t(intersect_points)');
%       

t=(0:length(P1)-1)*PERIOD;
T_FF = floor(0.2/PERIOD);
intersections = find_intersections(P1,P2_shift,T_FF);
intersections(1,:) = intersections(1,:)*PERIOD;
%intersections = InterX([t; P1'],[t;P2_shift']);
%intersections(:,find(intersections(1,:) > FF1(end)*PERIOD)) = [];
%only one intersection after last footfall allowed
last_ff = find(intersections(1,:) > FF1(end)*PERIOD);
if length(last_ff) > 1
    intersections(:,last_ff(2:end)) = [];
end
%intersections(:,find(intersections(1,:) > FF2(end)*PERIOD)) = [];
last_ff = find(intersections(1,:) > FF2(end)*PERIOD);
if length(last_ff) > 1
    intersections(:,last_ff(2:end)) = [];
end
intersections(:,find(diff(intersections(1,:)) < 0.2)) = [];

% manual adjustments
%if isubject==6 && idistance==4 && isection==1 && itrial==2
%    intersections(:,6) = [3.1824; 2.7383];
%elseif isubject==3 && idistance==1 && isection==1 && itrial==2
%    intersections(:,1) = [];
%elseif (isubject==1 && idistance==3 && isection==3 && itrial==2) || (isubject==3 && idistance==7 && isection==1 && itrial==2) || (isubject==4 && idistance==3 && isection==1 && itrial==2)
%    intersections(:,end) = [];
%end

ff1 = FF1;
ff1ind = find(FF1 < FF2(1));
if length(ff1ind) > 1
  ff1(ff1ind(1:end-1)) = [];  
end

if isubject==2 && idistance==10 && isection==1 && itrial==2
    ff1 = ff1(1:end-1);
end

skipcorrection = 0;
if (isubject==2 && idistance==10 && isection==3 && itrial==2) || (isubject==3 && idistance==10 && isection==1 && itrial==2)|| (isubject==6 && idistance==9 && isection==1 && itrial==2)
    skipcorrection = 1;
elseif isubject==3 && idistance==9 && isection==1 && itrial==2
    skipcorrection = 1;
    intersections = intersections(:,1:end-1);
end



if size(intersections,2) ~= length(ff1)+length(FF2)-2 && ~skipcorrection
   % possibly missing a step 
   plot_foot_position(P1,P2_shift,ff1,FF2,label1,label2,color1,color2,PERIOD);
   missing_first = abs(FF2(1)*PERIOD - intersections(1,1)) > 0.5;
   missing_last = (abs(FF2(end)*PERIOD - intersections(1,end)) > 0.5 && abs(ff1(end)*PERIOD - intersections(1,end)) > 0.5);
   if (isubject==2 && idistance==7 && isection==2 && itrial==2) || (isubject==2 && idistance==8 && isection==2 && itrial==1) || (isubject==2 && idistance==9 && isection==2 && itrial==1) || (isubject==3 && idistance==9 && isection==2 && itrial==1)
       missing_last = 0;
   end
   if (isubject==6 && idistance==7 && isection==4 && itrial==2) || (isubject==9 && idistance==5 && isection==2 && itrial==2) || (isubject==9 && idistance==9 && isection==2 && itrial==1) || (isubject==9 && idistance==10 && isection==2 && itrial==1)
       missing_last = 1;
   end
   if missing_first
      % possibly missing first step 
      dist = P2_shift(FF2(1)) - P1(ff1(1));
      disp('missing first step')
      if dist < 0 % shift P1 down and P2 up
        tmp_p1 = P1(1:ff1(2)-20) + 0.5*dist;
        tmp_p2 = P2_shift(1:ff1(2)-20) - 0.5*dist;
      else % shift P2 down and P1 up
        tmp_p2 = P2_shift(1:FF2(2)-20) - 0.5*dist;
        tmp_p1 = P1(1:FF2(2)-20) + 0.5*dist;
      end
      first_int = find_intersections(tmp_p1,tmp_p2,T_FF);
      first_int(1,:) = first_int(1,:)*PERIOD;
      if(size(first_int,2) > 1)
          disp('Warning: check intersection points');
          if isubject==2 && idistance==6 && isection==1 && itrial==1
              first_int = first_int(:,1);
          end
      end
      if isubject==3 && idistance==7 && isection==1 && itrial==1
          intersections(:,1) = first_int(:,1);
      else
          intersections = [first_int,intersections];
      end
   end
   if missing_last || (isubject==7 && idistance==9 && isection==3 && itrial==1)
      % possibly missing last step 
      disp('missing last step')
      if P2_shift(FF2(end)) < P1(ff1(end))
          dif = P1(ff1(end)) - P2_shift(FF2(end));
          tmp_p2 = P2_shift(FF2(end-1)+20:end) + 0.5*dif; % shift up last section of P2
          tmp_p1 = P1(FF2(end-1)+20:end) - 0.5*dif;
          last_int = find_intersections(tmp_p1,tmp_p2,T_FF);
          last_int(1,:) = (FF2(end-1)+19+last_int(1,:))*PERIOD;
          % there shouldn't be more than one intersection point here
          if(size(last_int,2) > 1)
             disp('Warning: check intersection points'); 
             if (isubject==6 && idistance==5 && isection==4 && itrial==2)
                 last_int = last_int(:,1);
             end
          end
          intersections = [intersections,last_int];
      elseif P1(ff1(end)) < P2_shift(FF2(end))
          dif = P2_shift(FF2(end)) - P1(ff1(end));
          tmp_p1 = P1(ff1(end-1)+20:end) + 0.5*dif; % shift up last section of P1
          tmp_p2 = P2_shift(ff1(end-1)+20:end) - 0.5*dif;
          last_int = find_intersections(tmp_p1,tmp_p2,T_FF);
          last_int(1,:) = (ff1(end-1)+19+last_int(1,:))*PERIOD;
          % there shouldn't be more than one intersection point here
          if(size(last_int,2) > 1)
             disp('Warning: check intersection points'); 
             if isubject==6 && idistance==4 && isection==1 && itrial==2
                 last_int = last_int(:,1);
             end
          end
          intersections = [intersections,last_int];          
          
      end   
   end % if missing last step
   if ~(missing_first || missing_last)
       disp('Possible step missing; check intersection points')
       disp(['Subject ',num2str(isubject),' Distance ',num2str(idistance),' Trial ',num2str(isection),trials(itrial)])
       if isubject==3 && idistance==1 && isection==1 && itrial==2
           intersections = intersections(:,2:end);
       elseif isubject==6 && idistance==3 && isection==1 && itrial==2
           intersections(:,1) = [0.9438; 0.0341];
       end
   end
end % if size(intersections,2) ~= length(ff1)+length(FF2)-2

step_speed_ff = [0 intersections(1,:)];
step_length = [0 diff(intersections(2,:)) 0];
step_freq = [0 1./diff(intersections(1,:)) 0];
step_speed = step_length.*step_freq;

if isempty(intersections)
   disp('something went wrong') 
end

       if(max(step_freq) > 2.5) % if unrealistic step frequency detected, show which subject, distance, etc.
           %plot_foot_position(P1,P2_shift,FF1,FF2,label1,label2,color1,color2,PERIOD);
           disp(isubject)
           disp(idistance)
           disp(isection)
           disp(itrial)           
       end  
      
      % plot the left and right foot position
      %P2_shift(1:FF2(1)) = P2_shift(FF2(1));
      %plot_foot_position(P1,P2_shift,FF1,FF2,label1,label2,color1,color2,PERIOD);
      
      % rise time calculation
      xq = step_speed_ff(1):0.01:step_speed_ff(end);
      interp_speed = interp1(step_speed_ff,step_speed,xq);
      m = max(step_speed);
      f = find(interp_speed >= 0);
      ind1 = f(1);
      ind4 = f(end);
      f = find(interp_speed >= 0.9*m);
      ind2 = f(1);
      ind3 = f(end);
      %rise_time1 = zeros(4,1);
      %rise_time1(1) = xq(ind1);
      %rise_time1(2) = xq(ind2);
      %rise_time1(3) = xq(ind3);
      %rise_time1(4) = xq(ind4);
      %rise_times_all_sub(isubject,:,idistance,count)=diff(rise_time1);
      rise_times_all_sub(isubject,1,idistance,count) = xq(ind2);
      rise_times_all_sub(isubject,2,idistance,count) = xq(ind3)-xq(ind2);
      rise_times_all_sub(isubject,3,idistance,count) = step_speed_ff(end) - xq(ind3);
      
      % save peak speed and walk_duration
      peak_speed(isubject,idistance,count) = max(step_speed);
      walk_duration(isubject,idistance,count) = step_speed_ff(end); %length(P1)*PERIOD;
      
      % save step speeds for later
      step_speed_all{count} = step_speed;
      step_speed_ff_all{count} = step_speed_ff;
      
      step_length_all{count} = step_length;
      step_freq_all{count} = step_freq;
      
      count = count+1;
      end % for itrial
    end % for isection
    
    step_speed_all_dist{idistance} = step_speed_all;
    step_ff_all_dist{idistance} = step_speed_ff_all;
    step_length_all_dist{idistance} = step_length_all;
    step_freq_all_dist{idistance} = step_freq_all;
  end % loop for idistance
  fprintf(1, '\n'); % end line for each distance
  %% plots for individual subjects
  if (length(distances) > 3)
  % speed profile using step speeds  
  step_speed_fig=figure('Renderer', 'painters', 'Position', [10 10 1200 600]);
  hold on
  for i =1:length(distances) % plot step speed vs time for all distances
    for j=1:length(step_speed_all_dist{distances(i)})
        plot(step_ff_all_dist{distances(i)}{j},step_speed_all_dist{distances(i)}{j},'Color',colors(i,:))
    end
  end
  xlabel('Time (s)')
  ylabel('Step speed (m/s)')
  ax = gca;
  ax.FontSize = 18;
  % save speed profile
  saveas(gcf, strcat(save_dir,'all-step-speed-intersect-shift') , 'png');
 
  % peak speed vs. distance 
  figure(speed_dist_fig)
  hold on
  peak_speed2 = squeeze(peak_speed(isubject,:,:));
  %plot(actual_dist(distances),peak_speed2(distances,:),'-ob');
  hold on
  xlabel('Distance');
  ylabel('Peak speed (m/s)');
  xlim([0 inf]) 
  ylim([0 inf])
  % add a fit line
  n_dist = length(distances);
  x_fit = zeros(n_dist+1,1); % include the point (0,0)
  y_fit = zeros(n_dist+1,1);
  x_fit(2:end) = actual_dist(distances);
  y_fit(2:end) = mean(peak_speed2(distances,:),2);
  fitfun = fittype( @(a,b,x) a*(1-exp(-x/b)) );
  try
  [fitted_curve,gof] = fit(x_fit,y_fit,fitfun,'StartPoint',[1.5 2.0]);
  coeffvals = coeffvalues(fitted_curve);
  x100 = 0:0.01:x_fit(end);
  %h=plot(x100,fitted_curve(x100),'LineWidth',1.2,'color','k');
  plot(x100,fitted_curve(x100),'Color',colors(isubject,:));
  eqnlabel = [num2str(coeffvals(1)) '*(1-exp(-x/' num2str(coeffvals(2)) '))'];
  %label(h,eqnlabel)
  catch
  disp('Problem with peak speed vs. distance fit line')
  disp(isubject)
  end
  saveas(gcf, strcat(save_dir,'speed-distance') , 'png');
  
  % Duration vs distance
  figure
  this_duration = squeeze(walk_duration(isubject,:,:));
  plot(actual_dist(distances),this_duration(distances,:),'-o');
  xlabel('Distance');
  ylabel('Walk duration (s)');
  xlim([0 inf]) 
  ylim([0 inf])
  hold on
  % fit line for this plot
  y_fit(2:end)=mean(this_duration(distances,:),2);
  x_fit(2:end) = actual_dist(distances);
  fitfun = fittype( @(a,b,c,x) x/a+b*(1-exp(-x/c)) );
  try 
  [fitted_curve,gof] = fit(x_fit,y_fit,fitfun,'StartPoint',[1.5 2.2 1.0]);
  coeffvals = coeffvalues(fitted_curve);
  x100 = 0:0.01:x_fit(end);
  h3 = plot(x100,fitted_curve(x100),'LineWidth',1.2,'color','k');
  eqnlabel = ['x/' num2str(coeffvals(1)) '+' num2str(coeffvals(2)) '*(1-exp(-x/' num2str(coeffvals(3)) '))'];
  label(h3,eqnlabel)
  %plot(actual_dist(distances),squeeze(mean(rise_times_all_sub(isubject,1,distances,:),4)),'k')
  %plot(actual_dist(distances),squeeze(mean(rise_times_all_sub(isubject,2,distances,:),4)),'b')
  %plot(actual_dist(distances),squeeze(mean(rise_times_all_sub(isubject,3,distances,:),4)),'r')
  catch
  disp('Problem with fit line for duration vs. distance')
  disp(isubject)
  end
  saveas(gcf, strcat(save_dir,'duration-distance') , 'png');
  
  speed_all_sub{isubject} = step_speed_all_dist;
  times_all_sub{isubject} = step_ff_all_dist;
  length_all_sub{isubject} = step_length_all_dist;
  freq_all_sub{isubject} = step_freq_all_dist;
  end % if length(distances) > 3
end % for isubject

if length(subjects) == 10 && length(distances) == 10
    trialsperdist = length(section)*length(trials);
    trialspersub = trialsperdist*length(distances);
  %% speed profiles for everyone together
%   avg_end_time = zeros(10,1);
%   for idist = 1:10
%     sum1 = 0;
%     ntrials = 0;
%     for isub = 1:10
%         for itrial = 1:trialsperdist
%           if  itrial <= length(times_all_sub{isub}{idist}) && ~isempty(cell2mat(times_all_sub{isub}{idist}(itrial)))
%             tmp=cell2mat(times_all_sub{isub}{idist}(itrial));
%             sum1 = sum1+tmp(end);
%             ntrials = ntrials+1;
%           end
%         end
%     end
%        avg_end_time(idist) = sum1/ntrials;
%   end
  
%   step_speed_all_fig=figure('Renderer', 'painters', 'Position', [10 10 1200 600]);
%   hold on
%   for idist=1:10 
%     % find most common number of steps for this distance
%     stepcount = zeros(1,length(subjects)*trialsperdist);
%     
%     for isub=1:10
%       stepcount((isub-1)*trialsperdist+1:(isub-1)*trialsperdist+trialsperdist) = cellfun(@length,speed_all_sub{isub}{idist});      
%     end
%     
%     % resample so all trials of a distance have equal number of steps
%     %min1 = min(stepcount(idist,:));
%     min1 = mode(stepcount);
%     if min1==0
%         min1=3;
%     end
%     resampled_trials = zeros(min1,length(subjects)*trialsperdist);
%     resampled_times = zeros(min1,length(subjects)*trialsperdist);
%     for isub=1:10
%       for itrial=1:trialsperdist
%           if itrial <= length(speed_all_sub{isub}{idist}) && ~isempty(cell2mat(speed_all_sub{isub}{idist}(itrial)))
%            resampled_trials(:,(isub-1)*trialsperdist+itrial) = resample(cell2mat(speed_all_sub{isub}{idist}(itrial)),min1,length(cell2mat(speed_all_sub{isub}{idist}(itrial))),0);
%            % resample the times
%            this_time = cell2mat(times_all_sub{isub}{idist}(itrial));
%            rescaled_time = resample(this_time,min1,length(this_time),0);
%            resampled_times(:,(isub-1)*trialsperdist+itrial) = rescaled_time/max(rescaled_time) * avg_end_time(idist);
%            % plot this trial
%            plot([0 this_time],[0 reshape(cell2mat(speed_all_sub{isub}{idist}(itrial)),size(this_time))],'Color',colors(idist,:));
%           end
%       end
%     end
% 
%     %plot(resampled_times,resampled_trials,'Color',colors(idist,:));
%     %plot(mean(resampled_times,2),mean(resampled_trials,2),'Color','black','LineWidth',2);
%     
%     avg_times{idist} = mean(resampled_times,2);
%     avg_speeds{idist} = mean(resampled_trials,2);
% end
% 
% 
% for i =1:10
%     plot(avg_times{i},avg_speeds{i},'Color','black','LineWidth',2);
% end
% 
% 
%  xlabel('Time (s)')
%  ylabel('Step speed (m/s)')
%  ax = gca;
%  ax.FontSize = 18;
    
        plot_speeds_all_sub(times_all_sub,speed_all_sub) ;
         %plot_speeds_all_sub(times_all_sub,length_all_sub);
         %ylabel('Step length')
          %plot_speeds_all_sub(times_all_sub,freq_all_sub);
          %ylabel('1/step time')

    
    %% make a fit line for peak speed vs distance for everyone together
    peak_speed_all=zeros(1,size(peak_speed,1)*size(peak_speed,2)*size(peak_speed,3));
    actual_dist_all = repmat(actual_dist,1,size(peak_speed,1)*size(peak_speed,3));
    for i=1:size(peak_speed,1)
       tmp = reshape(peak_speed(i,:,:)/mean(peak_speed(i,10,:)),1,size(peak_speed,2)*size(peak_speed,3));
       peak_speed_all((i-1)*length(tmp)+1:i*length(tmp)) = tmp;
    end
    figure
    %plot(actual_dist_all,peak_speed_all,'o')
    hold on
    fitfun = fittype( @(a,b,x) a*(1-exp(-x/b)) );
    [fitted_curve1,gof1] = fit(actual_dist_all',peak_speed_all',fitfun,'StartPoint',[1.0 2.0]);
    disp(fitted_curve1)
    coeffvals = coeffvalues(fitted_curve1);
    % use fitnlm for p-values
    ft34model = @(b,x) b(1)*(1-exp(-x/b(2)));
    beta0 = [1.0 2.0]; % beta0 = b??
    mdl1=fitnlm(actual_dist_all',peak_speed_all', ft34model, beta0);
    
    x100 = 0:0.01:actual_dist(end);
    h=plot(x100,fitted_curve1(x100),'LineWidth',1.2,'color','k');
    peak_speed_all2 = reshape(peak_speed_all,10,trialsperdist*10);
    % peak_speed_all, peak_speed_all2 = normalized
    %errorbar(actual_dist,fitted_curve1(actual_dist),std(peak_speed_all2'),'.k');
    % error bars based on normalized numbers
    errorbar(actual_dist,mean(peak_speed_all2'),std(peak_speed_all2'),'.k');
    eqnlabel = [num2str(coeffvals(1)) '*(1-exp(-x/' num2str(coeffvals(2)) '))'];
    disp(eqnlabel)
    %label(h,eqnlabel)
    xlabel('Distance (m)')
    ylabel('Peak step speed (normalized for each subject)')
    
    a2 = axes('YAxisLocation', 'Right');
    set(a2, 'color', 'none')
    set(a2, 'XTick', [])
    set(a2, 'YLim', [0 1.2*mean(mean(peak_speed(:,10,:),3))])
    ylabel('Walk duration (s)') 
    
    peak_speed_reshape = zeros(size(peak_speed,1),size(peak_speed,2)*size(peak_speed,3));
    for i=1:length(actual_dist)
      peak_speed_reshape(i,:) = reshape(peak_speed(:,i,:),1,size(peak_speed,1)*size(peak_speed,3)); 
    end
    
      figure(speed_dist_fig)
      plot(x100,fitted_curve1(x100)*mean(mean(peak_speed(:,10,:),3)),'LineWidth',1.2,'color','k');
      % error bars based on non-normalized numbers
      errorbar(actual_dist,mean(peak_speed_reshape'),std(peak_speed_reshape'),'.k');
      for i=1:10
        plot(actual_dist,squeeze(peak_speed(i,:,:)),'.','Color',colors(i,:))
      end
    
    %% plot walk durations for everyone together
    walk_duration_all=zeros(1,size(walk_duration,1)*size(walk_duration,2)*size(walk_duration,3));
    for i=1:size(walk_duration,1)
       tmp = reshape(walk_duration(i,:,:)/mean(walk_duration(i,10,:)),1,size(walk_duration,2)*size(walk_duration,3));
       walk_duration_all((i-1)*length(tmp)+1:i*length(tmp)) = tmp;
    end
    %rise_times_all_sub  10 3 10 8
    rise_times_reshape = zeros(3,size(rise_times_all_sub,3),size(rise_times_all_sub,1)*size(rise_times_all_sub,4));
    for i=1:size(rise_times_all_sub,1)
        for j=1:size(rise_times_all_sub,3)
            for k=1:size(rise_times_all_sub,4)
                rise_times_reshape(:,j,(i-1)*size(rise_times_all_sub,4)+k) = rise_times_all_sub(i,:,j,k) / walk_duration(i,j,k);
            end
        end
    end
    
    %rise_times1 = reshape(rise_times_reshape(1,:,:),1,800);
    %flat_times = reshape(rise_times_reshape(2,:,:) - rise_times_reshape(1,:,:),1,800);
    %fall_times = reshape(rise_times_reshape(3,:,:) - rise_times_reshape(2,:,:),1,800);
 
    figure % walking duration vs distance
    %plot(actual_dist_all,walk_duration_all,'o')
    hold on
    fitfun = fittype( @(a,b,c,x) x/a+b*(1-exp(-x/c)) );
    [fitted_curve,gof] = fit(actual_dist_all',walk_duration_all',fitfun,'StartPoint',[15 0.1 0.5]);
    coeffvals = coeffvalues(fitted_curve);
    
    % use fitnlm for p-values
    ft34model = @(b,x) x/b(1)+b(2)*(1-exp(-x/b(3)));
    beta0 = coeffvalues(fitted_curve); 
    mdl=fitnlm(actual_dist_all',walk_duration_all', ft34model, beta0);
    
    x100 = 0:0.01:actual_dist(end);
    h=plot(x100,fitted_curve(x100),'LineWidth',1.2,'color','k');
    walk_duration_all2 = reshape(walk_duration_all,10,trialsperdist*10);
    plot(actual_dist,mean(walk_duration_all2'),'ko')
    errorbar(actual_dist,mean(walk_duration_all2'),std(walk_duration_all2'),'.k');
    eqnlabel = ['x/' num2str(coeffvals(1)) '+' num2str(coeffvals(2)) '*(1-exp(-x/' num2str(coeffvals(3)) '))'];    
    disp(fitted_curve)
    %label(h,eqnlabel)
    xlabel('Distance (m)')
    ylabel('Walk duration (normalized for each subject)')
    
    %rise_times1(isnan(rise_times1))=0;
    %[fitobject1,gof1] = fit(actual_dist_all',rise_times1','poly1');
    %plot(x100,fitobject1(x100).*fitted_curve(x100),'k')
    
    %flat_times(isnan(flat_times))=0;
    %[fitobject2,gof2] = fit(actual_dist_all',flat_times','poly1');
    %plot(x100,(fitobject1(x100)+fitobject2(x100)).*fitted_curve(x100),'b')
    
    %fall_times(isnan(fall_times))=0;
    %[fitobject3,gof3] = fit(actual_dist_all',fall_times','poly1');
    %plot(x100,(fitobject1(x100)+fitobject2(x100)+fitobject3(x100)).*fitted_curve(x100),'r')
    
    %plot(actual_dist,mean(rise_times_reshape(1,:,:),3).*fitted_curve(actual_dist)','k')
    %plot(actual_dist,mean(rise_times_reshape(2,:,:),3).*fitted_curve(actual_dist)','b')
    %plot(actual_dist,mean(rise_times_reshape(3,:,:),3).*fitted_curve(actual_dist)','r')
    x = actual_dist;
    rise_times_sum = cumsum(rise_times_reshape);
    y1 = mean(rise_times_sum(1,:,:),3).*fitted_curve(actual_dist)';
    y2 = mean(rise_times_sum(2,:,:),3).*fitted_curve(actual_dist)';
    y3 = mean(rise_times_sum(3,:,:),3).*fitted_curve(actual_dist)';
    y4 = fitted_curve(actual_dist)';
    shade1 = [241/255 250/255 238/255];
    shade2 = [168/255 218/255 220/255];
    shade3 = [69/255 123/255 157/255];
    
     %p3 = patch([x fliplr(x)], [y3 fliplr(y4)], shade3);
     %p3.LineStyle = 'none';

     p2 = patch([x fliplr(x)], [y2 fliplr(y3)], shade1);
     p2.LineStyle = 'none';
     
     p1 = patch([x fliplr(x)], [y1 fliplr(y2)], shade2);
     p1.LineStyle = 'none';
     
     p0 = patch([x fliplr(x)], [zeros(size(x)) fliplr(y1)],shade3);
     p0.LineStyle = 'none';
     
         h=plot(x100,fitted_curve(x100),'LineWidth',1.2,'color','k');
    walk_duration_all2 = reshape(walk_duration_all,10,trialsperdist*10);
    plot(actual_dist,mean(walk_duration_all2'),'ko')
    errorbar(actual_dist,mean(walk_duration_all2'),std(walk_duration_all2'),'.k');
    a2 = axes('YAxisLocation', 'Right');
    set(a2, 'color', 'none')
    set(a2, 'XTick', [])
    set(a2, 'YLim', [0 1.2*mean(mean(walk_duration(:,10,:),3))])
    ylabel('Walk duration (s)') 
    
    red = [230/255 57/255 70/255];
    shade1 = [241/255 250/255 238/255];
    shade2 = [168/255 218/255 220/255];
    shade3 = [69/255 123/255 157/255];
    shade4 = [29/255 53/255 87/255];
    figure
    errorbar(actual_dist, mean(rise_times_reshape(1,:,:),3),std(rise_times_reshape(1,:,:),0,3),'DisplayName','Rise time','Color',red)
    hold on
    errorbar(actual_dist, mean(rise_times_reshape(2,:,:),3),std(rise_times_reshape(2,:,:),0,3),'DisplayName','Level time','Color',shade4)
    errorbar(actual_dist, mean(rise_times_reshape(3,:,:),3),std(rise_times_reshape(3,:,:),0,3),'DisplayName','Fall time','Color',shade3)
    legend('Location','northwest')
    xlabel('Distance (m)')
    ylabel('Pecentage of total walk time')
    
    
    % use this before saving a figure as pdf
    set(gcf,'Units','inches');
    screenposition = get(gcf,'Position');
    set(gcf,...
    'PaperPosition',[0 0 screenposition(3:4)],...
    'PaperSize',[screenposition(3:4)]);
    
end

      
      