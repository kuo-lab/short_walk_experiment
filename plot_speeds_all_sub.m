function plot_speeds_all_sub(times_all_sub,speed_all_sub)  
% times_all_sub, speed_all_sub = cell arrays
% speed_all_sub{isub}{idist}(itrial) contains speed profile for one trial
% plots each distance in a different color
% black line is average for all trials of that distance

numDist = length(speed_all_sub{1});
numSub = length(speed_all_sub);
colors = jet(numDist);
avg_end_time = zeros(numDist,1);
avg_end_time_longest = zeros(numSub,1);
ntrials = zeros(numDist,1);

max_sub_speed = zeros(numSub,1);
for isub = 1:numSub
    speed_this_sub = zeros(size(speed_all_sub{isub}{10}));
    for j=1:length(speed_this_sub)
      speed_this_sub(j) = max(speed_all_sub{isub}{10}{j});
    end
    max_sub_speed(isub) = mean(speed_this_sub);
end

for idist = 1:numDist
    sum1 = 0;
    ntrials(idist) = 0;
    for isub = 1:numSub
        for itrial = 1:length(times_all_sub{isub}{idist})
            tmp=cell2mat(times_all_sub{isub}{idist}(itrial));
            sum1 = sum1+tmp(end);
            ntrials(idist) = ntrials(idist)+1;
        end
    end
       avg_end_time(idist) = sum1/ntrials(idist);
end

for isub = 1:numSub
    sum1 = 0;
    for itrial = 1:length(times_all_sub{isub}{numDist})
       tmp=cell2mat(times_all_sub{isub}{numDist}(itrial)); 
       sum1 = sum1+tmp(end);
    end
    avg_end_time_longest(isub) = sum1/length(times_all_sub{isub}{numDist});
end
time_scale = avg_end_time(numDist)./avg_end_time_longest;


step_speed_all_fig=figure('Renderer', 'painters', 'Position', [10 10 1200 600]);
hold on
for idist=1:numDist 
    % find most common number of steps for this distance
    stepcount = zeros(1,ntrials(idist));
    
    ind = 1;
    for isub=1:numSub
      stepcount(ind:ind+length(speed_all_sub{isub}{idist})-1) = cellfun(@length,speed_all_sub{isub}{idist}); 
      ind = ind+length(speed_all_sub{isub}{idist});
    end
    
    % resample so all trials of a distance have equal number of steps
    %min1 = min(stepcount(idist,:));
    min1 = mode(stepcount);
    resampled_trials = zeros(min1,ntrials(idist));
    rescaled_trials = zeros(min1,ntrials(idist));
    resampled_times = zeros(min1,ntrials(idist));
    rescaled_times = zeros(min1,ntrials(idist));
    trialcount = 1;
    for isub=1:numSub
      for itrial=1:length(speed_all_sub{isub}{idist})
           resampled_trials(:,trialcount) = resample(cell2mat(speed_all_sub{isub}{idist}(itrial)),min1,length(cell2mat(speed_all_sub{isub}{idist}(itrial))),0);
           rescaled_trials(:,trialcount) = resampled_trials(:,trialcount) ./ max_sub_speed(isub);
           % resample the times
           this_time = cell2mat(times_all_sub{isub}{idist}(itrial));
           resampled_times(:,trialcount) = resample(this_time,min1,length(this_time),0);
           % and rescale them to the average time for this distance
           rescaled_times(:,trialcount) = resampled_times(:,trialcount) * time_scale(isub);
           trialcount = trialcount+1;
           % plot this trial
           plot(this_time,reshape(cell2mat(speed_all_sub{isub}{idist}(itrial)),size(this_time)),'Color',colors(idist,:));
           % use line below for normalized version
           % also change what goes in avg_times and avg_speeds
           %plot(rescaled_times,rescaled_trials,'Color',colors(idist,:));
      end
    end

    %plot(resampled_times,rescaled_trials,'Color',colors(idist,:));
    %plot(mean(resampled_times,2),mean(rescaled_trials,2),'Color','black','LineWidth',2);
    
    % used rescaled_times and rescaled_trials for normalized version
    avg_times{idist} = mean(resampled_times,2);
    avg_speeds{idist} = mean(resampled_trials,2);
    %avg_times{idist} = mean(rescaled_times,2);
    %avg_speeds{idist} = mean(rescaled_trials,2);
end


for i =1:10
    plot(avg_times{i},avg_speeds{i},'Color','black','LineWidth',2);
end
    

 xlabel('Time (s)')
 ylabel('Step speed (m/s)')
 ax = gca;
 ax.FontSize = 18;

 % use this if plotting normalized speeds
% check that left y-axis goes 0 to 1.2
    %a2 = axes('YAxisLocation', 'Right');
    %set(a2, 'color', 'none')
    %set(a2, 'XTick', [])
    %set(a2, 'YLim', [0 1.2*mean(mean(max_sub_speed))])
    %set(a2, 'FontSize',18)
    %ylabel('Step speed (m/s)')
 
 
end
 
 