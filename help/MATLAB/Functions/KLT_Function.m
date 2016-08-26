function [] = KLT_Function( inputFile )
% This Function Opens the Custom KLT LogFile We Made,
% parses through it and plots the evaluations that we are interested in

% Current Structure of KLT Log File

%(frame) (timestamp hard-coded to 0, don’t know why they did )
%(stat.n_new) (stat.n_success) (stat.n_total)
%(stat.time_select*1e3) (stat.time_track*1e3)
%(time to track t3-t0 actual track time, others are 0)  
%(listID.size()  # of features)
%(id) (m_x)  (m_y)  (m_nx) (m_ny) (ErrorB4) (ErrorAfter)
%(id) (m_x)  (m_y)  (m_nx) (m_ny) (ErrorB4) (ErrorAfter)
%(id) (m_x)  (m_y)  (m_nx) (m_ny) (ErrorB4) (ErrorAfter)
%...

%inputFile = 'log_EuRoC_MH01_IMU_150.csv';

LogInfo = strsplit(inputFile, {'_','.csv'},'CollapseDelimiters',true ); %how do I deal with csv at the end?
%LogInfo(2) = Dataset
%LogInfo(3) = Sequence
%LogInfo(4) = Velocity Model
%LogInfo(5) = Feature Number

logDir = strcat('KLT_Figures/', LogInfo(2), '/', LogInfo(3), '/',  LogInfo(5), '/',LogInfo(4), '_' ); 
Directory = logDir{1};


%% Open The .csv file and save it to an array
M = csvread(inputFile);

%%

% matches (frameID,  Matches, How long to compute)
% features (frameID x featureID) with 1 indicating it exists
% Error (beforeOptimization, after)
idx = 1;
frameIdx = 1;

% try declaring in advance to speed up
matches = zeros(3000,3);
features = zeros(4500,35000);
Error = zeros(3000,2);

while (idx < length(M) )
    matches(frameIdx,1) = M(idx,1);
    matches(frameIdx,2) = M(idx+1,2);
    matches(frameIdx,3) = M(idx+3,1);
    count = M(idx+4,1);
    idx = idx + 5;
    BeforeErrorSum = 0;
    AfterErrorSum = 0;
    for i = 1:count
        features(frameIdx,M(idx,1)+1 ) = 1; %adjust to make index start @1
        BeforeErrorSum = BeforeErrorSum + M(idx,6);
        AfterErrorSum = AfterErrorSum + M(idx,7);
        idx = idx + 1;
    end
    Error(frameIdx,1) = BeforeErrorSum / count;
    Error(frameIdx,2) = AfterErrorSum / count;
    frameIdx = frameIdx + 1;
end 
    
%% Process the feature tracks
% Histogram is represented by H    H(x) = frequency of x 
[row, col] = size(features);

count = 0;
H = zeros(row,1);

for c = 1:col
    for r = 1:row
        if(features(r,c))
            count = count + 1;
        elseif (count ~= 0)
            H(count) = H(count) + 1;
            count = 0;
        end
    end
    count = 0;
end

%%
% Plot the Histogram


cleanH = find(H);
if(isempty(cleanH))
    'Empty'
else
    last = cleanH(end);

%     figure();
%     plot(H(1:last));
%     title('Tracking Length Histogram');
%     xlabel('Tracking Length (frame)');
%     ylabel('Frequency (# of features)');
    
    %% Group the Histogram into regions and plot
    H_ = H(1:last);

    block_size = 10;
    sum = 0;
    j=1;
    for i = 1:last
        sum = sum + H_(i);
        if(mod(i,block_size) == 0 )
            H_size(j) = sum;
            sum = 0;
            j = j + 1;
        end
    end
    H_size(j) = sum;

    %only plot up to the last non-zero value
    cleanH_size = find(H_size);
    last = cleanH_size(end);
%%
   for i = 1:last
       x_val(i) = i * 10;
   end

    figure();
    %bar(x_val, H_size);
    bar(x_val, H_size(1:last));
    title('Tracking Length Histogram');
    xlabel('Tracking Length (in frames)');
    ylabel('Frequency (# of features)');
    ax = gca;
    %ax.XTick = [0 10 20 30 40 50 60];
    %ax.XTick = ax.XTick * 10   
    set(gca,'YScale','log');
    

    saveas(gcf, strcat(Directory,'Tracking_Histogram.png') )
end
    

%%
% Plot the Feature Matches
cleanMatches = find(matches(:,1));
if( isempty(cleanMatches) )
    'Empty'
else
    last = cleanMatches(end);
    time = matches(1:last,1);

    figure();
    plot(time,matches(1:last,2));
    title('The number of tracked features');
    xlabel('Frame');
    ylabel('The number of tracked features');
    saveas(gcf, strcat(Directory, 'Feature_Matches.png') )
    
    % Plot the Length to compute each frame
    figure();
    plot(time,matches(1:last,3) * 1000);
    title('Time to compute Frame');
    xlabel('Frame');
    ylabel('Time to compute Frame (ms)');
    saveas(gcf, strcat(Directory,'Computation_Time.png') );
    
%% Plot the Reprojection Errors
    
    figure();
    %subplot(2,1,1);
    plot(time,Error(1:last,1));
    title('Average rms Error');
    %title('Average Reprojection Error with Initial Pose From Velocity Model');
    xlabel('Frame');
    %ylabel('Average rms Error');
    
    hold on;
    %subplot(2,1,2);
    plot(time,Error(1:last,2));
    %title('Average Reprojection Error After Optimization');
    %xlabel('Frame');
    ylabel('Average rms Error');
    
    legend('Before Optimization','After Optimization');
    
    hold off;

    saveas(gcf, strcat(Directory,'rms_Error.png') )
end

end

