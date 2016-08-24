function [] = ORB_SLAM2_Function( inputFile )

% This Function Opens the Custom ORB_SLAM2 LogFile We Made,
% parses through it and plots the evaluations that we are interested in

% Current Structure of ORB Log File

%(frameID) (frame TimeStamp) <only when tracking>
%(mapPoint ID) (angle) (octave) (x coord) (y coord) (response) (size) (repeats) <only when tracking>
%(Number of Inlier Matches) <only when tracking>
%(Time to Process Frame)
%(Reprojection Error Before Optimization) (Reprojection Error After Optimization) 
%("2" for noTracking OR 4x4 T_wc Matrix)

%(frameID) (frame TimeStamp) <only when tracking>
%(mapPoint ID) (angle) (octave) (x coord) (y coord) (response) (size) (repeats) <only when tracking>
%(Number of Inlier Matches) <only when tracking>
%(Time to Process Frame)
%(Reprojection Error Before Optimization) (Reprojection Error After Optimization) 
%("2" for noTracking OR 4x4 T_wc Matrix)

%...

LogInfo = strsplit(inputFile, {'_','.csv'},'CollapseDelimiters',true ); %how do I deal with csv at the end?
%LogInfo(2) = Dataset
%LogInfo(3) = Sequence
%LogInfo(4) = Velocity Model

logDir = strcat('ORB_SLAM2_Figures/', LogInfo(2), '/', LogInfo(3), '/', LogInfo(4), '_' ); 
Directory = logDir{1};
%% Open The .csv file and save it to an array
M = csvread(inputFile);


%% Parse through the array we just saved

% traverse until we get to when Tracking is succesful
% Save important information in arrays of the following formats

% matches (frameID, timestamp, Matches, How long to compute)
% features (frameID x featureID) with 1 indicating it exists
% repError (beforeOptimization, afterOptimization)


idx = 1;
frameIdx = 1;

% declare in advance to speed up
matches = zeros(3000,4);
features = zeros(4000,53000);
repError = zeros(3000,2);

while (idx < length(M) )
    if(  (M(idx,1)==0) && (M(idx,2)==0) )
        if( (M(idx+1,1)~=0) && (M(idx+1,2)~=0) )
            %the above conditions find when Feature tracking was succesful
            matches(frameIdx,1) = M(idx+1,1);   %store the frameID
            matches(frameIdx,2) = M(idx+1,2);   %store the timestamp
            
            %now step through each of the features
            idx = idx + 2;
            while ( M(idx,7) > 0)
                features(frameIdx,M(idx,1)+1) = 1;  %store that we found this
                idx = idx + 1;                      %feature at this frame
            end
            matches(frameIdx,3) = M(idx,1);     %store the total Matches
            matches(frameIdx,4) = M(idx + 1,1); %store how long to compute
            
            repError(frameIdx,1) = M(idx + 2,1);%store before Rep Error
            repError(frameIdx,2) = M(idx + 2,2);%store after Rep Error
            
            frameIdx = frameIdx + 1;
        end
    end
    idx = idx +1;
end

%% Process the Feature Tracks Matrix

% The Histogram is represented by H    
% H(x) = frequency of x 

[row, col] = size(features);
count = 0;
H = zeros(row,1);

%loop through the matrix col by col
%record each consecutive stream of 1s to the histogram
% (singles count)
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

%% Plot the Histogram of Feature Tracking Lengths



%only plot up to the last non-zero value
cleanH = find(H);
if( isempty(cleanH) )
    'Empty'
else
    last = cleanH(end);
    
    %figure();
    %plot(H(1:last));
%     bar(H(1:last));
%     title('Tracking Length Histogram');
%     xlabel('Tracking Length (in frames)');
%     ylabel('Frequency (# of features)');
% 
%     saveas(gcf, strcat(Directory,'Tracking_Histogram.png'))
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

%% Plot the Feature Matches

%only plot up to the last non-zero value
cleanMatches = find(matches(:,2));
if( isempty(cleanMatches) )
    'Empty'
else
    last = cleanMatches(end);
    time = matches(1:last,2);

    %Plot relative time instead of absolute
    time = time - time(1);

    figure();
    plot(time,matches(1:last,3));
    title('The number of tracked features');
    xlabel('Time (s)');
    ylabel('The number of tracked features');

    saveas(gcf, strcat(Directory, 'Feature_Matches.png') )
    %% Plot the Time to compute each frame

    figure();
    plot(time,matches(1:last,4) * 1000);
    title('Time to compute Frame');
    xlabel('Time (s)');
    ylabel('Time to compute Frame (ms)');

    saveas(gcf, strcat(Directory,'Computation_Time.png') )
    
    %% Plot the Reprojection Errors
    
    figure();
    %subplot(2,1,1);
    plot(time,repError(1:last,1));
    title('Average Reprojection Error');
    %title('Average Reprojection Error with Initial Pose From Velocity Model');
    xlabel('Time (s)');
    ylabel('Average Reprojection Error (pixels)');
    
    hold on;
    %subplot(2,1,2);
    plot(time,repError(1:last,2));
    %title('Average Reprojection Error After Optimization');
    xlabel('Time (s)');
    ylabel('Average Reprojection Error (pixels)');
    
    legend('Before Optimization','After Optimization');
    
    hold off;

    saveas(gcf, strcat(Directory,'Reprojection_Error.png') )
%% Now plot everything together
    figure();
    subplot(3,1,1);
    plot(time,matches(1:last,3));
    title('The number of tracked features');
    xlabel('Time (s)');
    ylabel('The number of tracked features');
    
    subplot(3,1,2);
    plot(time,matches(1:last,4) * 1000);
    title('Time to compute Frame');
    xlabel('Time (s)');
    ylabel('Time to compute Frame (ms)');
    
    subplot(3,1,3);
    plot(time,repError(1:last,1));
    title('Average Reprojection Error');
    xlabel('Time (s)');
    ylabel('Average Reprojection Error (pixels)');
    hold on;
    plot(time,repError(1:last,2));
    xlabel('Time (s)');
    ylabel('Average Reprojection Error (pixels)');
    legend('Before Optimization','After Optimization');
    hold off;

    saveas(gcf, strcat(Directory,'Matches_Time_Error.png') )


end



end

