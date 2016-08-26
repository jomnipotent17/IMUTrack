function [] = Compare3ORB( file1, file2, file3 )
%function [featAvg, featSTD, timeAvg, timeSTD] = Compare3ORB( file1, file2, file3 )
%Given Three ORB Log Files, this function calls the modular function
% and then plots the data against one another

% file1 = 'ORB_Myung_Aerial_myungIMU.csv';
% file2 = 'ORB_Myung_Aerial_noIMU.csv';
% file3 = 'ORB_Myung_Aerial_velocitySwitch.csv';

% file1 = 'ORB_Myung_Desk_myungIMU.csv';
% file2 = 'ORB_Myung_Desk_noIMU.csv';
% file3 = 'ORB_Myung_Desk_velocitySwitch.csv';

% file1 = 'ORB_EuRoC_V203_myungIMU.csv';
% file2 = 'ORB_EuRoC_V203_noIMU.csv';
% file3 = 'ORB_EuRoC_V203_velocitySwitch.csv';

% file1 = 'ORB_EuRoC_MH03_myungIMU.csv';
% file2 = 'ORB_EuRoC_MH03_noIMU.csv';
% file3 = 'ORB_EuRoC_MH03_velocitySwitch.csv';

load('imudata.mat');
load('kltFrameTimes.mat');
%% Call the helper function
[frameData1, Hist1] = Modular_ORB_Function(file1);
[frameData2, Hist2] = Modular_ORB_Function(file2);
[frameData3, Hist3] = Modular_ORB_Function(file3);

%% Edge Cases to deal with tracking loss
if(isempty(frameData1) && isempty(frameData2) && isempty(frameData3) )
    %'No Features Tracked For All'
    return
elseif(isempty(frameData1) || isempty(frameData2) || isempty(frameData3))
    %'At least one method has No Features Tracked'
    if(isempty(frameData1))
        if(~isempty(frameData2))
            %copy frameData2 into 1 and set things to zero
            frameData1 = CopyFrameDataORB(frameData2);
        else
            %otherwise copy frameData3 into 1
            frameData1 = CopyFrameDataORB(frameData3);
        end
    elseif(isempty(frameData2))
        if(~isempty(frameData1))
            %copy frameData1 into 2 and set things to zero
            frameData2 = CopyFrameDataORB(frameData1);
        else
            %otherwise copy frameData3 into 2
            frameData2 = CopyFrameDataORB(frameData3);
        end 
    else
        if(~isempty(frameData1))
            %copy 1 into 3
            frameData3 = CopyFrameDataORB(frameData1);
        else
            %copy 2 into 3
            frameData3 = CopyFrameDataORB(frameData2);
        end
    end
end


%% Parse the file names
LogInfo1 = strsplit(file1, {'_','.csv'},'CollapseDelimiters',true );
model1 = LogInfo1{4};

LogInfo2 = strsplit(file2, {'_','.csv'},'CollapseDelimiters',true );
model2 = LogInfo2{4};

LogInfo3 = strsplit(file3, {'_','.csv'},'CollapseDelimiters',true );
model3 = LogInfo3{4};

%%
logDir = strcat('ORB_SLAM2_Figures/', LogInfo1(2), '/', LogInfo1(3), '/');
Directory = logDir{1};

%%
%make all time relative and start at 0. 
time(1) = min(frameData1(1,2), min(frameData2(1,2),frameData3(1,2)));
%%
frameData1(:,2) = frameData1(:,2) - time(1);
frameData2(:,2) = frameData2(:,2) - time(1);
frameData3(:,2) = frameData3(:,2) - time(1);

%% Prepare the imu data based on the sequence
% LogInfo2 is used but the others could be used as well

if( strcmp(LogInfo2{3},'Aerial') )
    imu = imu7(1968:3437,:,:,:,:,:,:);
    %'Aerial'
    imu(:,1) = imu(:,1)/1000;   %convert to seconds
    %align with image data via the last frame (off by a ms or too, but w/e)
    imu(:,1) = imu(:,1) - imu(end,1) + frameData1(end,2);     
elseif( strcmp(LogInfo2{3},'Desk') )
    imu = imu6(5933:8684,:,:,:,:,:,:);
    imu(:,1) = imu(:,1)/1000;   %convert to seconds
    %align with image data via the last frame (off by a ms or too, but w/e)
    imu(:,1) = imu(:,1) - imu(end,1) + frameData1(end,2); 
    %'Desk'
elseif(strcmp(LogInfo2{3},'MH01') )
    %'MH01'
    %imu = imu1(100:end,:,:,:,:,:,:);
    imu = imu1;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH02') )
    %'MH02'
    imu = imu2;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH03') )
    %'MH03'
    imu = imu3;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH04') )
    %'MH04'
    imu = imu4;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH05') )
    %'MH05'
    imu = imu5;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'V101') )
    %'V101'
    imu = imu8;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'V102') )
    %'V102'
    imu = imu9;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'V103') )
    %'V103'
    imu = imu10;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'V201') )
    %'V201'
    imu = imu11;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'V202') )
    %'V202'
    imu = imu12;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'V203') )
    %'V203'
    imu = imu13;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
else
    %'Error'
end

%%
%only plot imu data > 0 (relative time)

A = find(imu(:,1) > 0);
start = A(1);

imu = imu(start:end,:,:,:,:,:,:);



%% Plot the frameData

% Plot the matches
figure();
    subplot(2,1,1);
    plot( frameData1(:,2)  ,frameData1(:,3));
    title('The number of tracked features');
    xlabel('Time (s)');
    ylabel('The number of tracked features');
    hold on;
    plot( frameData2(:,2), frameData2(:,3));
    plot( frameData3(:,2), frameData3(:,3));
    hold off;
    legend(model1,model2,model3);
    
    subplot(2,1,2);
    plot( imu(:,1),imu(:,2))
    hold on;
    plot( imu(:,1),imu(:,3))
    plot( imu(:,1),imu(:,4))
    title('Angular Velocity');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    hold off;
    legend('w_x','w_y','w_z');
    saveas(gcf, strcat(Directory,'feature_matches.png') )
    saveas(gcf, strcat(Directory,'feature_matches.fig') )
  
    
% Plot the Length to compute each frame
figure();
    plot( frameData1(:,2), frameData1(:,4) * 1000);
    title('Time to compute Frame');
    xlabel('Time (s)');
    ylabel('Time to compute Frame (ms)');
    hold on;
    plot( frameData2(:,2), frameData2(:,4) * 1000);
    plot( frameData3(:,2), frameData3(:,4) * 1000);
    hold off;
    legend(model1, model2, model3);
    
    saveas(gcf, strcat(Directory,'computation_time.png') )
    saveas(gcf, strcat(Directory,'computation_time.fig') )
    
%% Plot the Reprojection Errors

% Reprojection Error vs. Time (No Optimization)
figure();
    plot( frameData1(:,2), frameData1(:,5));
    hold on;
    plot( frameData2(:,2), frameData2(:,5));
    plot( frameData3(:,2), frameData3(:,5));
    
    title('Average Reprojection Error Before Optimization');
    xlabel('Time (s)');
    ylabel('Average Reprojection Error');
    legend(model1,model2,model3);
    hold off;
    
    saveas(gcf, strcat(Directory,'Error_Before.png') )
    saveas(gcf, strcat(Directory,'Error_Before.fig') )
    
% Reprojection Error vs. Time (with Optimization)
figure();
    plot( frameData1(:,2), frameData1(:,6));
    hold on;
    plot( frameData2(:,2), frameData2(:,6));
    plot( frameData3(:,2), frameData3(:,6));
    
    title('Average Reprojection Error After Optimization');
    xlabel('Time (s)');
    ylabel('Average Reprojection Error');
    legend(model1,model2,model3);
    hold off;
    
    saveas(gcf, strcat(Directory,'Error_After.png') )
    saveas(gcf, strcat(Directory,'Error_After.fig') )
%% Plot the Histogram
%Group the histogram into blocks
[Hist1B, H1x] = Histogram_Blocker( Hist1, 10);
[Hist2B, H2x] = Histogram_Blocker( Hist2, 10);
[Hist3B, H3x] = Histogram_Blocker( Hist3, 10);

%plot it
figure();
    len = max(length(Hist1B), max(length(Hist2B),length(Hist3B)) );
    HistPlot = zeros(3,len);
    for i = 1:length(Hist1B)
       HistPlot(1,i) = Hist1B(i); 
    end
    for i = 1:length(Hist2B)
       HistPlot(2,i) = Hist2B(i); 
    end
    for i = 1:length(Hist3B)
       HistPlot(3,i) = Hist3B(i); 
    end
    
    HistPlot = transpose(HistPlot);
    if(length(H1x) > max(length(H2x),length(H3x) ) )
        Hx = H1x;
    elseif(length(H2x) > length(H3x) )
        Hx = H2x;
    else
        Hx = H3x;
    end

    bar(Hx, HistPlot); 
    title('Tracking Length Histogram');
    xlabel('Tracking Length (in frames)');
    ylabel('Frequency (# of features)');
    set(gca,'YScale','log');
    legend(model1,model2,model3);
    saveas(gcf, strcat(Directory,'histogram.png') )
    saveas(gcf, strcat(Directory,'histogram.fig') )

%% Obtain the average/std for feature track % and computation time
% % featAvg = [noIMU myung vel ...]
% % featSTD = [noIMU myung vel ...]
% % timeAvg = [noIMU myung vel ...]
% % timeSTD = [noIMU myung vel ...]    
% 
% featAvg = zeros(1,3);
% featSTD = zeros(1,3);
% timeAvg = zeros(1,3);
% timeSTD = zeros(1,3);
% 
% %Note the value 300 below is the targeted # of features and CAN change
% %as of writing this we are using 300, but if that changes, change this too
% %featAvg
% featAvg(1) = mean( frameData1(:,3) / 300);
% featAvg(2) = mean( frameData2(:,3) / 300);
% featAvg(3) = mean( frameData3(:,3) / 300);
% %featSTD
% featSTD(1) = std(  frameData1(:,3) / 300);
% featSTD(2) = std(  frameData2(:,3) / 300);
% featSTD(3) = std(  frameData3(:,3) / 300);
% %timeAvg
% timeAvg(1) = mean( frameData1(:,4) * 1000);
% timeAvg(2) = mean( frameData2(:,4) * 1000);
% timeAvg(3) = mean( frameData3(:,4) * 1000);
% %timeSTD
% timeSTD(1) = std(  frameData1(:,4) * 1000);
% timeSTD(2) = std(  frameData2(:,4) * 1000);
% timeSTD(3) = std(  frameData3(:,4) * 1000);

end