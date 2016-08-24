function [] = Compare2ORB( file1, file2 )
%Given Two ORB Log Files, this function calls the modular function
% and then plots the data against one another

% file1 = 'log_Myung_Desk_constant.csv';
% file2 = 'log_Myung_Desk_imuAided.csv';

% file1 = 'log_Myung_Aerial_constant.csv';
% file2 = 'log_Myung_Aerial_newMethod.csv';

file1 = 'log_EuRoC_MH01_constant.csv';
file2 = 'log_EuRoC_MH01_imuAided.csv';

% file1 = 'log_EuRoC_MH01_constant.csv';
% file2 = 'log_EuRoC_MH01_newMethod.csv';

% file1 = 'log_EuRoC_MH05_constant.csv';
% file2 = 'log_EuRoC_MH05_newMethod.csv';

% file1 = 'log_Myung_Desk_constant.csv';
% file2 = 'log_Myung_Desk_newMethod.csv';

load('imudata.mat');
load('kltFrameTimes.mat');
%% Call the helper function
[frameData1, Hist1] = Modular_ORB_Function(file1);
[frameData2, Hist2] = Modular_ORB_Function(file2);

if(isempty(frameData1) || isempty(frameData2) )
    'No Features Tracked'
    return
end


%% Parse the file names
LogInfo1 = strsplit(file1, {'_','.csv'},'CollapseDelimiters',true );
model1 = LogInfo1{4};

LogInfo2 = strsplit(file2, {'_','.csv'},'CollapseDelimiters',true );
model2 = LogInfo2{4};

%%
logDir = strcat('ORB_SLAM2_Figures/', LogInfo1(2), '/', LogInfo1(3), '/' , LogInfo1(4), '_vs_', LogInfo2(4), '_');
Directory = logDir{1}

%%
%make all time relative and start at 0. 
time(1) = min(frameData1(1,2),frameData2(1,2));
frameData1(:,2) = frameData1(:,2) - time(1);
frameData2(:,2) = frameData2(:,2) - time(1);

%%
if( strcmp(LogInfo2{3},'Aerial') )
    imu = imu7(1968:3437,:,:,:,:,:,:);
    'Aerial'
    imu(:,1) = imu(:,1)/1000;   %convert to seconds
    %align with image data via the last frame (off by a ms or too, but w/e)
    imu(:,1) = imu(:,1) - imu(end,1) + frameData1(end,2);     
elseif( strcmp(LogInfo2{3},'Desk') )
    imu = imu6(5933:8684,:,:,:,:,:,:);
    imu(:,1) = imu(:,1)/1000;   %convert to seconds
    %align with image data via the last frame (off by a ms or too, but w/e)
    imu(:,1) = imu(:,1) - imu(end,1) + frameData1(end,2); 
    'Desk'
elseif(strcmp(LogInfo2{3},'MH01') )
    'MH01'
    %imu = imu1(100:end,:,:,:,:,:,:);
    imu = imu1;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH02') )
    'MH02'
    imu = imu2;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH03') )
    'MH03'
    imu = imu3;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH04') )
    'MH04'
    imu = imu4;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
elseif(strcmp(LogInfo2{3},'MH05') )
    'MH05'
    imu = imu5;
    imu(:,1) = imu(:,1) / 1000000000;
    imu(:,1) = imu(:,1) - time(1);
else
    'Error'
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
    plot( frameData2(:,2) ,frameData2(:,3));
    hold off;
    legend(model1,model2);
    
    subplot(2,1,2);
    plot( imu(:,1),imu(:,2))
    hold on;
    plot( imu(:,1),imu(:,3))
    plot( imu(:,1),imu(:,4))
    title('Angular Velocity');
    xlabel('Time (s)');
    ylabel('(rad/s)');
    hold off;
    legend('w_x','w_y','w_z');
    saveas(gcf, strcat(Directory,'feature_matches.png') )
  
    
% Plot the Length to compute each frame
figure();
    subplot(2,1,1);
    plot( frameData1(:,2), frameData1(:,4) * 1000);
    title('Time to compute Frame');
    xlabel('Time (s)');
    ylabel('Time to compute Frame (ms)');
    hold on;
    plot( frameData2(:,2), frameData2(:,4) * 1000);
    hold off;
    legend(model1, model2);
    
    subplot(2,1,2);
    plot( imu(:,1),imu(:,2))
    hold on;
    plot( imu(:,1),imu(:,3))
    plot( imu(:,1),imu(:,4))
    title('Angular Velocity');
    xlabel('Time (s)');
    ylabel('(rad/s)');
    hold off;
    legend('w_x','w_y','w_z');
    saveas(gcf, strcat(Directory,'computation_time.png') )
    
% Plot the rms Errors
figure();
    subplot(3,1,1);
    plot( frameData1(:,2), frameData1(:,5));
    hold on;
    plot( frameData1(:,2), frameData1(:,6));
    title(strcat(model1,' Average rms Error'));
    xlabel('Time (s)');
    ylabel('Average rms Error');
    legend('Before Optimization','After Optimization');
    hold off;
    
    subplot(3,1,2);
    plot( frameData2(:,2), frameData2(:,5));
    hold on;
    plot( frameData2(:,2), frameData2(:,6));
    title(strcat(model2,' Average rms Error'));
    xlabel('Time (s)');
    ylabel('Average rms Error');
    legend('Before Optimization','After Optimization');
    hold off;
    
    subplot(3,1,3);
    plot( imu(:,1),imu(:,2))
    hold on;
    plot( imu(:,1),imu(:,3))
    plot( imu(:,1),imu(:,4))
    title('Angular Velocity');
    xlabel('Time (s)');
    ylabel('(rad/s)');
    hold off;
    legend('w_x','w_y','w_z');
    saveas(gcf, strcat(Directory,'rms_Error.png') )
%% Plot the Histogram
%Group the histogram into blocks
[Hist1B, H1x] = Histogram_Blocker( Hist1, 10);
[Hist2B, H2x] = Histogram_Blocker( Hist2, 10);

%plot it
figure();
    len = max(length(Hist1B),length(Hist2B));
    HistPlot = zeros(2,len);
    for i = 1:length(Hist1B)
       HistPlot(1,i) = Hist1B(i); 
    end
    for i = 1:length(Hist2B)
       HistPlot(2,i) = Hist2B(i); 
    end
    
    HistPlot = transpose(HistPlot);
    if(length(H1x) > length(H2x) )
        Hx = H1x;
    else
        Hx = H2x;
    end

    bar(Hx, HistPlot); 
    title('Tracking Length Histogram');
    xlabel('Tracking Length (in frames)');
    ylabel('Frequency (# of features)');
    set(gca,'YScale','log');
    legend(model1,model2);
    saveas(gcf, strcat(Directory,'histogram.png') )

end

