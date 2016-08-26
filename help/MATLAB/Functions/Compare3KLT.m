function [featAvg, featSTD, timeAvg, timeSTD] = Compare3KLT( file1, file2, file3 )

%Given Three KLT Log Files, this function calls the modular function
% and then plots the data against one another

% file1 = 'KLT_Myung_Aerial_myungIMU.csv';
% file2 = 'KLT_Myung_Aerial_noIMU.csv';
% file3 = 'KLT_Myung_Aerial_velocitySwitch.csv';

% file1 = 'KLT_Myung_Desk_myungIMU.csv';
% file2 = 'KLT_Myung_Desk_noIMU.csv';
% file3 = 'KLT_Myung_Desk_velocitySwitch.csv';
% 
% file1 = 'KLT_EuRoC_V203_myungIMU.csv';
% file2 = 'KLT_EuRoC_V203_noIMU.csv';
% file3 = 'KLT_EuRoC_V203_velocitySwitch.csv';

% file1 = 'KLT_EuRoC_MH03_myungIMU.csv';
% file2 = 'KLT_EuRoC_MH03_noIMU.csv';
% file3 = 'KLT_EuRoC_MH03_velocitySwitch.csv';

load('imudata.mat');
load('kltFrameTimes.mat');
%% Call the helper function
[frameData1, Hist1] = Modular_KLT_Function(file1);
[frameData2, Hist2] = Modular_KLT_Function(file2);
[frameData3, Hist3] = Modular_KLT_Function(file3);


%% Parse the file names
LogInfo1 = strsplit(file1, {'_','.csv'},'CollapseDelimiters',true );
model1 = LogInfo1{4};

LogInfo2 = strsplit(file2, {'_','.csv'},'CollapseDelimiters',true );
model2 = LogInfo2{4};

LogInfo3 = strsplit(file3, {'_','.csv'},'CollapseDelimiters',true );
model3 = LogInfo3{4};

%%
logDir = strcat('KLT_Figures/', LogInfo1(2), '/', LogInfo1(3), '/');
Directory = logDir{1}

%%

if( strcmp(LogInfo2{3},'Aerial') )
    time = aerial_time;
    imu = imu7(1969:3436,:,:,:,:,:,:);
    'Aerial'
elseif( strcmp(LogInfo2{3},'Desk') )
    time = desk_time;
    imu = imu6(5934:8683,:,:,:,:,:,:);
    'Desk'
elseif(strcmp(LogInfo2{3},'MH01') )
    'MH01'
    imu = imu1;
    imu(:,1) = imu(:,1) / 1000000;
    time = mh01_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'MH02') )
    'MH02'
    imu = imu2;
    imu(:,1) = imu(:,1) / 1000000;
    time = mh02_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'MH03') )
    'MH03'
    imu = imu3;
    imu(:,1) = imu(:,1) / 1000000;
    time = mh03_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'MH04') )
    'MH04'
    imu = imu4;
    imu(:,1) = imu(:,1) / 1000000;
    time = mh04_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'MH05') )
    'MH05'
    imu = imu5;
    imu(:,1) = imu(:,1) / 1000000;
    time = mh05_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'V101') )
    'V101'
    imu = imu8;
    imu(:,1) = imu(:,1) / 1000000;
    time = v101_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'V102') )
    'V102'
    imu = imu9;
    imu(:,1) = imu(:,1) / 1000000;
    time = v102_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'V103') )
    'V103'
    imu = imu10;
    imu(:,1) = imu(:,1) / 1000000;
    time = v103_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'V201') )
    'V201'
    imu = imu11;
    imu(:,1) = imu(:,1) / 1000000;
    time = v201_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'V202') )
    'V202'
    imu = imu12;
    imu(:,1) = imu(:,1) / 1000000;
    time = v202_time;
    time = time / 1000000;
elseif(strcmp(LogInfo2{3},'V203') )
    'V203'
    imu = imu13;
    imu(:,1) = imu(:,1) / 1000000;
    time = v203_time;
    time = time / 1000000;
else
    'Error'
end

%% Plot the frameData

% Plot the matches
figure();
    subplot(2,1,1);
    plot( (time - time(1))/1000 ,frameData1(:,2));
    title('The number of tracked features');
    xlabel('Time (s)');
    ylabel('The number of tracked features');
    hold on;
    plot( (time - time(1))/1000 ,frameData2(:,2));
    plot( (time - time(1))/1000 ,frameData3(:,2));
    hold off;
    legend(model1,model2,model3);
    
    subplot(2,1,2);
    plot((imu(:,1) - time(1))/1000 ,imu(:,2))
    hold on;
    plot((imu(:,1) - time(1))/1000,imu(:,3))
    plot((imu(:,1) - time(1))/1000,imu(:,4))
    title('Angular Velocity');
    xlabel('Time (s)');
    ylabel('(rad/s)');
    hold off;
    legend('w_x','w_y','w_z');
    saveas(gcf, strcat(Directory,'feature_matches.png') )
    saveas(gcf, strcat(Directory,'feature_matches.fig') )
  
    
% Plot the Length to compute each frame
figure();
    plot( (time - time(1))/1000, frameData1(:,3) * 1000);
    title('Time to compute Frame');
    xlabel('Time (s)');
    ylabel('Time to compute Frame (ms)');
    hold on;
    plot( (time - time(1))/1000, frameData2(:,3) * 1000);
    plot( (time - time(1))/1000, frameData3(:,3) * 1000);
    hold off;
    legend(model1, model2,model3);
  
    saveas(gcf, strcat(Directory,'computation_time.png') )
    saveas(gcf, strcat(Directory,'computation_time.fig') )
%% Plot the rms Errors
%Error Before
figure();
    plot( (time - time(1))/1000, frameData1(:,4));
    hold on;
    plot( (time - time(1))/1000, frameData2(:,4));
    plot( (time - time(1))/1000, frameData3(:,4));
    title('Average rms Error Before Optimization');
    xlabel('Time (s)');
    ylabel('Average rms Error');
    legend(model1,model2,model3);
    hold off;
    saveas(gcf, strcat(Directory,'Error_Before.png') )
    saveas(gcf, strcat(Directory,'Error_Before.png') )

%Error After
figure();
    plot( (time - time(1))/1000, frameData1(:,5));
    hold on;
    plot( (time - time(1))/1000, frameData2(:,5));
    plot( (time - time(1))/1000, frameData3(:,5));
    title('Average rms Error After Optimization');
    xlabel('Time (s)');
    ylabel('Average rms Error');
    legend(model1,model2,model3);
    hold off; 
    saveas(gcf, strcat(Directory,'Error_After.png') )
    saveas(gcf, strcat(Directory,'Error_After.png') )
    
%% Plot the Histogram
%Group the histogram into blocks
[Hist1B, H1x] = Histogram_Blocker( Hist1, 10);
[Hist2B, H2x] = Histogram_Blocker( Hist2, 10);
[Hist3B, H3x] = Histogram_Blocker( Hist3, 10);

%plot it
figure();
    len = max(length(Hist1B),max(length(Hist2B),length(Hist3B)));
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
% featAvg = [noIMU myung vel ...]
% featSTD = [noIMU myung vel ...]
% timeAvg = [noIMU myung vel ...]
% timeSTD = [noIMU myung vel ...]    

featAvg = zeros(1,3);
featSTD = zeros(1,3);
timeAvg = zeros(1,3);
timeSTD = zeros(1,3);

%Note the value 300 below is the targeted # of features and CAN change
%as of writing this we are using 300, but if that changes, change this too
%featAvg
featAvg(1) = mean( frameData1(:,2) / 300);
featAvg(2) = mean( frameData2(:,2) / 300);
featAvg(3) = mean( frameData3(:,2) / 300);
%featSTD
featSTD(1) = std(  frameData1(:,2) / 300);
featSTD(2) = std(  frameData2(:,2) / 300);
featSTD(3) = std(  frameData3(:,2) / 300);
%timeAvg
timeAvg(1) = mean( frameData1(:,3) * 1000);
timeAvg(2) = mean( frameData2(:,3) * 1000);
timeAvg(3) = mean( frameData3(:,3) * 1000);
%timeSTD
timeSTD(1) = std(  frameData1(:,3) * 1000);
timeSTD(2) = std(  frameData2(:,3) * 1000);
timeSTD(3) = std(  frameData3(:,3) * 1000);

    
end



