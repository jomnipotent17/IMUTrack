% This is the highest Level Script that calls all of the other scripts and
% functions for parsing and plotting the experimental results from Both
% ORB_SLAM and KLT.

% clear all
% close all
% clc



%% ORB_SLAM2_Parser and Plotter
%414 seconds
%Seq = {'EuRoC_MH01';'EuRoC_MH02';'EuRoC_MH03';'EuRoC_MH04';'EuRoC_MH05';'Myung_Desk';'Myung_Aerial';'EuRoC_V101';'EuRoC_V102';'EuRoC_V103';'EuRoC_V201';'EuRoC_V202';'EuRoC_V203'};
% Seq = {'EuRoC_MH03';'EuRoC_MH04';'EuRoC_MH05';'Myung_Desk';'Myung_Aerial';'EuRoC_V103';'EuRoC_V203'};
% Model = {'noIMU';'myungIMU';'velocitySwitch'};
% 
% 
% for i = 1:7
%     logFile1 =  strcat('ORB_',Seq{i}, '_', Model{1}, '.csv')
%     logFile2 =  strcat('ORB_',Seq{i}, '_', Model{2}, '.csv')
%     logFile3 =  strcat('ORB_',Seq{i}, '_', Model{3}, '.csv')
%     Compare3ORB(logFile1,logFile2,logFile3);
% end

%% KLT Parser and Plotter
%924 seconds
Seq = {'EuRoC_MH01';'EuRoC_MH02';'EuRoC_MH03';'EuRoC_MH04';'EuRoC_MH05';'Myung_Desk';'Myung_Aerial';'EuRoC_V101';'EuRoC_V102';'EuRoC_V103';'EuRoC_V201';'EuRoC_V202';'EuRoC_V203'};
%Seq = {'EuRoC_MH03';'EuRoC_MH04';'EuRoC_MH05';'Myung_Desk';'Myung_Aerial';'EuRoC_V103';'EuRoC_V203'};
%Seq = {'EuRoC_MH03';'Myung_Desk'};

%Model = {'noIMU';'myungIMU';'velocitySwitch'};
Model = {'noIMU';'myungIMU';'velocitySwitch';'axisAngleIntegration'};

%initialize table variables
featAvg = zeros(length(Seq),length(Model));
featSTD = featAvg;
timeAvg = featAvg;
timeSTD = featAvg;

for i = 1:length(Seq)
    logFile1 =  strcat('KLT_',Seq{i}, '_', Model{1}, '.csv');
    logFile2 =  strcat('KLT_',Seq{i}, '_', Model{2}, '.csv');
    logFile3 =  strcat('KLT_',Seq{i}, '_', Model{3}, '.csv');
    logFile4 =  strcat('KLT_',Seq{i}, '_', Model{4}, '.csv');
    [featAvg(i,:), featSTD(i,:), timeAvg(i,:), timeSTD(i,:)] = Compare4KLT(logFile1,logFile2,logFile3,logFile4);
end

%plot the tables
f = figure;
t1 = uitable(f,'Data',featAvg,'ColumnWidth',{100},'ColumnName',Model,'RowName',Seq);
% Set width and height
t1.Position(3) = t1.Extent(3);
t1.Position(4) = t1.Extent(4);
f.Name = 'featAvg (%)';
saveas(gcf, 'KLT_Figures/featAvgTable.png');
saveas(gcf, 'KLT_Figures/featAvgTable.fig');

f = figure;
t2 = uitable(f,'Data',featSTD,'ColumnWidth',{100},'ColumnName',Model,'RowName',Seq);
% Set width and height
t2.Position(3) = t2.Extent(3);
t2.Position(4) = t2.Extent(4);
f.Name = 'featSTD (%)';
saveas(gcf, 'KLT_Figures/featSTDTable.png');
saveas(gcf, 'KLT_Figures/featSTDTable.fig');

f = figure;
t3 = uitable(f,'Data',timeAvg,'ColumnWidth',{100},'ColumnName',Model,'RowName',Seq);
% Set width and height
t3.Position(3) = t3.Extent(3);
t3.Position(4) = t3.Extent(4);
f.Name = 'timeAvg (ms)';
saveas(gcf, 'KLT_Figures/timeAvgTable.png');
saveas(gcf, 'KLT_Figures/timeAvgTable.fig');

f = figure;
t4 = uitable(f,'Data',timeSTD,'ColumnWidth',{100},'ColumnName',Model,'RowName',Seq);
% Set width and height
t4.Position(3) = t4.Extent(3);
t4.Position(4) = t4.Extent(4);
f.Name = 'timeSTD (ms)';
saveas(gcf, 'KLT_Figures/timeSTDTable.png');
saveas(gcf, 'KLT_Figures/timeSTDTable.fig');

