% This script reads in the KLT image file names and extracts
% the timestamps to correlate the frame #s with the timestamps

% desk_time(frame#) = timestamp
% aerial_time(frame#) = timestamp


fileDesk = 'desk_img.log';
fileAerial = 'aerial_img.log';
fileMH01 = 'mh01_c0.csv';
fileMH02 = 'mh02_c0.csv';
fileMH03 = 'mh03_c0.csv';
fileMH04 = 'mh04_c0.csv';
fileMH05 = 'mh05_c0.csv';

fileV101 = 'v101_c0.csv';
fileV102 = 'v102_c0.csv';
fileV103 = 'v103_c0.csv';
fileV201 = 'v201_c0.csv';
fileV202 = 'v202_c0.csv';
fileV203 = 'v203_c0.csv';
%% Desk
fidDesk = fopen(fileDesk);
desk = fscanf(fidDesk,'%s');

desk_ = strsplit(desk, '.jpg');

name = desk_(1);
nameData = strsplit(name{1}, {'_','.'},'CollapseDelimiters',true );
desk_time(1) = str2num(nameData{4});

for i = 2:length(desk_)   
    name = desk_(i);
    nameData = strsplit(name{1}, {'_','.'},'CollapseDelimiters',true );
    if(length(nameData) >= 3 )
        desk_time(i) = str2num(nameData{3});
    end
end

%% Aerial
fidAerial = fopen(fileAerial);
aerial = fscanf(fidAerial,'%s');

aerial_ = strsplit(aerial, '.jpg');

name = aerial_(1);
nameData = strsplit(name{1}, {'_','.'},'CollapseDelimiters',true );
aerial_time(1) = str2num(nameData{4});

for i = 2:length(aerial_)   
    name = aerial_(i);
    nameData = strsplit(name{1}, {'_','.'},'CollapseDelimiters',true );
    if(length(nameData) >= 3 )
        aerial_time(i) = str2num(nameData{3});
    end
end

%% EuRoC
% fidMH01 = fopen(fileMH01);
% mh01 = fscanf(fidMH01,'%f');

mh01_time = csvread(fileMH01);
mh02_time = csvread(fileMH02);
mh03_time = csvread(fileMH03);
mh04_time = csvread(fileMH04);
mh05_time = csvread(fileMH05);

v101_time = csvread(fileV101);
v102_time = csvread(fileV102);
v103_time = csvread(fileV103);
v201_time = csvread(fileV201);
v202_time = csvread(fileV202);
v203_time = csvread(fileV203);


%convert from nano to milliseconds
% mh01_time = mh01_time / 1000000;
% mh02_time = mh02_time / 1000000;
% mh03_time = mh03_time / 1000000;
% mh04_time = mh04_time / 1000000;
% mh05_time = mh05_time / 1000000;

%% Save to workspace file

save('kltFrameTimes','desk_time','aerial_time','mh01_time','mh02_time','mh03_time','mh04_time','mh05_time','v101_time','v102_time','v103_time','v201_time','v202_time','v203_time');

