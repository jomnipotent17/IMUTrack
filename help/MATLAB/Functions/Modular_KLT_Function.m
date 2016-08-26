function [frameData, Hist] = Modular_KLT_Function( inputFile )
% This Function Opens the Custom KLT LogFile We Made,
% parses through it and extracts all of the relevant data

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
%inputFile = 'log_Myung_Aerial_IMU_150.csv';


%% Open The .csv file and save it to an array
M = csvread(inputFile);

%%

% matches (frameID,  Matches, How long to compute, Error Before, Error After)
% features (frameID x featureID) with 1 indicating it exists
idx = 1;
frameIdx = 1;

% try declaring in advance to speed up
matches = zeros(3000,5);
features = zeros(4500,35000);

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
    matches(frameIdx,4) = BeforeErrorSum / count;
    matches(frameIdx,5) = AfterErrorSum / count;
    frameIdx = frameIdx + 1;
end 
%%
% Get the Accurate last frameIdx
frameIdx = frameIdx - 1;

%%
% Get rid of extra values in matches matrix store to output
frameData = matches(1:frameIdx,1:5);

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

%Shorten it to only the non-zero values
cleanH = find(H);
if( isempty(cleanH) )
    Hist = H(1);
else
    last = cleanH(end);
    Hist = H(1:last);
end

end