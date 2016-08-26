function [frameData, Hist] = Modular_ORB_Function( inputFile )
% This Function Opens the Custom ORB_SLAM2 LogFile We Made,
% parses through it and extracts all of the relevant data

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

%% Open The .csv file and save it to an array
M = csvread(inputFile);


%% Parse through the array we just saved

% traverse until we get to when Tracking is succesful
% Save important information in arrays of the following formats

% matches (frameID, timestamp, Matches, How long to compute, Error Before, Error After)
% features (frameID x featureID) with 1 indicating it exists
% repError (beforeOptimization, afterOptimization)


idx = 1;
frameIdx = 1;

% declare in advance to speed up
matches = zeros(3000,6);
features = zeros(4000,53000);

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
            
            matches(frameIdx,5) = M(idx + 2,1);%store before Rep Error
            matches(frameIdx,6) = M(idx + 2,2);%store before Rep Error
            
            frameIdx = frameIdx + 1;
        end
    end
    idx = idx +1;
end

%%
% Get the Accurate last frameIdx
frameIdx = frameIdx - 1;

%%
% Get rid of extra values in matches matrix store to output
frameData = matches(1:frameIdx,1:6);

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

%Shorten it to only the non-zero values
cleanH = find(H);
if( isempty(cleanH) )
    Hist = H(1);
else
    last = cleanH(end);
    Hist = H(1:last);
end


end

