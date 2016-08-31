function [ error ] = RepError(frame1, frame2)
%RepError Given tracked points in frame1 and frame2, this
% function constructs and uses a Fundemental Matrix to  
% reproject points and to calculate the average Reprojection Error

% Note: the Fundamental matrix could be backwards (i.e. transform points
    % from 1 -> 2 instead of 2 -> 1, but that would be a simple fix
    
% Also, I may need to do some sort of normalization but I am not sure

%% Step 1: Associate Matched Points and discard the rest
% i  %index for frame1
j = 1;  %index for frame2
k = 1;  %index for matched points

for i=1:length(frame1)
    if(~frame1(i,1))    %no more features from frame1
       break 
    end
    while(frame2(j,1) < frame1(i,1))
        j = j + 1;
    end
    if(frame(i,1) == frame2(j,1))
       %we have a match, copy the data into the matched variables
       matchedPoints1(k,1:2) = frame1(i,2:3);
       matchedPoints2(k,1:2) = frame2(j,2:3);
       k = k + 1;
    end
    %otherwise just keep going through the for loop
end

if(k == 1)
    error = 0;
    return
end
% Now we have two variables each with (x,y) entries for each points
%matchedPoints1 = (x y; x y; ...)
%matchedPoints2 = ''

%% Step 2: Construct the Fundamental Matrix
F = estimateFundamentalMatrix(matchedPoints1,matchedPoints2);

%% Step 3: Warp Points from Frame 2 to Frame 1
matchedPoints2(:,3) = ones();   %turn into homogenous coordinates
mP2 = matchedPoints2.';         %transpose this for easy multiplication

warpedPoints = F*mP2;           %multiply (warp the points)
warpedPoints = warpedPoints.';  %transpose to be in same format as other points

%% Step 4: Compute the individual errors
errorInd = zeros(length(matchedPoints1));
for i = 1:length(matchedPoints1)
   errorInd(i) = sqrt((matchedPoints1(i,1) - warpedPoints(i,1))^2 + (matchedPoints1(i,2) - warpedPoints(i,2))^2  );
end

%% Step 5: Compute the Average Reprojection Error
error = mean(errorInd);

end

