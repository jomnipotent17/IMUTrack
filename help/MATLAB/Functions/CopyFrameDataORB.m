function [ frameDataOut ] = CopyFrameDataORB(otherFrameData)
    frameDataOut = otherFrameData;
    frameDataOut(:,3:6) = zeros(length(frameDataOut),4);
end

