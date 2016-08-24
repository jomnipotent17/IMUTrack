function [HistB, xVal] = Histogram_Blocker( Hist, block_size )
% This function takes a histogram and groups it into sections 
% of length block_size. It also returns the corresponding x_values

%% Group the Histogram into regions
sum = 0;
j=1;
for i = 1:length(Hist)
    sum = sum + Hist(i);
    if(mod(i,block_size) == 0 )
        HistB(j) = sum;
        sum = 0;
        j = j + 1;
    end
end
HistB(j) = sum;

for i = 1:j
   xVal(i) = i * block_size;
end

end

