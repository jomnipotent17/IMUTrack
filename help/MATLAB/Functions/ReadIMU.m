% This Script reads in the IMU data files,
% and depending on which one it is it switches the values around,
% Gets rid of early data and late data and is left with only relevant
% timestamped data (accelerometer and gyro data)

%output format
% t wx wy wz ax ay az

%all of the data is there, later we can only plot what we want based 
% on the timestamps


%%
file1 = 'imu_data_mh01.csv';
file2 = 'imu_data_mh02.csv';
file3 = 'imu_data_mh03.csv';
file4 = 'imu_data_mh04.csv';
file5 = 'imu_data_mh05.csv';
file6 = 'desk_imu.log';
file7 = 'aerial_imu.log';

file8  = 'imu_data_v1_1.csv';
file9  = 'imu_data_v1_2.csv';
file10 = 'imu_data_v1_3.csv';
file11 = 'imu_data_v2_1.csv';
file12 = 'imu_data_v2_2.csv';
file13 = 'imu_data_v2_3.csv';

shapeFile6 = 'desk_imu_shape.txt'; 
shapeFile7 = 'aerial_imu_shape.txt';
%% Read in the file 

% EuRoC Log Files
%time is in nanoseconds
%w is in radians/sec
%a is in m/s^2
% t wx wy wz ax ay az
imu1 = csvread(file1,1,0);
imu2 = csvread(file2,1,0);
imu3 = csvread(file3,1,0);
imu4 = csvread(file4,1,0);
imu5 = csvread(file5,1,0);

imu8 = csvread(file8,1,0);
imu9 = csvread(file9,1,0);
imu10 = csvread(file10,1,0);
imu11 = csvread(file11,1,0);
imu12 = csvread(file12,1,0);
imu13 = csvread(file13,1,0);

%%
% Myung Desk Sequence

% logfile format
% t1 t2 ax ay az wx wy wz (unprocessed values)
% convert to
% t wx wy wz ax ay az
    FID6 = fopen(file6);
    imu6temp = fscanf(FID6,'%f');
    
    FID6_S  = fopen(shapeFile6);
    s6 = fscanf(FID6_S,'%f');
    
    acc_bias = [s6(1) s6(2) s6(3)];
    Ca = [s6(4) s6(5) s6(6),
          s6(7) s6(8) s6(9),
          s6(10) s6(11) s6(12)];
    
    gyro_bias = [s6(13) s6(14) s6(15)];
    Cw = [s6(16) s6(17) s6(18),
          s6(19) s6(20) s6(21),
          s6(22) s6(23) s6(24)];
    
    %convert this into a matrix (it has 8 columns)
    %but the second timestamp doesn't matter
    %and we need to subtract the bias and multiply by the correlation
    %matrix    
    imu6temp2 = zeros(length(imu6temp)/8, 8);
    
    for i = 1:length(imu6temp2)
        for j = 1:8
            imu6temp2(i,j) = imu6temp( (i-1)*8 + j );
        end
    end
    
    %now multiply through and store in the final matrix
    imu6 = zeros(length(imu6temp2),7);
    
    for i = 1:length(imu6)
        imu6(i,1) = imu6temp2(i,1); %store the timestamp
        
        %subtract the gyro bias
        imu6temp2(i,6) = imu6temp2(i,6) - gyro_bias(1);
        imu6temp2(i,7) = imu6temp2(i,7) - gyro_bias(2);
        imu6temp2(i,8) = imu6temp2(i,8) - gyro_bias(3);
        %imu6temp2(i,6:8) = imu6temp2(i,6:8) - gyro_bias; 
        
        %%
        %multiply by the correlation matrix
        old_gyro = [imu6temp2(i,6)
                    imu6temp2(i,7) 
                    imu6temp2(i,8) ];
        w_f = Cw * old_gyro;
        %w_f = Cw * [imu6temp2(i,6), imu6temp2(i,7), imu6temp2(i,8) ]
        
        
        %store the processed gyro data
        imu6(i,2) = w_f(1);
        imu6(i,3) = w_f(2);
        imu6(i,4) = w_f(3);
        
        %subtract the acc bias
        imu6temp2(i,3) = imu6temp2(i,3) - acc_bias(1);
        imu6temp2(i,4) = imu6temp2(i,4) - acc_bias(2);
        imu6temp2(i,5) = imu6temp2(i,5) - acc_bias(3);
        %imu6temp2(i,3:5) = imu6temp2(i,3:5) - acc_bias; 
        
        %multiply by the correlation matrix
        old_acc = [imu6temp2(i,3)
                   imu6temp2(i,4)
                   imu6temp2(i,5) ];
        a_f = Ca * old_acc;
        %a_f = Ca * [imu6temp2(i,3), imu6temp2(i,4), imu6temp2(i,5) ]
        
        %store the processed gyro data
        imu6(i,5) = a_f(1);
        imu6(i,6) = a_f(2);
        imu6(i,7) = a_f(3);
    end

%%
% Myung Aerial Sequence

    FID7 = fopen(file7);
    imu7temp = fscanf(FID7,'%f');
    
    FID7_S  = fopen(shapeFile7);
    s7 = fscanf(FID7_S,'%f');
    
    acc_bias = [s7(1) s7(2) s7(3)];
    Ca = [s7(4) s7(5) s7(6),
          s7(7) s7(8) s7(9),
          s7(10) s7(11) s7(12)];
    
    gyro_bias = [s7(13) s7(14) s7(15)];
    Cw = [s7(16) s7(17) s7(18),
          s7(19) s7(20) s7(21),
          s7(22) s7(23) s7(24)];
    
    %convert this into a matrix (it has 8 columns)
    %but the second timestamp doesn't matter
    %and we need to subtract the bias and multiply by the correlation
    %matrix    
    imu7temp2 = zeros(length(imu7temp)/8, 8);
    
    for i = 1:length(imu7temp2)
        for j = 1:8
            imu7temp2(i,j) = imu7temp( (i-1)*8 + j );
        end
    end
    
    %now multiply through and store in the final matrix
    imu7 = zeros(length(imu7temp2),7);
    
    for i = 1:length(imu7)
        imu7(i,1) = imu7temp2(i,1); %store the timestamp
        
        %subtract the gyro bias
        imu7temp2(i,6) = imu7temp2(i,6) - gyro_bias(1);
        imu7temp2(i,7) = imu7temp2(i,7) - gyro_bias(2);
        imu7temp2(i,8) = imu7temp2(i,8) - gyro_bias(3);
        
        %%
        %multiply by the correlation matrix
        old_gyro = [imu7temp2(i,6)
                    imu7temp2(i,7) 
                    imu7temp2(i,8) ];
        w_f = Cw * old_gyro;
        %w_f = Cw * [imu6temp2(i,6), imu6temp2(i,7), imu6temp2(i,8) ]
        
        
        %store the processed gyro data
        imu7(i,2) = w_f(1);
        imu7(i,3) = w_f(2);
        imu7(i,4) = w_f(3);
        
        %subtract the acc bias
        imu7temp2(i,3) = imu7temp2(i,3) - acc_bias(1);
        imu7temp2(i,4) = imu7temp2(i,4) - acc_bias(2);
        imu7temp2(i,5) = imu7temp2(i,5) - acc_bias(3);
        %imu6temp2(i,3:5) = imu6temp2(i,3:5) - acc_bias; 
        
        %multiply by the correlation matrix
        old_acc = [imu7temp2(i,3)
                   imu7temp2(i,4)
                   imu7temp2(i,5) ];
        a_f = Ca * old_acc;
        %a_f = Ca * [imu6temp2(i,3), imu6temp2(i,4), imu6temp2(i,5) ]
        
        %store the processed gyro data
        imu7(i,5) = a_f(1);
        imu7(i,6) = a_f(2);
        imu7(i,7) = a_f(3);
    end
    
%% Now Save the imu variables

save('imudata.mat','imu1','imu2','imu3','imu4','imu5','imu6','imu7','imu8','imu9','imu10','imu11','imu12','imu13');