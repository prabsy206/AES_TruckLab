%%% filtering the cabin view

clear; close all; clc;
load('scan_raw_06.mat');

for i=1:length(scan_raw_05)
    for j=1:360
        if scan_raw_05(1,i).Angles(j,1)>=(120*pi/180) || scan_raw_05(1,i).Angles(j,1)<=(-120*pi/180)
            scan_filtered_ranges(j,1) = scan_raw_05(1,i).Ranges(j,1);
        else
            if scan_raw_05(1,i).Ranges(j,1)<=0.15
                scan_filtered_ranges(j,1) = 0;
            else
                %scan_filtered_ranges(j,1) = scan_raw_05(1,i).Ranges(j,1);
                scan_filtered_ranges(j,1) = 0;
            end
        end
    end
    scan_filtered(i) = lidarScan(scan_filtered_ranges,scan_raw_05(1,i).Angles);
end

save scan_red;

