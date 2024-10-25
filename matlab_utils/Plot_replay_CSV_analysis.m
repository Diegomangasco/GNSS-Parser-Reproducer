% Simple MATLAB script for plotting the data coming out of the .csv files
% generated with the test_rate mode of replay.py
clear all
close all
clc
colors = lines(4); % Generates 4 distinct colors
% Different position (UBX)
colors(1,:)=[255/255,242/255,0/255]; % Yellow
% Same position (UBX)
colors(2,:)=[255/255,0/255,0/255]; % Red
% Different position (NMEA)
colors(3,:)=[196/255 230/255 155/255]; % Light green
% Same position (NMEA)
colors(4,:)=[16/255 138/255 59/255]; % Dark green

% Alternatively, you can define custom colors or use markers for each band
figure; % Open a new figure and hold it for multiple plots
title('Trace analysis without clustered data');
data=readtable("out_A4.csv");

% Keep only non-clustered data
data=data(data.Clustered==0,:);

% Remove points with invalid lat/lon
data=data(data.Latitude~=0,:);
data=data(data.Longitude~=0,:);

% UBX messages
UBX_data=data(data.Message_type=="UBX-NAV-PVT",:);

% NMEA sentences
NMEA_data=data(contains(data.Message_type,"NMEA"),:);

% UBX different position
idx = find(UBX_data.Same_pos_as_previous == 0);
lat = UBX_data.Latitude(idx);
lon = UBX_data.Longitude(idx);
color = colors(1,:);
geoscatter(lat, lon, 20, color, 'o', 'filled');
hold on;

% NMEA different position
idx = find(NMEA_data.Same_pos_as_previous == 0);
lat = NMEA_data.Latitude(idx);
lon = NMEA_data.Longitude(idx);
color = colors(3,:);
geoscatter(lat, lon, 20, color, 'o', 'filled');
hold on;

% UBX same position
idx = find(UBX_data.Same_pos_as_previous == 1);
lat = UBX_data.Latitude(idx);
lon = UBX_data.Longitude(idx);
color = colors(2,:);
geoscatter(lat, lon, 20, color, 'o', 'filled');
hold on;

% NMEA same position
idx = find(NMEA_data.Same_pos_as_previous == 1);
lat = NMEA_data.Latitude(idx);
lon = NMEA_data.Longitude(idx);
color = colors(4,:);
geoscatter(lat, lon, 20, color, 'o', 'filled');

hold off;
legend('Different position (UBX)','Different position (NMEA)','Same position (UBX)','Same position (NMEA)', 'Location', 'best');

figure;
hold on;
plot(data.Timestamp_ms,data.Peridocity_ms,'--');
plot(UBX_data.Timestamp_ms,UBX_data.Peridocity_ms,'-');
plot(NMEA_data.Timestamp_ms,NMEA_data.Peridocity_ms,'-');
xlabel("Time since beginning of the trace [ms]");
ylabel("Time since last update [ms]");
legend('All','UBX only','NMEA only');
title('Update rate analysis without clustered data');
hold off;