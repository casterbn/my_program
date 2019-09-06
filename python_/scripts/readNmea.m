
function [hour, minute, second, velocity, numsats, height, lat_decimal, long_decimal, quality] = readNmea(filename)
%% Enter NMEA GPS data
% GPS data = readGPS (filename)
% GSPdata is a matrix with the following content:
% 
% Definition: http://en.wikipedia.org/wiki/NMEA_0183

fid = fopen (filename, 'rt');

% is looking for the line feeds, 10 is the ASCII code for one
% Newline
filelength = nnz (fread (fid) == 10);
disp(sprintf('Approx.% d lines are read in ...', filelength));

%set pointer back to start
fseek(fid, 0, 'bof');

% Preallocation of the data
i = 1;
GPS_data = zeros(filelength, 9);
hour = 0;
minute = 0;
second = 0;
UTC = 0;
lat_degree = 0;
lat_decimal = 0;
long_decimal = 0;
lat_A = '0';
long_A = '0';
velocity = 0;
course = 0;
numsats = 0;
height = 0;
lat_decimal = 0;
long_decimal = 0;
quality = 0;

while ~ feof(fid)% as long as the end of the file is not reached
    line = fgetl(fid); % read line
    if isempty (line)% if empty line
        continue% skip
    elseif strncmp(line, '$GPGGA', 6)% if line with $ GPRMC
        data = textscan(line, '%s%f%f%c%f%c%f%f%f%f', 1, 'delimiter', ','); 
        % compute UTC (HHMMSS.SSS), Universal Time Coordinated 
        hour = floor (data {2} / 10000); 
        minute = floor ((data {2} -hour * 10000) / 100); 
        second = (data {2} -floor (data {2} / 100) * 100); 
        UTC = strcat (num2str (hour), ':', num2str (minute), ':', num2str (second)); 

        % compute latitude (DDMM.MMM) and longitude (DDDMM.MMMM) 
        lat_degree = floor (data {3} / 100); 
        lat_decimal = (data {3} -lat_degree * 100) / 60; 
        lat = lat_degree + lat_decimal;
%         lat_A = strcat (num2str (lat_degree + lat_decimal), data {4}); 

        long_degree = floor (data {5} / 100); 
        long_decimal = (data {5} -long_degree * 100) / 60; 
        lon = long_decimal + long_degree;
%         long_A = strcat (num2str (long_degree + long_decimal), data {6});

        numsats = data {8};

        % GPS quality:
        % 0 is invalid
        % 1 for GPS fix
        % 2 fix for DGPS
        % 6 for protected (only with NMEA-0183 from version 2.3)
        quality = data {7};

        % Of the antenna via geoid or MSL (mean sea level)
        height = data {10};
        
    elseif strncmp (line, '$ GPVTG', 6)% if line with $ GPRMC
        GPVTGdata = textscan (line, '% s% f% c% f% c% f% c% f% c', 1, 'delimiter', ',');
        % compute velocity (km / h) and course 
        velocity = GPVTGdata {8}; 
        course = GPVTGdata {2}; 
    end
    GPS_data(i,:) = [hour, minute, second, velocity, numsats, height, lat, lon, quality];
    i = i + 1; % one line ahead
end

fclose(fid);

hour = GPS_data (:, 1);
minute = GPS_data (:, 2);
second = GPS_data (:, 3);
velocity = GPS_data (:, 4);
numsats = GPS_data (:, 5);
height = GPS_data (:, 6);
lat_decimal = GPS_data (:, 7);
long_decimal = GPS_data (:, 8);
quality = GPS_data (:, 9);

end





