function pos = plh2xyz(inPos)
%PLH2XYZ converts coordinates given in lat, lon and hgt into a
%   cartesian system.
%
%   PLH2XYZ(lat,lon,hgt) converts the coordinates using the WGS84
%      ellipsoid parameters.
%   PLH2XYZ(lat,lon,hgt,a,b) converts the coordinates using the 
%      ellipsoid parameters defined by 'a and 'b'.
%
%   NOTE: All angles must be given in radians
lat = inPos(1);
lon = inPos(2);
hgt = inPos(3);

a = 6378137.0;
b = 6356752.31425;

e2 = (a^2-b^2)/(a^2);
N = calcN(lat,a,b);

pos = zeros(3, 1);
pos(1) = (N+hgt)*cos(lat)*cos(lon);
pos(2) = (N+hgt)*cos(lat)*sin(lon);
pos(3) = ((1.0-e2)*N+hgt)*sin(lat);
