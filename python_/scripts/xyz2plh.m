function pos = xyz2plh(inPos)
%XYZ2PLH converts coordinates given in a cartesian system into 
%   lat, lon and hgt.
%
%   XYZ2PLH(x,y,z) converts the coordinates using the WGS84
%      ellipsoid parameters.
%   XYZ2PLH(x,y,z,a,b) converts the coordinates using the 
%      ellipsoid parameters defined by 'a and 'b'.
%
%   NOTE: Return values for lat and lon are given in radians


a  = 6378137.0;
b  = 6356752.31425;

e2 = (a^2-b^2)/(a^2);

x  = inPos(1);
y  = inPos(2);
z  = inPos(3);

p  = sqrt(x^2+y^2);

pos = zeros(3,1);

% check for sigularity
if p<=1.0e-6
	if z>0
		pos(1) = pi/2.0;
	else
		pos(1) = -pi/2.0;
	end
	pos(2) = 0.0;		% longitude does not exist
	pos(3) = abs(z)-b;

else
	N0 = 0;
	h0 = 0;
	
	phi = atan(z/p/(1-e2));

	N1 = calcN(phi,a,b);
	h1 = p/cos(phi) - N1;
	phi = atan((z/p)/(1-e2*N1/(N1+h1)));

	while abs(N1-N0)>=0.01 & abs(h1-h0)>=0.01
		N0 = N1;
		h0 = h1;

		N1 = calcN(phi,a,b);
		h1 = p/cos(phi) - N1;
		phi = atan((z/p)/(1-e2*N1/(N1+h1)));
	end

	pos(1) = phi;
	pos(2) = atan2(y,x);		% longitude is given by a closed formula
	pos(3) = h1;
end		

