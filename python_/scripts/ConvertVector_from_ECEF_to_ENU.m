
function vec_enu = ConvertVector_from_ECEF_to_ENU( pos, pos_ref )

% Get the curvilinear coordinates of reference point
pos_plh = xyz2plh( pos_ref );

lat_ref = pos_plh(1);
lon_ref = pos_plh(2);

% Rotation Matrix
R = [ -sin(lon_ref),              cos(lon_ref),               0; ...
      -sin(lat_ref)*cos(lon_ref), -sin(lat_ref)*sin(lon_ref), cos(lat_ref); ...
      cos(lat_ref)*cos(lon_ref),  cos(lat_ref)*sin(lon_ref),  sin(lat_ref)];

% Transform ECEF to ENU Local geodetic Coordinates
vec_enu = R * ( pos - pos_ref );