function Data=make_kml(name, Data, frequency, choice)
%Written by: James Chan
%Edited by: Curtis Pidgeon - 2015-02-24
%For the purpose taking in a matrix that has latitude and longitude and
%writing a kml file. 
%name is the name you want the kml file to be called
%Data is the matrix containing lat/long (latitude is a column, longitude is
%a column)
%frequency is the frequency at which you want to downsample the data. It must
%be an integer. If you wish to keep the same frequency, enter 1.
%choice is if you want it to be a track or if you want points. 1 is for
%track, 2 is for points

%check if downsampling is required
if (frequency~=1)
    Data=Data(1 : frequency : end,:);
end

%parse the input data
Lat = Data(:,1);
Long = Data(:,2);

LongLat = [Long, Lat];

fid = fopen([name '.kml'], 'wt');
sizeMat = size(LongLat);
col = sizeMat(:,1);
i = 1;

%Start KML
fprintf(fid,'<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">\n');
%Opening New Document
fprintf(fid,'<Document>\n\t<name>Output Trajectory</name>\n\t<description></description>\n\t<open>1</open>\n');

%% Setting Trajectory Line Styling
fprintf(fid,'\t<!-- Trajectory Line Style -->\n');
%Setting Linestyle 
fprintf(fid,'\t<Style id="TrajLineStyle">\n\t\t<LineStyle>\n\t\t\t<color>ff0000ff</color>\n\t\t\t<width>4</width>\n\t\t</LineStyle>\n');
%Setting Polystyle
fprintf(fid,'\t\t<PolyStyle>\n\t\t\t<color>ff0000ff</color>\n\t\t</PolyStyle>\n\t</Style>');
%% Setting Trajectory Point Styling
fprintf(fid,'\t<!-- Trajectory Point Style -->\n');
%Setting Iconstyle
fprintf(fid,'\t<Style id="TrajPointStyle">\n\t\t<IconStyle>\n\t\t\t<color>ff0000ff</color>\n\t\t\t<scale>0.5</scale>\n\t\t\t<Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_square.png</href></Icon>\n\t\t</IconStyle>\n');
%Setting Label Style
fprintf(fid,'\t\t<LabelStyle>\n\t\t\t<color>ff0000ff</color>\n\t\t\t<scale>0</scale>\n\t\t</LabelStyle>\n\t</Style>\n');

fprintf(fid,'\t<Folder>\n\t\t<name>Trajectory</name>\n\t\t<description></description>\n\t\t<visibility>1</visibility>\n\t\t<open>0</open>\n');
if choice == 1
    fprintf(fid,'\t\t<Placemark>\n\t\t\t<name>Trajectory</name>\n\t\t\t<description></description>\n\t\t\t<visibility>1</visibility>\n\t\t\t<styleUrl>#TrajLineStyle</styleUrl>\n\t\t\t<LineString>\n\t\t\t\t<altitudeMode>clampToGround</altitudeMode>\n\t\t\t\t<extrude>1</extrude>\n\t\t\t\t<tessellate>1</tessellate>\n\t\t\t\t<coordinates>\n');

    %flip commands to ensure matrix is in proper orientation
    fprintf(fid,'\t\t\t\t\t%3.11s,%3.11s,0\n',LongLat');

    fprintf(fid,'\t\t\t\t</coordinates>\n\t\t\t\t<gx:altitudeMode>clampToGround</gx:altitudeMode>\n\t\t\t</LineString>\n\t\t</Placemark>\n');

elseif choice == 2
    while i  < col 
        %fprintf(fid, '%.6f, %.6f, 0.0 \n', LatLong(i,1));
        fprintf(fid,'\t\t<Placemark>\n\t\t\t<name>%u</name>\n\t\t\t<description></description>\n\t\t\t<styleUrl>#TrajPointStyle</styleUrl>\n\t\t\t<visibility>1</visibility>\n\t\t\t<Point>\n\t\t\t\t<coordinates>\n',i);
        fprintf(fid, '\t\t\t\t\t%3.11s,%3.11s,0\n',LongLat(i,:)');
        fprintf(fid, '\t\t\t\t</coordinates>\n\t\t\t</Point>\n\t\t</Placemark>\n');
        i = i + 1;
    end;
end;
fprintf(fid,'\t</Folder>\n');

fprintf(fid, '%s \n', '</Document></kml>');
fclose(fid);

end
% %     =======
% function make_kml(name, Data, frequency, choice);
% %Written by: James Chan
% %For the purpose taking in a matrix that has latitude and longitude and
% %writing a kml file. 
% %name is the name you want the kml file to be called
% %Data is the matrix containing lat/long (latitude is a column, longitude is
% %a column)
% %frequency is the frequency at which you want to downsample the data. It must
% %be an integer. If you wish to keep the same frequency, enter 1.
% %choice is if you want it to be a track or if you want points. 1 is for
% %track, 2 is for points
% 
% Lat = Data(:,1);
% Long = Data(:,2);
% 
% Lat = downsample (Lat, frequency);
% Long = downsample (Long, frequency);
% 
% LatLong = [Lat, Long];
% 
% fid = fopen([name '.kml'], 'wt');
% sizeMat = size(LatLong);
% col = sizeMat(:,1);
% i = 1;
% 
% if choice == 1
%     
% header=['<kml xmlns="http://earth.google.com/kml/2.0"><Placemark><description>"' name '"</description><LineString><tessellate>1</tessellate><coordinates>'];
% footer='</coordinates></LineString> </Placemark></kml>';
% 
% fprintf(fid, '%s \n',header);
% 
% %flip commands to ensure matrix is in proper orientation
% fprintf(fid, '%.6f, %.6f, 0.0 \n', flipud(rot90(fliplr(LatLong))));
% 
% fprintf(fid, '%s', footer);
% 
% elseif choice == 2
% 
% fprintf(fid, '%s \n', '<?xml version="1.0" encoding="UTF-8"?>');
% fprintf(fid, '%s \n', '<kml xmlns="http://earth.google.com/kml/2.1">');
% fprintf(fid, '%s \n', '<Document>');
% 
% test = flipud(rot90(fliplr(LatLong)));
% 
% 
% while i  < col 
% 
% %fprintf(fid, '%.6f, %.6f, 0.0 \n', LatLong(i,1));
% fprintf(fid, '%s \n', '<Placemark><Point><coordinates>');
% fprintf(fid, '%.6f, %.6f, 0.0 \n',  test(:,i));
% fprintf(fid, '%s \n', '</coordinates></Point></Placemark>');
% i = i + 1;
% 
% end;
% fprintf(fid, '%s \n', '</Document></kml>');
% 
% end;
% 
% fclose(fid);
