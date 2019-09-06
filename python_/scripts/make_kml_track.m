function result = make_kml_track(fname,Data);
result =1;
name=fname(max(findstr('\',fname))+1:end);
fid = fopen([fname '.kml'],'wt');
fprintf(fid,'<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2">\n');
fprintf(fid,'  <Document>    <name>%s</name>    <description>%s</description>\n',name,name);
fprintf(fid,'    <Style id="yellowLineGreenPoly">      <LineStyle>        <color>7f0000ff</color>        <width>4</width>      </LineStyle>      <PolyStyle>        <color>7f0000ff</color>      </PolyStyle>    </Style>\n');
fprintf(fid,'    <Placemark>      <name>Absolute Extruded</name>            <styleUrl>#yellowLineGreenPoly</styleUrl>      <LineString>        <extrude>1</extrude>        <tessellate>1</tessellate>               <coordinates>\n'); 

for i=1:size(Data,1)
    fprintf(fid,'%3.11f,%3.11f\n',Data(i,2),Data(i,1));
end
fprintf(fid,'</coordinates>      </LineString>\n');
fprintf(fid,'</Placemark>  </Document>\n');
fprintf(fid,'</kml>\n');
fclose(fid);