import gpxpy
import pandas as pd
import folium
from folium.map import Marker, Template

def get_path_coordinates(activity_id):
    filename = f"path/activity_{activity_id}.gpx"
    gpx = gpxpy.parse(open(filename))
    track = gpx.tracks[0]
    segment = track.segments[0]
    data_act= []
    segment_length = segment.length_3d()
    for point_idx, point in enumerate(segment.points):
        data_act.append([point.longitude, point.latitude,point.elevation,
                point.time, segment.get_speed(point_idx)])

        columns = ['Longitude', 'Latitude', 'Altitude', 'Time', 'Speed']
    df_gpx = pd.DataFrame(data_act, columns=columns)
    return list(map(list, zip(df_gpx.Longitude, df_gpx.Latitude)))


# Modify Marker template to include the onClick event
click_template = """{% macro script(this, kwargs) %}
    var {{ this.get_name() }} = L.marker(
        {{ this.location|tojson }},
        {{ this.options|tojson }}
    ).addTo({{ this._parent.get_name() }}).on('click', onClick);
{% endmacro %}"""

# Change template to custom template
Marker._template = Template(click_template)

#Map
df_loc = pd.read_csv('activities.csv')
m = folium.Map(
    location=[df_loc["Begin Latitude (°DD)"][0].mean(),df_loc["Begin Longitude (°DD)"][0].mean()],
    control_scale=True,
    zoom_start = 6)

# Add js to draw paths on marker click
m_id = m.get_name()
click_js = f"""function onClick(e) {{                                 
                 var coords = e.target.options.pathCoords;                 
                 var path = L.polyline(coords); 
                
                {m_id}.eachLayer(function(layer){{
                   if (layer instanceof L.Polyline)
                      {{ {m_id}.removeLayer(layer) }}
                      }});
                     
                path.addTo({m_id});
                 }}"""
                 
e = folium.Element(click_js)
html = m.get_root()
html.script.add_child(e)

for i, row in df_loc.iterrows():
    activity_id = row["Activity ID"]   
    path_coordinates = get_path_coordinates(activity_id)    
    folium.Marker(
        location = [row["Begin Latitude (°DD)"],row["Begin Longitude (°DD)"]],
        pathCoords=path_coordinates
    ).add_to(m)  
#x an y coord
#x1 = float(input("Enter The Starting Position North:"))
#x2 = float(input("Enter The Starting Position West:"))
#y1 = float(input("Enter The Ending Position North:"))
#y2 = float(input("Enter The Starting Position West:"))

#x = (x1, x2)
#y = (y1, y2)

#m = folium.Map(location = [x1,x2],zoom_start = 17) #creating masking layer via direct
#loc =[x,y]
#folium.LatLngPopup().add_to(m)
#folium.Marker([x1,x2],tooltip="<i>Starting Position</i>").add_to(m)
#folium.Marker([y1,y2],tooltip="<i>Ending Position</i>").add_to(m)
#folium.PolyLine(loc,color='blue', weight=10).add_to(m)

m.save("map_index.html")
