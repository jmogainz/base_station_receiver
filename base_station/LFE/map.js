const map = L.map('map').setView([40.730610, -73.935242], 13);

const tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 18
}).addTo(map);

const marker = L.marker([40.730610, -73.935242], { draggable: true }).addTo(map);

const polyline = L.polyline([], { color: '#0000FF' }).addTo(map);

map.on('click', function(e) {
  const latlng = e.latlng;
  const positions = polyline.getLatLngs();
  positions.push(latlng);
  polyline.setLatLngs(positions);
});

window.onbeforeunload = function() {
  const positions = polyline.getLatLngs();
  sessionStorage.setItem('positions', JSON.stringify(positions));
};

const storedPositions = JSON.parse(sessionStorage.getItem('positions'));
if (storedPositions) {
  polyline.setLatLngs(storedPositions);
}
