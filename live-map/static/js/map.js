/**
 * map.js — MapLibre GL JS initialization with dark basemap.
 */

const CORNWALL_CENTER = [-5.05, 50.27];
const INITIAL_ZOOM = 8;

let map = null;

export function initMap() {
    map = new maplibregl.Map({
        container: 'map',
        style: {
            version: 8,
            name: 'MLAT Dark',
            sources: {
                'carto-dark': {
                    type: 'raster',
                    tiles: [
                        'https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}@2x.png',
                        'https://b.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}@2x.png',
                        'https://c.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}@2x.png',
                    ],
                    tileSize: 256,
                    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a> &copy; <a href="https://carto.com/">CARTO</a>'
                }
            },
            layers: [
                {
                    id: 'carto-dark-layer',
                    type: 'raster',
                    source: 'carto-dark',
                }
            ]
        },
        center: CORNWALL_CENTER,
        zoom: INITIAL_ZOOM,
        pitch: 0,
        bearing: 0,
    });

    map.addControl(new maplibregl.NavigationControl(), 'top-right');
    map.addControl(new maplibregl.ScaleControl({ maxWidth: 200 }), 'bottom-right');
    map.addControl(new maplibregl.FullscreenControl(), 'top-right');

    return map;
}

export function getMap() {
    return map;
}

export function flyTo(lon, lat, zoom = 12) {
    if (map) {
        map.flyTo({ center: [lon, lat], zoom, duration: 1500 });
    }
}
