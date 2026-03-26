import { NEXUS_SVG } from "./config.js";
import { escapeHtml } from "./format.js";
import { state } from "./state.js";

const map = L.map("leafletMap", {
  center: [-35.3632621, 149.1652374],
  zoom: 18,
  zoomControl: true,
  attributionControl: true,
  preferCanvas: true,
});

L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 21,
  maxNativeZoom: 19,
  attribution: "© OpenStreetMap contributors",
  updateWhenIdle: true,
  updateWhenZooming: false,
  keepBuffer: 4,
}).addTo(map);

const sharedCanvas = L.canvas({ padding: 0.5 });
state.mapReady = true;
let mapClickHandler = null;

export function vehicleLatLng(vehicle) {
  if (!vehicle || !vehicle.gps) return null;
  const { lat, lon } = vehicle.gps;
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
  return [lat, lon];
}

export function buildVehicleIndex(data) {
  const byId = new Map();
  for (const vehicle of data.vehicles) byId.set(vehicle.id, vehicle);
  return byId;
}

function linkKey(link) {
  return [link.src, link.dst].sort().join("__");
}

function fmtHeading(headingDeg) {
  return Number.isFinite(headingDeg) ? `${Math.round(headingDeg)}°` : "--";
}

function makeMarkerIcon(vehicleId, headingDeg) {
  const rotation = Number.isFinite(headingDeg) ? headingDeg : 0;
  return L.divIcon({
    className: "",
    html: `
      <div class="drone-marker-wrap" data-vid="${vehicleId}">
        <div class="drone-marker" style="--heading:${rotation}deg;">${NEXUS_SVG}<span class="drone-heading-arrow"></span></div>
      </div>
    `,
    iconSize: [56, 68],
    iconAnchor: [28, 34],
  });
}

function makeLabelIcon(label) {
  return L.divIcon({
    className: "drone-label-icon",
    html: `<div class="drone-label">${escapeHtml(label)}</div>`,
    iconSize: null,
    iconAnchor: [0, 0],
  });
}

export function updateMarkerClasses(data) {
  for (const vehicle of data.vehicles) {
    const entry = state.markers.get(vehicle.id);
    if (!entry || !entry.el) continue;
    entry.el.classList.toggle("connected", !!vehicle.state?.connected);
    entry.el.classList.toggle("selected", state.selected === vehicle.id);
  }
}

function renderTargetMarker() {
  const target = state.pendingTarget;
  if (!target) {
    if (state.targetMarker) {
      map.removeLayer(state.targetMarker);
      state.targetMarker = null;
    }
    return;
  }

  const latlng = [target.latitude, target.longitude];
  if (!state.targetMarker) {
    state.targetMarker = L.circleMarker(latlng, {
      radius: 9,
      color: "rgba(255, 122, 69, 0.96)",
      weight: 2,
      fillColor: "rgba(255, 122, 69, 0.28)",
      fillOpacity: 0.9,
    }).addTo(map);
  } else {
    state.targetMarker.setLatLng(latlng);
  }
}

export function renderMap(data, onSelect, onMapTarget) {
  if (!state.mapReady) return;

  if (!mapClickHandler) {
    mapClickHandler = (event) => onMapTarget(event.latlng);
    map.on("click", mapClickHandler);
  }

  const vehiclesById = buildVehicleIndex(data);
  const validVehicles = data.vehicles.filter((vehicle) => vehicleLatLng(vehicle));

  if (!state.mapFitted && validVehicles.length) {
    const latlngs = validVehicles.map(vehicleLatLng);
    if (latlngs.length === 1) {
      map.setView(latlngs[0], 18);
    } else {
      map.fitBounds(L.latLngBounds(latlngs), { padding: [60, 60] });
    }
    state.mapFitted = true;
  }

  const seenVehicles = new Set();

  for (const vehicle of validVehicles) {
    const id = vehicle.id;
    const latlng = vehicleLatLng(vehicle);
    seenVehicles.add(id);

    if (state.markers.has(id)) {
      const entry = state.markers.get(id);
      entry.marker.setLatLng(latlng);
      entry.labelMarker.setLatLng(latlng);
      if (entry.el) {
        entry.el.style.setProperty("--heading", `${Number.isFinite(vehicle.heading_deg) ? vehicle.heading_deg : 0}deg`);
      }
    } else {
      const marker = L.marker(latlng, { icon: makeMarkerIcon(id, vehicle.heading_deg) })
        .addTo(map)
        .on("click", (event) => {
          L.DomEvent.stopPropagation(event);
          onSelect(id);
        });

      const labelMarker = L.marker(latlng, {
        icon: makeLabelIcon(vehicle.label),
        interactive: false,
      }).addTo(map);

      const entry = { marker, labelMarker, el: null };
      state.markers.set(id, entry);

      requestAnimationFrame(() => {
        const root = marker.getElement();
        if (!root) return;
        const el = root.querySelector(".drone-marker") || root;
        const current = state.markers.get(id);
        if (current) current.el = el;
        updateMarkerClasses(data);
      });
    }
  }

  for (const [id, entry] of state.markers.entries()) {
    if (!seenVehicles.has(id)) {
      map.removeLayer(entry.marker);
      map.removeLayer(entry.labelMarker);
      state.markers.delete(id);
    }
  }

  renderLinks(data, vehiclesById);
  renderTargetMarker();
  updateMarkerClasses(data);
}

function renderLinks(data, vehiclesById) {
  const seenLinks = new Set();

  for (const link of data.links) {
    const src = vehiclesById.get(link.src);
    const dst = vehiclesById.get(link.dst);
    if (!src || !dst) continue;

    const p1 = vehicleLatLng(src);
    const p2 = vehicleLatLng(dst);
    if (!p1 || !p2) continue;

    const key = linkKey(link);
    seenLinks.add(key);

    const isSelected = !!state.selected && (state.selected === link.src || state.selected === link.dst);
    const color = isSelected ? "rgba(255,122,69,0.42)" : "rgba(255,122,69,0.24)";
    const weight = isSelected ? 2.2 : 1.4;
    const dashArray = isSelected ? "8 7" : "6 7";

    if (state.uwbLines.has(key)) {
      const entry = state.uwbLines.get(key);
      entry.line.setLatLngs([p1, p2]);
      entry.line.setStyle({ color, weight, dashArray });
    } else {
      const line = L.polyline([p1, p2], {
        color,
        weight,
        dashArray,
        smoothFactor: 0,
        renderer: sharedCanvas,
      }).addTo(map);

      state.uwbLines.set(key, { line });
    }
  }

  for (const [key, entry] of state.uwbLines.entries()) {
    if (!seenLinks.has(key)) {
      map.removeLayer(entry.line);
      state.uwbLines.delete(key);
    }
  }
}
