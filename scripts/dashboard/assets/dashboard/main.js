import { fetchState, sendCommand } from "./api.js";
import { POLL_MS } from "./config.js";
import { renderAll } from "./render.js";
import { selectVehicle } from "./select.js";
import { state } from "./state.js";

function rerender(data) {
  renderAll(data, handleSelect, handleCommand, handleMapTarget);
}

function handleSelect(id) {
  selectVehicle(id, rerender);
}

async function handleCommand(vehicleId, command, payload) {
  if (state.commandBusy) return;
  state.commandBusy = true;
  state.commandError = false;
  state.commandStatus = `${vehicleId}: ${command}...`;
  if (state.latest) rerender(state.latest);

  try {
    const result = await sendCommand(vehicleId, command, payload);
    state.commandStatus = `${vehicleId}: ${result.message || "command sent"}`;
  } catch (err) {
    state.commandError = true;
    state.commandStatus = `${vehicleId}: ${err.message || err}`;
  } finally {
    state.commandBusy = false;
    if (state.latest) rerender(state.latest);
  }
}

async function handleMapTarget(latlng) {
  if (!state.selected || !state.clickToGoArmed) return;

  const payload = {
    latitude: latlng.lat,
    longitude: latlng.lng,
    altitude: state.gotoAltitude,
  };
  state.pendingTarget = {
    vehicleId: state.selected,
    latitude: payload.latitude,
    longitude: payload.longitude,
    altitude: payload.altitude,
  };

  if (state.latest) rerender(state.latest);
  await handleCommand(state.selected, "go_to", payload);
}

async function refresh() {
  try {
    const data = await fetchState();
    if (!data) return;

    const selectedStillExists = state.selected && data.vehicles.some((vehicle) => vehicle.id === state.selected);
    if (!selectedStillExists) {
      state.selected = data.vehicles.length ? data.vehicles[0].id : null;
    }

    rerender(data);
  } catch (err) {
    document.getElementById("statusText").textContent = `Dashboard error: ${err.message || err}`;
  }
}

async function refreshLoop() {
  await refresh();
  state.refreshTimer = window.setTimeout(refreshLoop, POLL_MS);
}

refreshLoop();
