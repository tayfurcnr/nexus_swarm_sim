import { fetchState, sendCommand } from "./api.js";
import { POLL_MS } from "./config.js";
import { renderAll, renderCommandPanel, renderNeighborList, renderVehicleDetail, renderVehicleList } from "./render.js";
import { selectVehicle } from "./select.js";
import { state } from "./state.js";
import { updateMarkerClasses } from "./map.js";

function rerender(data) {
  renderAll(data, handleSelect, handleCommand, handleBatchCommand, handleMapTarget);
}

function rerenderSelection(data) {
  state.lastVehicleListStructureSignature = "";
  state.lastVehicleListLiveSignature = "";
  renderVehicleList(data, handleSelect, { force: true });
  renderVehicleDetail(data, handleCommand);
  renderCommandPanel(data, handleCommand, handleBatchCommand);
  renderNeighborList(data);
  updateMarkerClasses(data);
}

function handleSelect(id) {
  selectVehicle(id, rerenderSelection);
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

async function handleBatchCommand(command, payload = {}) {
  if (state.commandBusy || !state.latest) return;

  const targets = state.latest.vehicles.filter((vehicle) => !!vehicle.state?.connected);
  if (!targets.length) return;

  state.commandBusy = true;
  state.commandError = false;
  state.commandStatus = `${targets.length} vehicles: ${command}...`;
  if (state.latest) rerender(state.latest);

  let successCount = 0;
  let firstError = null;

  try {
    for (const vehicle of targets) {
      try {
        await sendCommand(vehicle.id, command, payload);
        successCount += 1;
      } catch (err) {
        if (!firstError) firstError = `${vehicle.id}: ${err.message || err}`;
      }
    }

    state.commandError = successCount !== targets.length;
    state.commandStatus = firstError
      ? `${command}: ${successCount}/${targets.length} ok · ${firstError}`
      : `${command}: ${successCount}/${targets.length} succeeded`;
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
