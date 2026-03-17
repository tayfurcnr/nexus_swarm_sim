import { fetchState } from "./api.js";
import { POLL_MS } from "./config.js";
import { renderAll } from "./render.js";
import { selectVehicle } from "./select.js";
import { state } from "./state.js";

function rerender(data) {
  renderAll(data, handleSelect);
}

function handleSelect(id) {
  selectVehicle(id, rerender);
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
