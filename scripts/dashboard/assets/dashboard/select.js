import { state } from "./state.js";

export function getSelectedVehicle(data) {
  if (!data || !data.vehicles || !data.vehicles.length) return null;
  if (!state.selected) return data.vehicles[0];
  return data.vehicles.find((vehicle) => vehicle.id === state.selected) || data.vehicles[0];
}

export function selectVehicle(id, rerender) {
  if (state.selected === id) return;
  state.selected = id;

  if (!state.latest) return;
  rerender(state.latest);
}
