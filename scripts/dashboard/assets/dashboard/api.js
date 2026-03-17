import { state } from "./state.js";

export async function fetchState() {
  if (state.refreshing) return null;
  state.refreshing = true;

  try {
    const res = await fetch("/api/state", {
      cache: "no-store",
      headers: {
        Accept: "application/json",
      },
    });

    if (!res.ok) {
      throw new Error(`HTTP ${res.status}`);
    }

    const data = await res.json();
    if (!data || !Array.isArray(data.vehicles) || !Array.isArray(data.links)) {
      throw new Error("Invalid API payload");
    }

    return data;
  } finally {
    state.refreshing = false;
  }
}
