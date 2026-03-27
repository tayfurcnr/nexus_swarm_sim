import { escapeHtml, fmt, fmt7 } from "./format.js";
import { VEHICLE_LIST_REFRESH_MS } from "./config.js";
import { renderMap } from "./map.js";
import { getSelectedVehicle } from "./select.js";
import { state } from "./state.js";

export function renderSystem(data) {
  const signature = JSON.stringify({
    host: data.host,
    summary: data.summary,
  });

  if (signature === state.lastSystemSignature) {
    document.getElementById("clockText").textContent = `Last refresh: ${new Date().toLocaleTimeString()}`;
    return;
  }

  state.lastSystemSignature = signature;

  document.getElementById("statusText").textContent =
    (data.summary?.vehicle_count ?? 0) > 0
      ? `Dashboard streaming ${data.summary?.vehicle_count ?? 0} vehicles and ${data.summary?.link_count ?? 0} live links on ${escapeHtml(data.host?.hostname ?? "--")}.`
      : "Waiting for active vehicles...";

  document.getElementById("clockText").textContent = `Last refresh: ${new Date().toLocaleTimeString()}`;
}

export function renderVehicleList(data, onSelect, options = {}) {
  const list = document.getElementById("vehicleList");
  const force = !!options.force;
  const structureSignature = JSON.stringify(
    data.vehicles.map((vehicle) => ({
      id: vehicle.id,
      connected: !!vehicle.state?.connected,
      selected: state.selected === vehicle.id,
    })),
  );
  const liveSignature = JSON.stringify(
    data.vehicles.map((vehicle) => ({
      id: vehicle.id,
      position: vehicle.position,
    })),
  );
  const now = Date.now();

  if (!force) {
    if (structureSignature === state.lastVehicleListStructureSignature) {
      if (liveSignature === state.lastVehicleListLiveSignature) return;
      if (now - state.lastVehicleListRenderAt < VEHICLE_LIST_REFRESH_MS) return;
    }
  }

  state.lastVehicleListStructureSignature = structureSignature;
  state.lastVehicleListLiveSignature = liveSignature;
  state.lastVehicleListRenderAt = now;

  const existing = new Map();

  for (const el of list.querySelectorAll("[data-vid]")) {
    existing.set(el.dataset.vid, el);
  }

  const seen = new Set();

  for (const vehicle of data.vehicles) {
    seen.add(vehicle.id);
    const isActive = state.selected === vehicle.id;

    const html = `
      <div class="vehicle-dot ${vehicle.state?.connected ? "ok" : ""}"></div>
      <div class="vehicle-main">
        <small>${escapeHtml(vehicle.namespace ?? "--")}</small>
        <strong>${escapeHtml(vehicle.label)}</strong>
      </div>
      <div class="vehicle-meta">
        ${vehicle.state?.connected ? "CONNECTED" : "NO FCU"}<br>
        ${fmt(vehicle.position?.x)}, ${fmt(vehicle.position?.y)}, ${fmt(vehicle.position?.z)}
      </div>
    `;

    if (existing.has(vehicle.id)) {
      const el = existing.get(vehicle.id);
      el.className = `vehicle-item${isActive ? " active" : ""}`;
      if (el.innerHTML !== html) el.innerHTML = html;
    } else {
      const el = document.createElement("button");
      el.type = "button";
      el.dataset.vid = vehicle.id;
      el.className = `vehicle-item${isActive ? " active" : ""}`;
      el.innerHTML = html;
      el.addEventListener("click", () => onSelect(vehicle.id));
      list.appendChild(el);
    }
  }

  for (const [id, el] of existing.entries()) {
    if (!seen.has(id)) el.remove();
  }
}

export function renderVehicleDetail(data, onCommand) {
  const detail = document.getElementById("selectedSummary");
  const vehicle = getSelectedVehicle(data);

  if (!vehicle) {
    detail.innerHTML = `
      <p class="eyebrow">Nexus Swarm Sim</p>
      <h1>Mission Control</h1>
      <p class="sub">Select a vehicle on the map or from the roster.</p>
    `;
    state.lastSelectionRenderId = null;
    state.lastVehicleDetailSignature = "";
    return;
  }

  state.selected = vehicle.id;

  const gps = vehicle.gps;
  const links = data.links.filter((link) => link.src === vehicle.id || link.dst === vehicle.id);
  const lastSeen = Number.isFinite(vehicle.last_seen_sec) ? `${fmt(vehicle.last_seen_sec)} s ago` : "--";
  const missionState = vehicle.state?.armed ? "Armed" : "Standby";
  const navState = vehicle.state?.guided ? "Guided" : "Tracked";
  const telemetryState = vehicle.state?.connected ? "Telemetry Linked" : "No FCU Link";
  const signature = JSON.stringify({
    id: vehicle.id,
    position: vehicle.position,
    gps: vehicle.gps,
    state: vehicle.state,
    host_ip: vehicle.host_ip,
    hostname: vehicle.hostname,
    fcu_url: vehicle.fcu_url,
    last_seen_sec: vehicle.last_seen_sec,
    link_count: links.length,
  });
  const html = `
    <div class="dossier-shell">
      <p class="eyebrow">Selected Vehicle</p>
      <div class="dossier-grid">
        <section class="dossier-card dossier-card-identity">
          <div class="dossier-inline-head">
            <div class="dossier-title-col">
              <div class="dossier-kicker">${escapeHtml(vehicle.label.replace("NEXUS #", "#"))}</div>
              <div class="dossier-title-row">
                <div class="detail-name">${escapeHtml(vehicle.label)}</div>
                <span class="dossier-state">${escapeHtml(navState)}</span>
              </div>
              <div class="dossier-namespace">${escapeHtml(vehicle.namespace ?? "--")}</div>
            </div>
            <div class="dossier-tags">
              <span class="dossier-tag ${vehicle.state?.connected ? "is-live" : "is-cold"}">${escapeHtml(telemetryState)}</span>
              <span class="dossier-tag">${escapeHtml(missionState)}</span>
              <span class="dossier-tag">${escapeHtml(vehicle.state?.mode || "N/A")}</span>
            </div>
          </div>
        </section>

        <section class="dossier-card dossier-card-primary">
          <div class="dossier-card-head">
            <small class="list-label">Position + Global Fix</small>
            <span class="dossier-card-note">Local and GPS frames</span>
          </div>
          <div class="dossier-metrics dossier-metrics-telemetry">
            <div class="dossier-metric dossier-metric-gps">
              <small>X</small>
              <strong>${fmt(vehicle.position?.x)}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Y</small>
              <strong>${fmt(vehicle.position?.y)}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Z</small>
              <strong>${fmt(vehicle.position?.z)}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Latitude</small>
              <strong>${gps ? fmt7(gps.lat) : "--"}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Longitude</small>
              <strong>${gps ? fmt7(gps.lon) : "--"}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Altitude</small>
              <strong>${gps ? `${fmt(gps.alt)} m` : "--"}</strong>
            </div>
          </div>
        </section>

        <section class="dossier-card">
          <div class="dossier-card-head">
            <small class="list-label">Status + Endpoints</small>
            <span class="dossier-card-note">Mission and routing</span>
          </div>
          <div class="dossier-metrics dossier-metrics-telemetry">
            <div class="dossier-metric dossier-metric-gps">
              <small>Neighbors</small>
              <strong>${links.length}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Last Seen</small>
              <strong>${escapeHtml(lastSeen)}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Status Code</small>
              <strong>${escapeHtml(String(vehicle.state?.system_status ?? "--"))}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Host</small>
              <strong>${escapeHtml(vehicle.host_ip ?? "--")}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>Hostname</small>
              <strong>${escapeHtml(vehicle.hostname ?? "--")}</strong>
            </div>
            <div class="dossier-metric dossier-metric-gps">
              <small>FCU</small>
              <strong>${escapeHtml(vehicle.fcu_url ?? "--")}</strong>
            </div>
          </div>
        </section>
      </div>
    </div>
  `;

  if (signature !== state.lastVehicleDetailSignature && detail.innerHTML !== html) {
    detail.innerHTML = html;
    state.lastVehicleDetailSignature = signature;
  }

  state.lastSelectionRenderId = vehicle.id;
}

export function renderCommandPanel(data, onCommand, onBatchCommand) {
  const panel = document.getElementById("commandPanel");
  const vehicle = getSelectedVehicle(data);
  const connectedVehicles = data.vehicles.filter((entry) => !!entry.state?.connected);

  if (!vehicle) {
    panel.innerHTML = '<div class="muted">Select a vehicle to send arm, mode, takeoff, and map target commands.</div>';
    state.lastCommandPanelSignature = "";
    return;
  }

  const commandStatus = state.commandStatus
    ? `<div class="quick-status${state.commandError ? " is-error" : ""}">${escapeHtml(state.commandStatus)}</div>`
    : "";
  const connected = !!vehicle.state?.connected;
  const pendingTarget = state.pendingTarget && state.pendingTarget.vehicleId === vehicle.id
    ? `<div class="target-readout">Target: ${escapeHtml(fmt7(state.pendingTarget.latitude))}, ${escapeHtml(fmt7(state.pendingTarget.longitude))} @ ${escapeHtml(fmt(state.pendingTarget.altitude))} m</div>`
    : "";
  const signature = JSON.stringify({
    id: vehicle.id,
    connected,
    mode: vehicle.state?.mode,
    commandStatus: state.commandStatus,
    commandError: state.commandError,
    takeoffAltitude: state.takeoffAltitude,
    gotoAltitude: state.gotoAltitude,
    clickToGoArmed: state.clickToGoArmed,
    commandTab: state.commandTab,
    pendingTarget: state.pendingTarget,
    connectedVehicleIds: connectedVehicles.map((entry) => entry.id),
  });

  const singleTabActive = state.commandTab !== "multi";
  const tabsHtml = `
    <div class="command-tabs" role="tablist" aria-label="Quick command scope">
      <button type="button" class="command-tab ${singleTabActive ? "is-active" : ""}" data-command-tab="single">Single</button>
      <button type="button" class="command-tab ${singleTabActive ? "" : "is-active"}" data-command-tab="multi">Multi</button>
    </div>
  `;

  const singleHtml = `
      <div class="quick-actions">
        <button type="button" class="quick-btn" data-command="set_mode" data-mode="GUIDED">GUIDED</button>
        <button type="button" class="quick-btn" data-command="set_mode" data-mode="LOITER">LOITER</button>
        <button type="button" class="quick-btn" data-command="set_mode" data-mode="RTL">RTL</button>
        <button type="button" class="quick-btn" data-command="arm">ARM</button>
        <button type="button" class="quick-btn quick-btn-warn" data-command="disarm">DISARM</button>
      </div>

      <div class="takeoff-row">
        <label class="takeoff-field">
          <span>Takeoff Altitude (m)</span>
          <input id="takeoffAltitude" type="number" min="0.5" step="0.5" value="${escapeHtml(String(state.takeoffAltitude))}">
        </label>
        <button type="button" class="quick-btn quick-btn-accent" data-command="takeoff">TAKEOFF</button>
      </div>

      <div class="goto-row">
        <label class="takeoff-field">
          <span>Goto Altitude (m)</span>
          <input id="gotoAltitude" type="number" min="0.5" step="0.5" value="${escapeHtml(String(state.gotoAltitude))}">
        </label>
        <button type="button" class="quick-btn ${state.clickToGoArmed ? "quick-btn-accent" : ""}" id="clickToGoToggle">${state.clickToGoArmed ? "CLICK-TO-GO ON" : "CLICK-TO-GO"}</button>
      </div>

      <div class="goto-hint">${state.clickToGoArmed ? "Click an empty point on the map to send the selected vehicle there in GUIDED mode." : "Arm click-to-go, then click an empty point on the map to send the selected vehicle."}</div>
  `;

  const multiHtml = `
      <div class="multi-hint">Scope: all connected vehicles (${connectedVehicles.length}).</div>
      <div class="takeoff-row">
        <label class="takeoff-field">
          <span>Takeoff Altitude (m)</span>
          <input id="takeoffAltitude" type="number" min="0.5" step="0.5" value="${escapeHtml(String(state.takeoffAltitude))}">
        </label>
        <button type="button" class="quick-btn quick-btn-accent" data-batch-command="takeoff">TAKEOFF ALL</button>
      </div>
      <div class="quick-actions">
        <button type="button" class="quick-btn" data-batch-command="set_mode" data-mode="GUIDED">ALL GUIDED</button>
        <button type="button" class="quick-btn" data-batch-command="set_mode" data-mode="LOITER">ALL LOITER</button>
        <button type="button" class="quick-btn" data-batch-command="set_mode" data-mode="RTL">ALL RTL</button>
        <button type="button" class="quick-btn" data-batch-command="arm">ARM ALL</button>
        <button type="button" class="quick-btn quick-btn-warn" data-batch-command="disarm">DISARM ALL</button>
      </div>
      <div class="goto-hint">Multi commands are sent sequentially to all currently connected vehicles.</div>
  `;

  const html = `
    <div class="command-shell">
      <div class="command-head">
        <div>
          <div class="command-title">${escapeHtml(vehicle.label)}</div>
          <div class="command-sub">${escapeHtml(vehicle.namespace ?? "--")} · ${connected ? "FCU Connected" : "No FCU Link"}</div>
        </div>
        <span class="dossier-tag ${connected ? "is-live" : "is-cold"}">${escapeHtml(vehicle.state?.mode || "N/A")}</span>
      </div>

      ${tabsHtml}
      ${singleTabActive ? singleHtml : multiHtml}

      ${pendingTarget}

      ${commandStatus}
    </div>
  `;

  if (signature !== state.lastCommandPanelSignature || panel.innerHTML !== html) {
    panel.innerHTML = html;
    state.lastCommandPanelSignature = signature;
  }

  const altitudeInput = panel.querySelector("#takeoffAltitude");
  if (altitudeInput) {
    altitudeInput.disabled = state.commandBusy || !connected;
    altitudeInput.oninput = () => {
      const nextValue = Number(altitudeInput.value);
      if (Number.isFinite(nextValue) && nextValue > 0) {
        state.takeoffAltitude = nextValue;
      }
    };
  }

  const gotoAltitudeInput = panel.querySelector("#gotoAltitude");
  if (gotoAltitudeInput) {
    gotoAltitudeInput.disabled = state.commandBusy || !connected;
    gotoAltitudeInput.oninput = () => {
      const nextValue = Number(gotoAltitudeInput.value);
      if (Number.isFinite(nextValue) && nextValue > 0) {
        state.gotoAltitude = nextValue;
      }
    };
  }

  const clickToGoToggle = panel.querySelector("#clickToGoToggle");
  if (clickToGoToggle) {
    clickToGoToggle.disabled = state.commandBusy || !connected;
    clickToGoToggle.onclick = () => {
      state.clickToGoArmed = !state.clickToGoArmed;
      renderCommandPanel(data, onCommand);
    };
  }

  for (const button of panel.querySelectorAll("[data-command]")) {
    button.disabled = state.commandBusy || !connected;
    button.onclick = () => {
      const command = button.dataset.command;
      const payload = {};
      if (button.dataset.mode) payload.mode = button.dataset.mode;
      if (command === "takeoff") payload.altitude = state.takeoffAltitude;
      onCommand(vehicle.id, command, payload);
    };
  }

  for (const tab of panel.querySelectorAll("[data-command-tab]")) {
    tab.disabled = state.commandBusy;
    tab.onclick = () => {
      const nextTab = tab.dataset.commandTab === "multi" ? "multi" : "single";
      if (state.commandTab === nextTab) return;
      state.commandTab = nextTab;
      renderCommandPanel(data, onCommand, onBatchCommand);
    };
  }

  for (const button of panel.querySelectorAll("[data-batch-command]")) {
    button.disabled = state.commandBusy || !connectedVehicles.length;
    button.onclick = () => {
      const command = button.dataset.batchCommand;
      const payload = {};
      if (button.dataset.mode) payload.mode = button.dataset.mode;
      if (command === "takeoff") payload.altitude = state.takeoffAltitude;
      onBatchCommand(command, payload);
    };
  }
}

export function renderNeighborList(data) {
  const list = document.getElementById("neighborList");
  const vehicle = getSelectedVehicle(data);

  if (!vehicle) {
    if (state.lastNeighborListSignature !== "__empty__") {
      list.innerHTML = '<div class="empty-state">Select a vehicle to inspect nearby UWB links.</div>';
      state.lastNeighborListSignature = "__empty__";
    }
    return;
  }

  const links = data.links
    .filter((link) => link.src === vehicle.id || link.dst === vehicle.id)
    .sort((left, right) => left.distance_3d - right.distance_3d);

  const signature = JSON.stringify({
    vehicleId: vehicle.id,
    links: links.map((link) => ({
      src: link.src,
      dst: link.dst,
      distance_3d: link.distance_3d,
      los: link.los,
      rssi_dbm: link.rssi_dbm,
    })),
  });

  if (!links.length) {
    if (state.lastNeighborListSignature !== `${vehicle.id}:none`) {
      list.innerHTML = '<div class="empty-state">No recent UWB links for this vehicle.</div>';
      state.lastNeighborListSignature = `${vehicle.id}:none`;
    }
    return;
  }

  if (signature === state.lastNeighborListSignature) return;
  state.lastNeighborListSignature = signature;

  list.innerHTML = links.map((link) => {
    const neighborId = link.src === vehicle.id ? link.dst : link.src;
    return `
      <div class="neighbor-item ${link.los ? "los" : "nlos"}">
        <div class="neighbor-dot ${link.los ? "ok" : ""}"></div>
        <div class="neighbor-main">
          <small>Neighbor</small>
          <strong>${escapeHtml(neighborId)}</strong>
        </div>
        <div class="neighbor-meta">${fmt(link.distance_3d)} m<br>${link.los ? "LOS" : "NLOS"} · ${fmt(link.rssi_dbm)} dBm</div>
      </div>
    `;
  }).join("");
}

export function renderAll(data, onSelect, onCommand, onBatchCommand, onMapTarget) {
  state.latest = data;
  renderSystem(data);
  renderVehicleList(data, onSelect);
  renderVehicleDetail(data, onCommand);
  renderCommandPanel(data, onCommand, onBatchCommand);
  renderNeighborList(data);
  renderMap(data, onSelect, onMapTarget);
}
