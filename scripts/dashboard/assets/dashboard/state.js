export const state = {
  selected: null,
  latest: null,
  markers: new Map(),
  uwbLines: new Map(),
  mapReady: false,
  mapFitted: false,
  refreshTimer: null,
  refreshing: false,
  lastSelectionRenderId: null,
  lastSystemSignature: "",
};
