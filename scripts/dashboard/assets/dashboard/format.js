export function fmt(v) {
  return Number.isFinite(v) ? v.toFixed(2) : "--";
}

export function fmt7(v) {
  return Number.isFinite(v) ? v.toFixed(7) : "--";
}

export function escapeHtml(text) {
  return String(text)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}
