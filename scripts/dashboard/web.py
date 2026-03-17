import json
import mimetypes
import os
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse


def resolve_asset_path(asset_name):
    assets_root = Path(__file__).resolve().parent / "assets"
    asset_path = (assets_root / asset_name).resolve()
    try:
        asset_path.relative_to(assets_root)
    except ValueError:
        return None
    return asset_path if asset_path.is_file() else None


def read_index_html():
    index_path = Path(__file__).resolve().parent / "index.html"
    return index_path.read_text(encoding="utf-8")


def create_handler(dashboard):
    class DashboardHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self._send_html(read_index_html())
                return
            if parsed.path == "/api/state":
                self._send_json(dashboard.snapshot())
                return
            if parsed.path.startswith("/assets/"):
                asset_name = parsed.path[len("/assets/"):]
                self._send_asset(asset_name)
                return
            self.send_error(404, "Not found")

        def log_message(self, format_string, *args):
            return

        def _send_html(self, body):
            encoded = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

        def _send_json(self, payload):
            encoded = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

        def _send_file(self, path, content_type):
            if not os.path.exists(path):
                self.send_error(404, "Not found")
                return
            with open(path, "rb") as handle:
                data = handle.read()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _send_asset(self, asset_name):
            asset_path = resolve_asset_path(asset_name)
            if asset_path is None:
                self.send_error(404, "Not found")
                return
            content_type, _ = mimetypes.guess_type(str(asset_path))
            self._send_file(str(asset_path), content_type or "application/octet-stream")

    return DashboardHandler
