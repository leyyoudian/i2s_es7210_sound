#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Board self-test - PC upload server (with online playback)

Features:
  1. MCU streams recorded WAV to /upload, saved to local uploads/ folder.
  2. Browser visit http://<this-PC-IP>:<port>/ to list and play all recordings.

Usage:
    python upload_server.py [port] [save-dir]

    Default port 8080, default save dir ./uploads/
    On MCU side, set TEST_SERVER_IP in main/board_test.h to this PC's LAN IP.
"""

import sys
import os
import time
import html
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, unquote

DEFAULT_PORT = 8080
SAVE_DIR = "uploads"


def get_lan_ip() -> str:
    """通过 UDP 连接法自动获取物理网卡的局域网 IP（绕过虚拟网卡）

    原理：UDP connect 到外部地址时，内核会自动选择出口网卡，
    getsockname() 返回该网卡的 IP，虚拟网卡（VMware/Hyper-V/Docker）无路由故被跳过。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 53))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


class UploadHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != "/upload":
            self.send_response(404)
            self.end_headers()
            return

        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length)

        if len(body) < 44:
            self.send_response(400)
            self.end_headers()
            self.log_message("data too short (%d bytes), ignored", len(body))
            return

        riff = body[0:4]
        fmt = body[8:12]
        channels = int.from_bytes(body[22:24], "little")
        sample_rate = int.from_bytes(body[24:28], "little")
        bits = int.from_bytes(body[34:36], "little")
        data_bytes = int.from_bytes(body[40:44], "little")

        if riff != b"RIFF" or fmt != b"WAVE":
            self.send_response(400)
            self.end_headers()
            self.log_message("not WAV data, ignored")
            return

        os.makedirs(SAVE_DIR, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        # 优先使用 MCU 端 X-Filename 头, 否则按时间戳命名
        xname = self.headers.get("X-Filename", "")
        if xname and xname == os.path.basename(xname) and xname.endswith(".wav"):
            basename = "%s_%s" % (timestamp, xname)
        else:
            basename = "mic_test_%s.wav" % timestamp
        filename = os.path.join(SAVE_DIR, basename)
        with open(filename, "wb") as f:
            f.write(body)

        duration = (data_bytes / (sample_rate * channels * bits / 8)) if sample_rate else 0

        self.send_response(200)
        self.send_header("Content-Length", "2")
        self.end_headers()
        self.wfile.write(b"OK")

        self.log_message(
            "saved: %s (%.1fs, %dch, %dkHz, %dbit, %d bytes)",
            filename, duration, channels, sample_rate // 1000, bits, len(body),
        )

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/files":
            name = parsed.query.replace("f=", "").strip()
            name = unquote(name)
            if not name or name != os.path.basename(name):
                self.send_response(400)
                self.end_headers()
                return
            filepath = os.path.join(SAVE_DIR, name)
            if not os.path.exists(filepath):
                self.send_response(404)
                self.end_headers()
                return
            with open(filepath, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "audio/wav")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(self._render_index().encode("utf-8"))

    def _render_index(self):
        os.makedirs(SAVE_DIR, exist_ok=True)
        files = sorted(os.listdir(SAVE_DIR), reverse=True)
        wavs = [f for f in files if f.lower().endswith(".wav")]

        rows = []
        if not wavs:
            rows.append(
                '<tr><td colspan="5" style="color:#888;">No recordings yet, waiting for MCU upload...</td></tr>'
            )
        for i, name in enumerate(wavs):
            fpath = os.path.join(SAVE_DIR, name)
            size = os.path.getsize(fpath)
            size_kb = size / 1024.0
            dur = "?"
            try:
                with open(fpath, "rb") as f:
                    hdr = f.read(44)
                sr = int.from_bytes(hdr[24:28], "little")
                ch = int.from_bytes(hdr[22:24], "little")
                bits = int.from_bytes(hdr[34:36], "little")
                db = int.from_bytes(hdr[40:44], "little")
                if sr and ch and bits:
                    dur = "%.1fs" % (db / (sr * ch * bits / 8))
            except Exception:
                pass
            url = "/files?f=" + html.escape(name)
            rows.append(
                "<tr>"
                "<td>%d</td>"
                "<td>%s</td>"
                "<td>%.1f KB</td>"
                "<td>%s</td>"
                '<td><audio controls preload="none" src="%s"></audio></td>'
                "</tr>" % (i + 1, html.escape(name), size_kb, dur, url)
            )

        rows_html = "".join(rows)
        return (
            "<!DOCTYPE html>\n"
            "<html lang=\"zh-CN\">\n"
            "<head>\n"
            "<meta charset=\"utf-8\">\n"
            "<title>Board Self-Test - Recording Playback</title>\n"
            "<style>\n"
            "  body { font-family: 'Microsoft YaHei', Arial, sans-serif; margin: 24px; background:#f5f6fa; }\n"
            "  h1 { font-size: 22px; }\n"
            "  table { border-collapse: collapse; width: 100%; background:#fff; }\n"
            "  th, td { border: 1px solid #ddd; padding: 10px 12px; text-align: left; }\n"
            "  th { background:#3b82f6; color:#fff; }\n"
            "  tr:nth-child(even) { background:#f9fafb; }\n"
            "  audio { width: 320px; }\n"
            "  .tip { color:#666; margin: 12px 0; }\n"
            "</style>\n"
            "</head>\n"
            "<body>\n"
            "<h1>Recording Playback</h1>\n"
            "<div class=\"tip\">" + str(len(wavs)) + " recording(s). Refresh to see the latest upload.</div>\n"
            "<table>\n"
            "  <tr><th>#</th><th>Filename</th><th>Size</th><th>Duration</th><th>Play</th></tr>\n"
            "  " + rows_html + "\n"
            "</table>\n"
            "</body>\n"
            "</html>"
        )

    def log_message(self, fmt, *args):
        sys.stderr.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), fmt % args))


def main():
    global SAVE_DIR
    port = DEFAULT_PORT
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print("port param invalid, using default 8080")
    if len(sys.argv) > 2:
        SAVE_DIR = sys.argv[2]

    server = HTTPServer(("0.0.0.0", port), UploadHandler)
    os.makedirs(SAVE_DIR, exist_ok=True)
    lan_ip = get_lan_ip()
    print("======================================")
    print("  Board self-test upload server started")
    print("  Listen: 0.0.0.0:%d" % port)
    print("  Save dir: %s" % os.path.abspath(SAVE_DIR))
    print("  Playback page: http://%s:%d/" % (lan_ip, port))
    print("  Set TEST_SERVER_IP in main/board_test.h to this PC's LAN IP")
    print("======================================")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped")
        server.server_close()


if __name__ == "__main__":
    main()