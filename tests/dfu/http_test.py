#!/usr/bin/env python3
# End to end: DfuHttp (dfu_http_host) downloads from Python's http.server
# over TCP. The server honours Range and drops the first connections part
# way, so the download resumes; the boot then installs the image.
#
# Usage: http_test.py <dfu_http_host> <image dir>

import http.server
import os
import socketserver
import subprocess
import sys
import threading

HOST, IMG = sys.argv[1], sys.argv[2]
drops = [7000, 15000]


class H(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def do_GET(self):
        data = open(os.path.join(IMG, self.path.lstrip("/")), "rb").read()
        start = 0
        rng = self.headers.get("Range")
        if rng:
            start = int(rng.split("=")[1].split("-")[0])
            self.send_response(206)
            self.send_header("Content-Range", "bytes %d-%d/%d" %
                             (start, len(data) - 1, len(data)))
        else:
            self.send_response(200)
        body = data[start:]
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if drops:
            cut = drops.pop(0)
            self.wfile.write(body[:cut - start if cut > start else 0])
            self.wfile.flush()
            self.close_connection = True
            return
        self.wfile.write(body)


srv = socketserver.TCPServer(("127.0.0.1", 0), H)
port = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()
r = subprocess.run([HOST, IMG, str(port), "/v3.bin"], capture_output=True,
                   text=True, timeout=120)
print(r.stdout.strip())
ok = r.returncode == 0 and "installed, 3 connections, violations 0" in r.stdout
srv.shutdown()
print("http_test: %s" % ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 1)
