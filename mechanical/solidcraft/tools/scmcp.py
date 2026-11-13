#!/usr/bin/env python3
"""A persistent client for SolidCraft's MCP server (`solidcraft-cli mcp`).

The server speaks JSON-RPC over stdio, so one server process is one design session. `serve` starts a
headless server and keeps it behind a Unix socket; `call` sends one MCP tool call to it and prints the
reply. Several sessions can run side by side, one socket each.

    scmcp.py serve --sock /tmp/sc-a.sock [--in design.sc] &      # one headless design session
    scmcp.py call  --sock /tmp/sc-a.sock batch @steps.json        # args from a file ...
    scmcp.py call  --sock /tmp/sc-a.sock measure '{"summary":true}' # ... or inline JSON
    scmcp.py tools --sock /tmp/sc-a.sock                          # tool names
    scmcp.py stop  --sock /tmp/sc-a.sock

The server may read and write files only in mechanical/solidcraft and /tmp (passed with --allow-path).
"""
import argparse
import json
import os
import socket
import socketserver
import subprocess
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # mechanical/solidcraft


class Server:
    def __init__(self, design=None, extra_paths=()):
        cmd = ["solidcraft-cli", "mcp", "--headless", "--compact", "--allow-path", ROOT, "--allow-path", "/tmp"]
        for p in extra_paths:
            cmd += ["--allow-path", p]
        if design:
            cmd += ["--in", design]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=sys.stderr,
                                     text=True, bufsize=1)
        self.lock = threading.Lock()
        self.next_id = 0
        self.rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                "clientInfo": {"name": "scmcp", "version": "1"}})
        self.notify("notifications/initialized", {})

    def notify(self, method, params):
        self.proc.stdin.write(json.dumps({"jsonrpc": "2.0", "method": method, "params": params}) + "\n")
        self.proc.stdin.flush()

    def rpc(self, method, params):
        with self.lock:
            self.next_id += 1
            rid = self.next_id
            self.proc.stdin.write(json.dumps({"jsonrpc": "2.0", "id": rid, "method": method, "params": params}) + "\n")
            self.proc.stdin.flush()
            while True:
                line = self.proc.stdout.readline()
                if not line:
                    raise RuntimeError("solidcraft-cli mcp exited")
                msg = json.loads(line)
                if msg.get("id") == rid:
                    return msg


def serve(args):
    srv = Server(args.design, args.allow_path or ())
    if os.path.exists(args.sock):
        os.unlink(args.sock)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            req = json.loads(self.rfile.readline())
            if req.get("method") == "__stop__":
                self.wfile.write(b'{"stopped":true}\n')
                threading.Thread(target=server.shutdown).start()
                return
            try:
                resp = srv.rpc(req["method"], req.get("params", {}))
            except Exception as e:                  # noqa: BLE001 - reported to the caller
                resp = {"error": {"message": str(e)}}
            self.wfile.write((json.dumps(resp) + "\n").encode())

    server = socketserver.ThreadingUnixStreamServer(args.sock, Handler)
    print("scmcp: serving on", args.sock, flush=True)
    try:
        server.serve_forever()
    finally:
        srv.proc.stdin.close()
        srv.proc.wait(timeout=30)
        if os.path.exists(args.sock):
            os.unlink(args.sock)


def request(sock, method, params):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(sock)
    s.sendall((json.dumps({"method": method, "params": params}) + "\n").encode())
    buf = b""
    while not buf.endswith(b"\n"):
        chunk = s.recv(1 << 20)
        if not chunk:
            break
        buf += chunk
    s.close()
    return json.loads(buf)


def load_arg(a):
    if a is None:
        return {}
    if a.startswith("@"):
        with open(a[1:]) as fh:
            return json.load(fh)
    return json.loads(a)


def call(sock, tool, arguments):
    """One tool call; returns (ok, content list). Usable from other scripts."""
    resp = request(sock, "tools/call", {"name": tool, "arguments": arguments})
    if "error" in resp:
        return False, [{"type": "text", "text": json.dumps(resp["error"])}]
    res = resp["result"]
    return not res.get("isError", False), res.get("content", [])


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve")
    s.add_argument("--sock", required=True)
    s.add_argument("--in", dest="design")
    s.add_argument("--allow-path", action="append")
    c = sub.add_parser("call")
    c.add_argument("--sock", required=True)
    c.add_argument("tool")
    c.add_argument("arguments", nargs="?")
    c.add_argument("--max", type=int, default=20000, help="truncate printed text to this many characters")
    t = sub.add_parser("tools")
    t.add_argument("--sock", required=True)
    st = sub.add_parser("stop")
    st.add_argument("--sock", required=True)
    args = ap.parse_args()

    if args.cmd == "serve":
        serve(args)
    elif args.cmd == "call":
        ok, content = call(args.sock, args.tool, load_arg(args.arguments))
        for item in content:
            if item.get("type") == "text":
                txt = item["text"]
                print(txt if len(txt) <= args.max else txt[:args.max] + "\n... [%d chars truncated]" % (len(txt) - args.max))
            elif item.get("type") == "image":
                print("[image %s, %d bytes base64]" % (item.get("mimeType"), len(item.get("data", ""))))
            else:
                print(json.dumps(item)[:2000])
        sys.exit(0 if ok else 1)
    elif args.cmd == "tools":
        resp = request(args.sock, "tools/list", {})
        for tl in resp["result"]["tools"]:
            print(tl["name"], "-", tl.get("description", "").split("\n")[0][:150])
    elif args.cmd == "stop":
        print(request(args.sock, "__stop__", {}))


if __name__ == "__main__":
    main()
