"""開發用靜態伺服器：關掉快取，否則改了 ES module 瀏覽器還是吃舊的。

埠號固定用 8126。localStorage 是綁「協定＋主機＋埠號」的，換個埠號等於換一個
儲存空間，之前存的設定就讀不到了 —— 所以請固定用 http://localhost:8126，
不要用 127.0.0.1（那也是不同的網域）。
"""
import os
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else int(os.environ.get("PORT", 8126))
    print(f"serving on http://localhost:{port}")
    ThreadingHTTPServer(("127.0.0.1", port), NoCacheHandler).serve_forever()
