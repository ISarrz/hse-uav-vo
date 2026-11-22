#!/usr/bin/env python3
"""Скачивание отдельных последовательностей EuRoC MAV.

Набор лежит в ETH Research Collection (doi:10.3929/ethz-b-000690084) тремя
большими архивами по 6-13 ГБ. Сервер поддерживает HTTP Range, поэтому из
архива читается только оглавление (несколько небольших запросов), а нужная
последовательность скачивается одним потоковым запросом через curl с докачкой.

    scripts/fetch_euroc.py --list
    scripts/fetch_euroc.py MH_01_easy V1_02_medium --out data/euroc
"""
import argparse
import io
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request
import zipfile
import zlib
from pathlib import Path

API = "https://www.research-collection.ethz.ch/server/api/core/bitstreams"
ARCHIVES = {
    "machine_hall": f"{API}/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/content",
    "vicon_room1": f"{API}/02ecda9a-298f-498b-970c-b7c44334d880/content",
    "vicon_room2": f"{API}/ea12bc01-3677-4b4c-853d-87c7870b8c44/content",
}
UA = "Mozilla/5.0 (X11; Linux x86_64) Chrome/130.0"


def archive_for(seq):
    if seq.startswith("MH_"):
        return "machine_hall"
    if seq.startswith("V1_"):
        return "vicon_room1"
    if seq.startswith("V2_"):
        return "vicon_room2"
    raise ValueError(f"неизвестная последовательность: {seq}")


def http_range(url, start, end, attempts=10):
    """GET с заголовком Range; при 429 и сетевых ошибках ждёт и повторяет."""
    for k in range(attempts):
        try:
            req = urllib.request.Request(
                url, headers={"User-Agent": UA, "Range": f"bytes={start}-{end}"})
            with urllib.request.urlopen(req, timeout=120) as r:
                data = r.read()
                if r.status == 206 and len(data) == end - start + 1:
                    return data, r.headers
            delay = 5
        except urllib.error.HTTPError as e:
            delay = int(e.headers.get("Retry-After") or 30) if e.code == 429 else 10
            print(f"  HTTP {e.code}, пауза {delay} с", file=sys.stderr, flush=True)
        except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
            delay = 10
            print(f"  {e}, пауза {delay} с", file=sys.stderr, flush=True)
        time.sleep(delay * (k + 1))
    raise IOError(f"не удалось получить байты {start}-{end}")


class HttpFile(io.RawIOBase):
    """Файл только для чтения поверх Range-запросов (для чтения оглавления zip)."""

    def __init__(self, url):
        self.url = url
        self.pos = 0
        # HEAD не возвращает Content-Length, поэтому размер берём из Content-Range
        _, headers = http_range(url, 0, 0)
        self.size = int(headers["Content-Range"].rsplit("/", 1)[1])

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=io.SEEK_SET):
        base = {io.SEEK_SET: 0, io.SEEK_CUR: self.pos, io.SEEK_END: self.size}[whence]
        self.pos = base + offset
        return self.pos

    def readinto(self, b):
        if self.pos >= self.size:
            return 0
        end = min(self.pos + len(b), self.size) - 1
        data, _ = http_range(self.url, self.pos, end)
        b[:len(data)] = data
        self.pos += len(data)
        return len(data)


def open_archive(name):
    raw = HttpFile(ARCHIVES[name])
    # оглавление большого zip занимает около мегабайта: читаем его крупными кусками
    return raw, zipfile.ZipFile(io.BufferedReader(raw, buffer_size=4 << 20))


def find_member(zf, seq):
    for info in zf.infolist():
        if info.filename.endswith(f"{seq}/{seq}.zip"):
            return info
    raise KeyError(f"{seq}.zip не найден в архиве")


def data_range(raw, info):
    """Байтовый диапазон сжатых данных файла внутри архива."""
    header, _ = http_range(raw.url, info.header_offset, info.header_offset + 29)
    name_len, extra_len = struct.unpack("<HH", header[26:30])
    start = info.header_offset + 30 + name_len + extra_len
    return start, start + info.compress_size - 1


def download(url, start, end, dst):
    """Потоковое скачивание диапазона через curl с докачкой при обрыве."""
    have = dst.stat().st_size if dst.exists() else 0
    total = end - start + 1
    while have < total:
        cmd = ["curl", "-sS", "-L", "-A", UA, "--fail", "--retry", "10", "--retry-delay", "30",
               "--retry-all-errors", "-r", f"{start + have}-{end}", url]
        with open(dst, "ab") as f:
            subprocess.run(cmd, stdout=f, check=False)
        new = dst.stat().st_size
        if new == have:
            time.sleep(60)
        have = new
        print(f"  {have / 1e9:.2f} / {total / 1e9:.2f} ГБ", flush=True)


def inflate(src, dst, method):
    """Распаковка члена внешнего архива (stored или deflate) во внутренний zip."""
    if method == zipfile.ZIP_STORED:
        src.rename(dst)
        return
    d = zlib.decompressobj(-15)
    with open(src, "rb") as fi, open(dst, "wb") as fo:
        while chunk := fi.read(16 << 20):
            fo.write(d.decompress(chunk))
        fo.write(d.flush())
    src.unlink()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sequences", nargs="*")
    ap.add_argument("--out", default="data/euroc")
    ap.add_argument("--list", action="store_true", help="показать последовательности и размеры")
    args = ap.parse_args()

    if args.list:
        for name in ARCHIVES:
            _, zf = open_archive(name)
            for info in zf.infolist():
                if info.filename.endswith((".zip", ".bag")) and "/._" not in info.filename:
                    print(f"{info.filename:55s} {info.file_size / 1e9:6.2f} ГБ")
        return

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    indexes = {}
    for seq in args.sequences:
        dst = out / seq
        if (dst / "mav0").exists():
            print(f"{seq}: уже скачан")
            continue
        name = archive_for(seq)
        if name not in indexes:
            indexes[name] = open_archive(name)
        raw, zf = indexes[name]
        info = find_member(zf, seq)
        if info.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED):
            raise NotImplementedError(f"метод сжатия {info.compress_type}")
        start, end = data_range(raw, info)
        print(f"{seq}: {info.compress_size / 1e9:.2f} ГБ", flush=True)

        part = out / f"{seq}.raw.part"
        download(raw.url, start, end, part)
        inner = out / f"{seq}.zip"
        inflate(part, inner, info.compress_type)
        with zipfile.ZipFile(inner) as z:
            z.extractall(dst)
        inner.unlink()
        print(f"{seq}: распакован в {dst}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
