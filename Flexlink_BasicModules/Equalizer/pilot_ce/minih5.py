"""Minimal pure-Python HDF5 reader (superblock v0, v1 object headers, symbol-table groups,
contiguous/compact layouts). Enough for simple files written by h5py/MATLAB without chunking.
Used only because h5py could not be installed in this workspace."""
import struct
import numpy as np


class MiniH5:
    def __init__(self, path):
        self.d = open(path, "rb").read()
        d = self.d
        assert d[:8] == b"\x89HDF\r\n\x1a\n"
        assert d[8] == 0, "only superblock v0 supported"
        self.so, self.sl = d[13], d[14]
        # root group symbol table entry at offset 24 + 4*8
        root = 24 + 4 * self.so
        self.root_ohdr = self._u(root + self.so, self.so)

    def _u(self, off, n):
        return int.from_bytes(self.d[off:off + n], "little")

    # ---------- object headers ----------
    def messages(self, addr):
        d = self.d
        ver, nmsg = d[addr], struct.unpack_from("<H", d, addr + 2)[0]
        assert ver == 1, f"object header version {ver} unsupported"
        hsize = struct.unpack_from("<I", d, addr + 8)[0]
        blocks = [(addr + 16, hsize)]
        out = []
        while blocks:
            start, size = blocks.pop(0)
            p, end = start, start + size
            while p + 8 <= end:
                mtype, msize, mflags = struct.unpack_from("<HHB", d, p)
                body = p + 8
                if mtype == 0x10:  # continuation
                    blocks.append((self._u(body, self.so), self._u(body + self.so, self.sl)))
                elif mtype != 0:
                    out.append((mtype, body, msize))
                p = body + msize
        return out

    # ---------- groups ----------
    def children(self, ohdr):
        for mtype, body, _ in self.messages(ohdr):
            if mtype == 0x11:
                btree, heap = self._u(body, self.so), self._u(body + self.so, self.so)
                return dict(self._walk_btree(btree, heap))
        return {}

    def _heap_str(self, heap, off):
        data_addr = self._u(heap + 8 + 2 * self.sl, self.so)
        s = data_addr + off
        return self.d[s:self.d.index(b"\0", s)].decode()

    def _walk_btree(self, node, heap):
        d = self.d
        assert d[node:node + 4] == b"TREE"
        level = d[node + 5]
        n = struct.unpack_from("<H", d, node + 6)[0]
        p = node + 8 + 2 * self.so  # first key
        for i in range(n):
            child = self._u(p + self.sl, self.so)
            if level > 0:
                yield from self._walk_btree(child, heap)
            else:
                yield from self._snod(child, heap)
            p += self.sl + self.so

    def _snod(self, addr, heap):
        d = self.d
        assert d[addr:addr + 4] == b"SNOD"
        n = struct.unpack_from("<H", d, addr + 6)[0]
        p = addr + 8
        esz = 2 * self.so + 4 + 4 + 16
        for i in range(n):
            name = self._heap_str(heap, self._u(p, self.so))
            yield name, self._u(p + self.so, self.so)
            p += esz

    # ---------- datatypes / dataspaces ----------
    def _dtype(self, off):
        d = self.d
        cv = d[off]
        cls, ver = cv & 0x0F, cv >> 4
        bits = d[off + 1] | (d[off + 2] << 8) | (d[off + 3] << 16)
        size = struct.unpack_from("<I", d, off + 4)[0]
        p = off + 8
        if cls == 0:  # fixed point
            signed = bool(bits & 0x08)
            return np.dtype(("<i" if signed else "<u") + str(size)), p + 4
        if cls == 1:  # float
            return np.dtype("<f" + str(size)), p + 12
        if cls == 3:  # string
            return np.dtype("S" + str(size)), p
        if cls == 6:  # compound
            nm = bits & 0xFFFF
            names, fmts, offs = [], [], []
            for _ in range(nm):
                e = d.index(b"\0", p)
                name = d[p:e].decode()
                if ver < 3:
                    p = p + ((e - p) // 8 + 1) * 8
                else:
                    p = e + 1
                if ver == 1:
                    moff = struct.unpack_from("<I", d, p)[0]; p += 4
                    p += 1 + 3 + 4 + 4 + 16  # dimensionality, reserved, perm, reserved, dims
                elif ver == 2:
                    moff = struct.unpack_from("<I", d, p)[0]; p += 4
                else:
                    nb = max(1, (size.bit_length() + 7) // 8)
                    moff = self._u(p, nb); p += nb
                mdt, p = self._dtype(p)
                names.append(name); fmts.append(mdt); offs.append(moff)
            return np.dtype({"names": names, "formats": fmts, "offsets": offs, "itemsize": size}), p
        if cls == 8:  # enum
            base, p2 = self._dtype(p)
            nm = bits & 0xFFFF
            names = []
            for _ in range(nm):
                e = d.index(b"\0", p2)
                names.append(d[p2:e].decode())
                p2 = p2 + ((e - p2) // 8 + 1) * 8 if ver < 3 else e + 1
            vals = np.frombuffer(d, base, nm, p2)
            dt = np.dtype(base, metadata={"enum": dict(zip(vals.tolist(), names))})
            return dt, p2 + nm * base.itemsize
        if cls == 9:  # variable length (string or sequence) -> stored in the global heap
            base, p2 = self._dtype(p) if (bits & 0x0F) == 0 else (None, p)
            return ("vlen", size), p2 if base is not None else p + 8
        raise NotImplementedError(f"datatype class {cls}")

    def _dspace(self, off):
        d = self.d
        ver, rank, flags = d[off], d[off + 1], d[off + 2]
        p = off + (8 if ver == 1 else 4)
        if ver == 2 and d[off + 3] == 2:
            return None
        dims = tuple(self._u(p + i * self.sl, self.sl) for i in range(rank))
        return dims

    def _decode(self, raw, dt, shape):
        arr = np.frombuffer(raw, dt, int(np.prod(shape)) if shape else 1).reshape(shape)
        if dt.names and set(n.lower() for n in dt.names) >= {"r", "i"} or (dt.names and set(dt.names) >= {"real", "imag"}):
            rn = [n for n in dt.names if n.lower() in ("r", "real")][0]
            iname = [n for n in dt.names if n.lower() in ("i", "imag")][0]
            arr = arr[rn].astype(float) + 1j * arr[iname].astype(float)
        return arr

    # ---------- attributes / datasets ----------
    def attrs(self, ohdr):
        d = self.d
        out = {}
        for mtype, body, _ in self.messages(ohdr):
            if mtype != 0x0C:
                continue
            ver = d[body]
            nsz, tsz, ssz = struct.unpack_from("<HHH", d, body + 2)
            p = body + 8
            pad = (lambda x: (x + 7) // 8 * 8) if ver == 1 else (lambda x: x)
            name = d[p:p + nsz - 1].decode() if ver == 1 else d[p:p + nsz].rstrip(b"\0").decode()
            p += pad(nsz)
            dt, _ = self._dtype(p); p += pad(tsz)
            shape = self._dspace(p); p += pad(ssz)
            n = int(np.prod(shape)) if shape else 1
            if isinstance(dt, tuple) and dt[0] == "vlen":
                vals = [self._global_heap_obj(p + i * dt[1] + 4) for i in range(n)]
                out[name] = vals[0].decode(errors="replace") if n == 1 else [v.decode(errors="replace") for v in vals]
                continue
            raw = d[p:p + n * dt.itemsize]
            out[name] = self._decode(raw, dt, shape if shape else ())
        return out

    def dataset(self, ohdr):
        d = self.d
        dt = shape = None
        raw = None
        for mtype, body, _ in self.messages(ohdr):
            if mtype == 1:
                shape = self._dspace(body)
            elif mtype == 3:
                dt, _ = self._dtype(body)
            elif mtype == 8:
                ver = d[body]
                if ver == 3:
                    cls = d[body + 1]
                    if cls == 1:  # contiguous
                        addr = self._u(body + 2, self.so)
                        size = self._u(body + 2 + self.so, self.sl)
                        raw = (addr, size)
                    elif cls == 0:  # compact
                        size = struct.unpack_from("<H", d, body + 2)[0]
                        raw = ("compact", body + 4, size)
                    else:
                        raise NotImplementedError("chunked layout")
                else:
                    raise NotImplementedError(f"layout v{ver}")
        n = int(np.prod(shape))
        if raw[0] == "compact":
            buf = d[raw[1]:raw[1] + raw[2]]
        elif raw[0] == 0xFFFFFFFFFFFFFFFF:
            buf = bytes(n * dt.itemsize)
        else:
            buf = d[raw[0]:raw[0] + n * dt.itemsize]
        return self._decode(buf, dt, shape)

    def _global_heap_obj(self, off):
        """Resolve a global heap ID (collection address + object index)."""
        d = self.d
        coll, idx = self._u(off, self.so), struct.unpack_from("<I", d, off + self.so)[0]
        assert d[coll:coll + 4] == b"GCOL"
        csize = self._u(coll + 8, self.sl)
        p, end = coll + 8 + self.sl, coll + csize
        while p < end:
            oid = struct.unpack_from("<H", d, p)[0]
            osz = self._u(p + 8, self.sl)
            if oid == idx:
                return d[p + 8 + self.sl:p + 8 + self.sl + osz]
            if oid == 0:
                break
            p += 8 + self.sl + (osz + 7) // 8 * 8
        raise KeyError(idx)

    def is_group(self, ohdr):
        return any(m[0] == 0x11 for m in self.messages(ohdr))

    def walk(self, ohdr=None, prefix=""):
        ohdr = self.root_ohdr if ohdr is None else ohdr
        yield prefix or "/", ohdr
        for name, child in self.children(ohdr).items():
            if self.is_group(child):
                yield from self.walk(child, prefix + "/" + name)
            else:
                yield prefix + "/" + name, child
