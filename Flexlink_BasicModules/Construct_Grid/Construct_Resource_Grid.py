import numpy as np
from enum import Enum

# Shared frequency-axis definitions (841 elements each)
K = np.arange(0, 841)            # 0 .. 840
TONE_I = np.arange(-420, 421)    # -420 .. 420
NUM_SIGNALS = K.size             # 841 signals per symbol

# 37 .. 121 are a multiple of 12 plus one final symbol: the pattern repeats a
# whole number of times for every PacketProfile, and the extra last symbol
# always starts a new repetition, so every grid ends with a Symbol_Ref.
# 4 (the control field: Ref, Data, Data, Ref) is 3 plus one, so it only ends
# with a Symbol_Ref for PacketProfile 1 or 3.
VALID_PACKET_LENS = (4, 37, 49, 73, 97, 121)
VALID_PROFILES = (1, 3, 6, 12)   # PacketProfile_1, _3, _6, _12

# Packet lengths that only allow some PacketProfiles (all others allow every one)
PROFILES_FOR_PACKET_LEN = {4: (1, 3)}


def valid_profiles(Packet_Len):
    """PacketProfiles allowed for Packet_Len (those that end the grid with a Symbol_Ref)."""
    return PROFILES_FOR_PACKET_LEN.get(Packet_Len, VALID_PROFILES)


class SymbolType(Enum):
    REF = "Symbol_Ref"
    DATA = "Symbol_Data"


class SignalType(Enum):
    UNDEFINED = 0     # layout not yet specified
    REF_P0 = 10
    REF_P1 = 20
    DC = 30
    DATA = 40


# DC signals: k = 414 .. 428 inclusive (15 signals, tones -6 .. +8), value forced to 0
DC_K = np.arange(414, 429)


def _data_symbol_layout():
    layout = np.full(NUM_SIGNALS, SignalType.DATA.value, dtype=np.int8)
    layout[DC_K] = SignalType.DC.value
    return layout


# Symbol_Ref layouts, one per ReferenceSymbolProfile (taken from ResourceGrid_Flexlink.ods,
# columns I..L). In every profile:
#   - Ref_P1 sits at k = p1_first, p1_first + P, ... (up to k=838)
#   - Ref_P0 sits at the k directly above each Ref_P1 (Ref_P1 + 1)
#   - DC occupies the listed inclusive k ranges (value forced to 0)
#   - every other signal is Data
VALID_REF_PROFILES = (3, 6, 12, 24)

_REF_SPEC = {
    3:  dict(p1_first=1,  dc=[(0, 0), (414, 414), (417, 417), (420, 420),
                              (423, 423), (426, 426), (840, 840)]),
    6:  dict(p1_first=4,  dc=[(0, 3), (414, 417), (420, 423), (426, 429), (840, 840)]),
    12: dict(p1_first=10, dc=[(0, 9), (408, 417), (420, 429), (840, 840)]),
    24: dict(p1_first=22, dc=[(0, 21), (408, 429), (840, 840)]),
}


def _ref_symbol_layout(ref_profile):
    spec = _REF_SPEC[ref_profile]
    layout = np.full(NUM_SIGNALS, SignalType.DATA.value, dtype=np.int8)
    p1 = np.arange(spec["p1_first"], 840, ref_profile)
    layout[p1] = SignalType.REF_P1.value
    layout[p1 + 1] = SignalType.REF_P0.value
    for lo, hi in spec["dc"]:
        layout[lo:hi + 1] = SignalType.DC.value
    return layout


DATA_SYMBOL_LAYOUT = _data_symbol_layout()
REF_SYMBOL_LAYOUTS = {p: _ref_symbol_layout(p) for p in VALID_REF_PROFILES}


def symbol_pattern(Profile):
    """One repetition of the profile: 1 Symbol_Ref then (Profile-1) Symbol_Data."""
    if Profile not in VALID_PROFILES:
        raise ValueError(f"Profile must be one of {VALID_PROFILES}, got {Profile}")
    return [SymbolType.REF] + [SymbolType.DATA] * (Profile - 1)


def symbol_types(Packet_Len, Profile):
    """Symbol type for each of the Packet_Len symbols (pattern repeated)."""
    if Packet_Len not in VALID_PACKET_LENS:
        raise ValueError(f"Packet_Len must be one of {VALID_PACKET_LENS}, got {Packet_Len}")
    pattern = symbol_pattern(Profile)
    if Profile not in valid_profiles(Packet_Len):
        raise ValueError(f"Packet_Len {Packet_Len} needs PacketProfile {valid_profiles(Packet_Len)} "
                         f"to end with a Symbol_Ref, got {Profile}")
    reps = -(-Packet_Len // len(pattern))          # ceil, truncates if not an exact fit
    return (pattern * reps)[:Packet_Len]


class Resource_Grid:
    """
    Resource grid of Packet_Len Symbols, each made of 841 Signals.

    Inputs:
      Packet_Len             : 4, 37, 49, 73, 97 or 121 (always ends with a Symbol_Ref;
                               4 only with PacketProfile 1 or 3)
      PacketProfile          : 1, 3, 6 or 12 -> which Symbols are Symbol_Ref / Symbol_Data
          1 : Ref, Ref, Ref, ...
          3 : Ref, Data, Data, | Ref, Data, Data, ...
          6 : Ref, Data x5, | ...
          12: Ref, Data x11, | ...
      ReferenceSymbolProfile : 3, 6, 12 or 24 -> signal layout used by every Symbol_Ref

    data[symbol, signal] is complex.
      - symbol index : 0 .. Packet_Len-1
      - signal index : 0 .. 840 (k); tone = Tone_i[k] = k - 420
    """

    def __init__(self, Packet_Len, PacketProfile, ReferenceSymbolProfile):
        self.Packet_Len = Packet_Len
        self.PacketProfile = PacketProfile
        self.ReferenceSymbolProfile = ReferenceSymbolProfile
        self.k = K
        self.Tone_i = TONE_I
        self.types = symbol_types(Packet_Len, PacketProfile)   # validates both args
        self.data = np.zeros((Packet_Len, NUM_SIGNALS), dtype=complex)
        # signal_types[n, k] -> numeric SignalType code of signal k in symbol n
        #   (10=Ref_P0, 20=Ref_P1, 30=DC, 40=Data)
        if ReferenceSymbolProfile not in VALID_REF_PROFILES:
            raise ValueError(f"ReferenceSymbolProfile must be one of {VALID_REF_PROFILES}, "
                             f"got {ReferenceSymbolProfile}")
        layouts = {SymbolType.REF: REF_SYMBOL_LAYOUTS[ReferenceSymbolProfile],
                   SymbolType.DATA: DATA_SYMBOL_LAYOUT}
        self.signal_types = np.array([layouts[t] for t in self.types], dtype=np.int8)
        self._dc_mask = self.signal_types == SignalType.DC.value

    # --- signal-type queries --------------------------------------------
    def signal_type(self, n, k):
        """SignalType enum of signal k in symbol n (numeric code: signal_types[n, k])."""
        return SignalType(int(self.signal_types[n, k]))

    def indices_of(self, n, sig_type):
        """k indices in symbol n that hold the given SignalType (enum or numeric code)."""
        code = sig_type.value if isinstance(sig_type, SignalType) else sig_type
        return np.flatnonzero(self.signal_types[n] == code)

    def count_of(self, sig_type):
        """Number of signals of the given type in the whole grid."""
        code = sig_type.value if isinstance(sig_type, SignalType) else sig_type
        return int(np.sum(self.signal_types == code))

    def enforce_dc(self):
        """Force every DC signal in the grid to 0."""
        self.data[self._dc_mask] = 0

    # --- symbol-type queries -------------------------------------------
    def symbol_type(self, n):
        return self.types[n]

    @property
    def ref_indices(self):
        return [i for i, t in enumerate(self.types) if t is SymbolType.REF]

    @property
    def data_indices(self):
        return [i for i, t in enumerate(self.types) if t is SymbolType.DATA]

    # --- access helpers -------------------------------------------------
    def symbol(self, n):
        """All 841 signals of symbol n (a view, so writes modify the grid)."""
        return self.data[n, :]

    def signal(self, n, k):
        return self.data[n, k]

    def signal_at_tone(self, n, tone):
        """Access by tone index (-420..420) instead of k."""
        return self.data[n, tone + 420]

    def set_symbol(self, n, values):
        values = np.asarray(values)
        if values.shape != (NUM_SIGNALS,):
            raise ValueError(f"Expected {NUM_SIGNALS} values, got shape {values.shape}")
        self.data[n, :] = values
        self.data[n, self._dc_mask[n]] = 0     # DC is always 0

    @property
    def shape(self):
        return self.data.shape  # (Packet_Len, 841)

    # --- file output ----------------------------------------------------
    def save_h5(self, filename="Resource_Grid.h5"):
        """
        Save the resource grid to an HDF5 file (requires h5py).

        Datasets:
          data          (Packet_Len, 841) complex   signal values
          signal_types  (Packet_Len, 841) int8      10=Ref_P0, 20=Ref_P1, 30=DC, 40=Data
          symbol_types  (Packet_Len,)     string    'Symbol_Ref' / 'Symbol_Data'
          k             (841,)  0 .. 840
          Tone_i        (841,)  -420 .. 420
        Root attributes: Packet_Len, PacketProfile, ReferenceSymbolProfile, and the
        signal-type codes (Ref_P0, Ref_P1, DC, Data).
        """
        import h5py
        with h5py.File(filename, "w") as f:
            f.create_dataset("data", data=self.data)
            f.create_dataset("signal_types", data=self.signal_types)
            f.create_dataset("symbol_types",
                             data=np.array([t.value for t in self.types], dtype="S11"))
            f.create_dataset("k", data=self.k)
            f.create_dataset("Tone_i", data=self.Tone_i)
            f.attrs["Packet_Len"] = self.Packet_Len
            f.attrs["PacketProfile"] = self.PacketProfile
            f.attrs["ReferenceSymbolProfile"] = self.ReferenceSymbolProfile
            for name, code in (("Ref_P0", SignalType.REF_P0), ("Ref_P1", SignalType.REF_P1),
                               ("DC", SignalType.DC), ("Data", SignalType.DATA)):
                f.attrs[name] = code.value
        return filename


if __name__ == "__main__":
    for n in VALID_PACKET_LENS:
        for p in VALID_PROFILES:
            if p not in valid_profiles(n):
                try:
                    Resource_Grid(n, p, 3)
                except ValueError:
                    continue
                raise AssertionError(f"Packet_Len {n} with PacketProfile {p} was not rejected")
            for r in VALID_REF_PROFILES:
                g = Resource_Grid(n, p, r)
                assert g.shape == (n, 841)
                assert len(g.ref_indices) == -(-n // p)           # ceil(n / p)
                assert g.types[-1] is SymbolType.REF              # last symbol is Ref
                assert g.ref_indices == list(range(0, n, p))
                assert len(g.ref_indices) + len(g.data_indices) == n
    g = Resource_Grid(37, 3, 6)
    print([t.name[0] for t in g.types[:9]])   # R D D R D D R D D
    print(Resource_Grid(37, 1, 12).ref_indices == list(range(37)))
    g = Resource_Grid(37, 6, 6)
    d = g.data_indices[0]
    assert len(g.indices_of(d, SignalType.DC)) == 15
    assert len(g.indices_of(d, SignalType.DATA)) == 826
    assert list(g.indices_of(d, SignalType.DC)) == list(range(414, 429))
    g.set_symbol(d, np.ones(841))
    assert np.all(g.data[d, 414:429] == 0) and np.all(g.data[d, :414] == 1) and np.all(g.data[d, 429:] == 1)
    # Control field grid: Ref, Data, Data, Ref with ReferenceSymbolProfile 3
    g = Resource_Grid(4, 3, 3)
    assert [t.name[0] for t in g.types] == ["R", "D", "D", "R"]
    assert [g.count_of(t) for t in (SignalType.REF_P0, SignalType.REF_P1, SignalType.DC,
                                    SignalType.DATA)] == [560, 560, 44, 2200]
    # Symbol_Ref layout checks (each type must cover all 841 signals)
    for r in VALID_REF_PROFILES:
        lay = REF_SYMBOL_LAYOUTS[r]
        counts = {t: int(np.sum(lay == t.value)) for t in SignalType}
        assert sum(counts.values()) == 841 and counts[SignalType.UNDEFINED] == 0
        assert counts[SignalType.REF_P0] == counts[SignalType.REF_P1]
    print("OK")
