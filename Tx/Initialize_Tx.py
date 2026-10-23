#!/usr/bin/env python3
"""
Initialize_Tx.py - Initialize a Flexlink transmitter.

Steps:
  1. Locate and read Parameters_Tx_payload.json (via flexlink_paths.py).
  2. Build a Resource_Grid (Construct_Grid/Construct_Resource_Grid.py) from
     PacketLength (37, 49, 73, 97 or 121; the grid always ends with a reference
     symbol), RefSignalSpacing and RefSymbolSpacing, and save it to the .h5 file
     named in File/Grid_Resource_Tx, in the Tx directory (flexlink_paths.TX).
  3. Count the "Ref_P0", "Ref_P1", "Data" and "DC" signals in the grid (codes
     from RG_Get_Number_Of_SigTypes, counted with Resource_Grid.count_of) and
     write them back to the JSON file as NumberOfRefSigs_P0, NumberOfRefSigs_P1,
     NumberOfDataSigs, NumberOfDCSigs. NumberOfDataSigs is the Data signal count
     multiplied by the bits per signal for Mod (BPSK 1, QPSK 2, 16QAM 4, 64QAM 6).
  4. Calculate DataBlockSize = EncoderBlockSize * EncoderCodeRate
     (e.g. 1296 * 5/6 = 1080) and CRClen from CRC
     ("Gen24" -> 24), and TransportBlockSize = DataBlockSize - CRClen
     (e.g. 1056); CodeBlockSize = EncoderBlockSize,
     CodeWordSize = CodeBlockSize * (Repetition + 1); and write them all to
     the JSON file.
  5. AvailableDataBlocks = NumberOfDataSigs // CodeWordSize (whole code words
     that fit), ZeroPadding = the remainder, and ScrambledSequenceSize =
     AvailableDataBlocks * CodeWordSize + ZeroPadding (checked to equal
     NumberOfDataSigs), written to the JSON file. ScrambledCodeWordSize is
     renamed to ScrambledSequenceSize. Stops with an error if CodeWordSize is
     bigger than NumberOfDataSigs.
  6. Apply the parameters to the transmitter.
  7. Rebuild the control field (Flexlink_BasicModules/Construct_Control_Field/
     Build_Control_Field.py) from the new parameters, ending with
     Tx/ControlSignals/Waveform_Control_Tx.h5. The control field only changes when
     the Tx parameters do, so this is the one place it is recalculated.
     Skipped with --dry-run or --no-control-field.

Usage:
    python Initialize_Tx.py                  # find the JSON via flexlink_paths
    python Initialize_Tx.py --params my.json # use an explicit JSON file
    python Initialize_Tx.py --dry-run -v     # compute counts only; no JSON/.h5 write, no hardware
    python Initialize_Tx.py --no-control-field   # do not rebuild the control field
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import os
import re
import sys
import tempfile
from fractions import Fraction
from pathlib import Path
from types import ModuleType
from typing import Any

PARAMS_FILENAME = "Parameters_Tx_payload.json"

# Resource grid construction / counting
GRID_FOLDER_NAME = "Construct_Grid"
GRID_MODULE = "Construct_Resource_Grid"          # Construct_Grid/Construct_Resource_Grid.py
GRID_CLASS = "Resource_Grid"
SIGCODE_MODULE = "RG_Get_Number_Of_SigTypes"      # RG_Get_Number_Of_SigTypes.py
SIGCODE_FUNC = "RG_Get_Number_Of_SigTypes"       # name -> numeric code (Ref_P0 -> 10, ...)

# Resource_Grid(...) argument -> JSON key
#   RefSignalSpacing -> ReferenceSymbolProfile
#   RefSymbolSpacing -> PacketProfile
GRID_ARGS = {
    "Packet_Len":             "PacketLength",
    "PacketProfile":          "RefSymbolSpacing",
    "ReferenceSymbolProfile": "RefSignalSpacing",
}

# Where the grid file name lives in the JSON (File -> Grid_Resource_Tx)
GRID_FILE_KEY = "Grid_Resource_Tx"
DEFAULT_GRID_FILE = "Grid_Resource_Tx.h5"

# Bits carried by each data signal for each modulation (Defined/Mod).
# NumberOfDataSigs = Data signals in the grid x bits per signal.
MOD_BITS = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6}

# Signal type in the grid -> key written to the JSON file
SIG_TYPE_TO_KEY = {
    "Ref_P0": "NumberOfRefSigs_P0",
    "Ref_P1": "NumberOfRefSigs_P1",
    "DC":     "NumberOfDCSigs",
    "Data":   "NumberOfDataSigs",
}

# Names that flexlink_paths.py might use for the folder holding the parameter
# files. Put the real one first once you know it.
CANDIDATE_PATH_NAMES = (
    "PARAMS_DIR", "PARAMETERS_DIR", "CONFIG_DIR", "TX_DIR",
    "TX_PARAMS_DIR", "DATA_DIR", "BASE_DIR", "ROOT_DIR",
)

# Top-level keys that must exist in the JSON. Adjust to your file's schema.
REQUIRED_KEYS: tuple[str, ...] = ()

log = logging.getLogger("Initialize_Tx")


# --------------------------------------------------------------------------- #
# Locating the parameter file
# --------------------------------------------------------------------------- #
def candidate_dirs_from_flexlink_paths() -> list[Path]:
    """Collect folder locations exposed by flexlink_paths.py."""
    try:
        import flexlink_paths  # noqa: WPS433 (runtime import is intentional)
    except ImportError:
        log.warning("flexlink_paths.py not importable; falling back to script folder")
        return []

    dirs: list[Path] = []

    # 1) Well-known names first, in priority order.
    for name in CANDIDATE_PATH_NAMES:
        value = getattr(flexlink_paths, name, None)
        if callable(value):
            try:
                value = value()
            except TypeError:
                continue
        if isinstance(value, (str, Path)):
            dirs.append(Path(value))

    # 2) Any other module-level str/Path that points to an existing folder.
    for name, value in vars(flexlink_paths).items():
        if name.startswith("_") or not isinstance(value, (str, Path)):
            continue
        p = Path(value)
        if p.is_dir() and p not in dirs:
            dirs.append(p)

    return dirs


def find_params_file(explicit: Path | None) -> Path:
    if explicit:
        if not explicit.is_file():
            raise FileNotFoundError(f"Parameter file not found: {explicit}")
        return explicit

    search = candidate_dirs_from_flexlink_paths() + [Path(__file__).resolve().parent, Path.cwd()]
    for d in search:
        f = d / PARAMS_FILENAME
        if f.is_file():
            log.info("Using parameters from %s", f)
            return f

    searched = "\n  ".join(str(d) for d in search)
    raise FileNotFoundError(f"{PARAMS_FILENAME} not found. Searched:\n  {searched}")


# --------------------------------------------------------------------------- #
# Loading / validating parameters
# --------------------------------------------------------------------------- #
def load_params(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as fh:
            params = json.load(fh)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: line {exc.lineno}, col {exc.colno}: {exc.msg}") from exc

    if not isinstance(params, dict):
        raise ValueError(f"{path} must contain a JSON object at top level")

    missing = [k for k in REQUIRED_KEYS if k not in params]
    if missing:
        raise ValueError(f"Missing required parameter(s): {', '.join(missing)}")

    return params


def _find_key(obj: Any, key: str) -> tuple[dict | None, Any]:
    """Depth-first search for `key`; returns (containing dict, value)."""
    if isinstance(obj, dict):
        if key in obj:
            return obj, obj[key]
        for v in obj.values():
            parent, val = _find_key(v, key)
            if parent is not None:
                return parent, val
    elif isinstance(obj, list):
        for v in obj:
            parent, val = _find_key(v, key)
            if parent is not None:
                return parent, val
    return None, None


def get_param(params: dict[str, Any], key: str) -> Any:
    parent, value = _find_key(params, key)
    if parent is None:
        raise ValueError(f"Parameter '{key}' not found in {PARAMS_FILENAME}")
    return value


def set_param(params: dict[str, Any], key: str, value: Any) -> None:
    """Update `key` where it already lives (even if nested); else add it to 'Calculated'."""
    parent, _ = _find_key(params, key)
    if parent is None:
        parent = params.get("Calculated") if isinstance(params.get("Calculated"), dict) else params
    parent[key] = value


def rename_param(params: dict[str, Any], old: str, new: str) -> None:
    """Rename key `old` to `new` wherever it lives, keeping its position in the file."""
    parent, _ = _find_key(params, old)
    if parent is None:
        return
    items = list(parent.items())
    new_exists = new in parent
    parent.clear()
    for key, value in items:
        if key != old:
            parent[key] = value
        elif not new_exists:          # if both exist, just drop the old one
            parent[new] = value
    log.info("Renamed '%s' to '%s' in %s", old, new, PARAMS_FILENAME)


def save_params(path: Path, params: dict[str, Any]) -> None:
    """Write JSON atomically so a crash can't leave a half-written file."""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(params, fh, indent=2)
            fh.write("\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


# --------------------------------------------------------------------------- #
# Resource grid
# --------------------------------------------------------------------------- #
def _module_search_dirs() -> list[Path]:
    """Folders to look in for Construct_Grid code: flexlink_paths + script folder."""
    base = candidate_dirs_from_flexlink_paths() + [Path(__file__).resolve().parent, Path.cwd()]
    dirs: list[Path] = []
    for d in base:
        for cand in (d, d / GRID_FOLDER_NAME, d.parent / GRID_FOLDER_NAME):
            if cand.is_dir() and cand not in dirs:
                dirs.append(cand)
    return dirs


def load_module(module_name: str) -> ModuleType:
    """Import <module_name>.py from the Construct_Grid / flexlink folders."""
    if module_name in sys.modules:
        return sys.modules[module_name]

    dirs = _module_search_dirs()
    for d in dirs:
        f = d / f"{module_name}.py"
        if f.is_file():
            # Let the module import its own siblings.
            if str(d) not in sys.path:
                sys.path.insert(0, str(d))
            spec = importlib.util.spec_from_file_location(module_name, f)
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
            log.debug("Loaded %s from %s", module_name, f)
            return module

    searched = "\n  ".join(str(d) for d in dirs) or "(none)"
    raise FileNotFoundError(f"{module_name}.py not found. Searched:\n  {searched}")


def build_resource_grid(params: dict[str, Any]) -> tuple[Any, ModuleType]:
    """Create Resource_Grid(Packet_Len, PacketProfile, ReferenceSymbolProfile) from the JSON."""
    kwargs = {arg: get_param(params, key) for arg, key in GRID_ARGS.items()}
    log.info("Constructing ResourceGrid: %s",
             ", ".join(f"{arg}={val} ({GRID_ARGS[arg]})" for arg, val in kwargs.items()))

    grid_module = load_module(GRID_MODULE)
    grid = getattr(grid_module, GRID_CLASS)(**kwargs)
    log.info("ResourceGrid shape: %s", getattr(grid, "shape", "?"))
    return grid, grid_module


def count_signal_types(resource_grid: Any) -> dict[str, int]:
    """
    Count each signal type in SIG_TYPE_TO_KEY.

    RG_Get_Number_Of_SigTypes(name) gives the numeric code of a signal type
    (Ref_P0 -> 10, Ref_P1 -> 20, DC -> 30, Data -> 40); the grid's count_of(code)
    then counts how many signals in the whole grid carry that code.
    """
    get_code = getattr(load_module(SIGCODE_MODULE), SIGCODE_FUNC)

    counts: dict[str, int] = {}
    for name in SIG_TYPE_TO_KEY:
        code = get_code(name)
        counts[name] = int(resource_grid.count_of(code))
        log.debug("  %-6s code=%d count=%d", name, code, counts[name])
    return counts


def bits_per_data_signal(params: dict[str, Any]) -> tuple[str, int]:
    """Bits per data signal from Defined/Mod: BPSK 1, QPSK 2, 16QAM 4, 64QAM 6."""
    mod = get_param(params, "Mod")
    key = str(mod).strip().upper().replace("-", "").replace(" ", "")
    if key not in MOD_BITS:
        raise ValueError(f"Mod is {mod!r}, expected one of {', '.join(MOD_BITS)}")
    return key, MOD_BITS[key]


def tx_dir(params_file: Path) -> Path:
    """The Tx directory: flexlink_paths.TX, else the folder holding the parameter file."""
    try:
        import flexlink_paths
        tx = getattr(flexlink_paths, "TX", None)
        if tx is not None and Path(tx).is_dir():
            return Path(tx)
    except ImportError:
        pass
    return params_file.resolve().parent


def grid_file_path(params: dict[str, Any], params_file: Path) -> Path:
    """The .h5 file named in File/Grid_Resource_Tx, in the Tx directory."""
    parent, name = _find_key(params, GRID_FILE_KEY)
    if parent is None or not name:
        name = DEFAULT_GRID_FILE
        log.warning("'%s' not in JSON; using %s", GRID_FILE_KEY, name)
    return tx_dir(params_file) / name


def calculate_data_block_size(params: dict[str, Any]) -> int:
    """
    DataBlockSize = EncoderBlockSize * EncoderCodeRate, e.g. 1296 * 5/6 = 1080.
    Stored in the JSON (Calculated/DataBlockSize); saved together with the signal counts.
    """
    rate_text = get_param(params, "EncoderCodeRate")
    block_size = get_param(params, "EncoderBlockSize")

    try:
        rate = Fraction(str(rate_text).replace(" ", ""))
    except (ValueError, ZeroDivisionError):
        raise ValueError(f"EncoderCodeRate must be a fraction like '5/6', got {rate_text!r}") from None
    if not 0 < rate <= 1:
        raise ValueError(f"EncoderCodeRate must be between 0 and 1, got {rate_text!r}")
    if not isinstance(block_size, int) or block_size <= 0:
        raise ValueError(f"EncoderBlockSize must be a positive integer, got {block_size!r}")

    data_bits = block_size * rate
    if data_bits.denominator != 1:
        raise ValueError(f"EncoderBlockSize {block_size} x EncoderCodeRate {rate_text} = "
                         f"{float(data_bits):g} is not a whole number of bits")

    data_block_size = int(data_bits)
    set_param(params, "DataBlockSize", data_block_size)
    log.info("DataBlockSize = %d x %s = %d", block_size, rate_text, data_block_size)
    return data_block_size


def calculate_code_sizes(params: dict[str, Any]) -> None:
    """
    CodeBlockSize      = EncoderBlockSize                  e.g. 1296
    CodeWordSize       = CodeBlockSize * (Repetition + 1)  e.g. 1296 * (15 + 1) = 20736
    """
    code_block_size = get_param(params, "EncoderBlockSize")
    repetition = get_param(params, "Repetition")
    if not isinstance(code_block_size, int) or code_block_size <= 0:
        raise ValueError(f"EncoderBlockSize must be a positive integer, got {code_block_size!r}")
    if not isinstance(repetition, int) or repetition <= 0:
        raise ValueError(f"Repetition must be a positive integer, got {repetition!r}")

    code_word_size = code_block_size * (repetition + 1)

    set_param(params, "CodeBlockSize", code_block_size)
    set_param(params, "CodeWordSize", code_word_size)
    log.info("CodeBlockSize = EncoderBlockSize = %d", code_block_size)
    log.info("CodeWordSize = %d x (%d + 1) = %d", code_block_size, repetition, code_word_size)


def calculate_crc_len(params: dict[str, Any]) -> int:
    """CRClen from the CRC setting: the number at the end of its name ('Gen24' -> 24)."""
    crc = str(get_param(params, "CRC"))
    match = re.fullmatch(r"[A-Za-z_]*(\d+)", crc.strip())
    if not match or int(match.group(1)) <= 0:
        raise ValueError(f"CRC must be a name ending in its length in bits, like 'Gen24', got {crc!r}")

    crc_len = int(match.group(1))
    set_param(params, "CRClen", crc_len)
    log.info("CRClen = %d (CRC %s)", crc_len, crc)
    return crc_len


def calculate_transport_block_size(params: dict[str, Any], data_block_size: int, crc_len: int) -> int:
    """TransportBlockSize = DataBlockSize - CRClen, e.g. 1080 - 24 = 1056."""
    transport_block_size = data_block_size - crc_len
    if transport_block_size <= 0:
        raise ValueError(f"TransportBlockSize = {data_block_size} - {crc_len} "
                         f"= {transport_block_size} must be positive")
    set_param(params, "TransportBlockSize", transport_block_size)
    log.info("TransportBlockSize = %d - %d = %d", data_block_size, crc_len, transport_block_size)
    return transport_block_size


def calculate_data_blocks(params: dict[str, Any]) -> None:
    """
    AvailableDataBlocks = how many whole CodeWords fit into NumberOfDataSigs
    ZeroPadding         = the remainder
    e.g. 46224 // 20736 = 2, 46224 % 20736 = 4752
    Fails if CodeWordSize is bigger than NumberOfDataSigs (no code word fits).
    """
    data_sigs = get_param(params, "NumberOfDataSigs")
    code_word_size = get_param(params, "CodeWordSize")
    if not isinstance(code_word_size, int) or code_word_size <= 0:
        raise ValueError(f"CodeWordSize must be a positive integer, got {code_word_size!r}")
    if code_word_size > data_sigs:
        raise ValueError(f"CodeWordSize {code_word_size} is bigger than NumberOfDataSigs {data_sigs}: "
                         f"the packet can't hold a single code word. Use a longer PacketLength, "
                         f"fewer Repetitions or a smaller EncoderBlockSize.")

    available_data_blocks, zero_padding = divmod(data_sigs, code_word_size)

    scrambled_sequence_size = available_data_blocks * code_word_size + zero_padding
    if scrambled_sequence_size != data_sigs:
        raise ValueError(f"ScrambledSequenceSize {available_data_blocks} x {code_word_size} + "
                         f"{zero_padding} = {scrambled_sequence_size} does not equal "
                         f"NumberOfDataSigs {data_sigs}")

    set_param(params, "AvailableDataBlocks", available_data_blocks)
    set_param(params, "ZeroPadding", zero_padding)
    rename_param(params, "ScrambledCodeWordSize", "ScrambledSequenceSize")
    set_param(params, "ScrambledSequenceSize", scrambled_sequence_size)
    log.info("AvailableDataBlocks = %d // %d = %d", data_sigs, code_word_size, available_data_blocks)
    log.info("ZeroPadding = %d %% %d = %d", data_sigs, code_word_size, zero_padding)
    log.info("ScrambledSequenceSize = %d x %d + %d = %d (= NumberOfDataSigs)",
             available_data_blocks, code_word_size, zero_padding, scrambled_sequence_size)


def prepare_resource_grid(params_file: Path, params: dict[str, Any], dry_run: bool) -> None:
    """Build the grid, count signal types, calculate AvailableDataBlocks/ZeroPadding, save .h5 and JSON."""
    if not dry_run and importlib.util.find_spec("h5py") is None:
        raise ValueError("Saving the ResourceGrid needs h5py - install it with: pip install h5py")

    grid, grid_module = build_resource_grid(params)
    counts = count_signal_types(grid)

    # Each Data signal carries 1, 2, 4 or 6 bits depending on the modulation.
    mod, bits = bits_per_data_signal(params)
    data_cells = counts["Data"]
    counts["Data"] = data_cells * bits
    log.info("Data signals in grid = %d, x %d bit(s) per signal (%s) = %d",
             data_cells, bits, mod, counts["Data"])

    for sig_type, key in SIG_TYPE_TO_KEY.items():
        set_param(params, key, counts[sig_type])
        log.info("  %-20s = %d", key, counts[sig_type])

    calculate_data_blocks(params)

    h5_path = grid_file_path(params, params_file)

    if dry_run:
        log.info("Dry run: %s and %s not written", params_file.name, h5_path.name)
        return

    # Save the grid first: if that fails, the JSON is left untouched.
    grid.save_h5(str(h5_path))
    log.info("ResourceGrid saved to %s", h5_path)

    save_params(params_file, params)
    log.info("Calculated values saved to %s", params_file)


# --------------------------------------------------------------------------- #
# Transmitter
# --------------------------------------------------------------------------- #
class FlexlinkTransmitter:
    """
    Wraps the transmitter hardware/driver.

    The methods below are the integration points: replace the bodies of
    connect(), apply() and verify() with the real Flexlink driver/API calls.
    """

    def __init__(self, params: dict[str, Any], dry_run: bool = False):
        self.params = params
        self.dry_run = dry_run
        self.connected = False

    def connect(self) -> None:
        log.info("Connecting to transmitter%s", " (dry run)" if self.dry_run else "")
        if self.dry_run:
            return
        # TODO: open the device / socket / serial port here, e.g.
        # self.dev = flexlink_driver.open(self.params["device"])
        self.connected = True

    def settings(self) -> dict[str, Any]:
        """Flatten the 'Defined' and 'Calculated' sections, dropping *_comment entries."""
        flat: dict[str, Any] = {}
        for section in ("Defined", "Calculated"):
            for key, value in self.params.get(section, {}).items():
                if not key.endswith("_comment"):
                    flat[key] = value
        return flat

    def apply(self) -> None:
        for key, value in self.settings().items():
            log.info("  set %-28s = %s", key, value)
            if self.dry_run:
                continue
            # TODO: write each parameter to the transmitter, e.g.
            # self.dev.set(key, value)

    def verify(self) -> bool:
        if self.dry_run:
            return True
        # TODO: read back and compare, e.g.
        # return all(self.dev.get(k) == v for k, v in self.params.items())
        return True

    def close(self) -> None:
        if self.connected:
            # TODO: self.dev.close()
            self.connected = False
            log.info("Transmitter connection closed")

    def __enter__(self) -> "FlexlinkTransmitter":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# --------------------------------------------------------------------------- #
# Control field
# --------------------------------------------------------------------------- #
def rebuild_control_field(params_file: Path, verbose: bool = False) -> bool:
    """Run Build_Control_Field.py so the control field matches the new Tx parameters."""
    try:
        from flexlink_paths import CONSTRUCT_CONTROL_FIELD
        folder = Path(CONSTRUCT_CONTROL_FIELD)
    except ImportError:
        folder = Path(__file__).resolve().parent.parent / "Flexlink_BasicModules" / "Construct_Control_Field"
    script = folder / "Build_Control_Field.py"
    if not script.is_file():
        log.error("Control field not rebuilt: %s not found", script)
        return False
    spec = importlib.util.spec_from_file_location("Build_Control_Field", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    log.info("Rebuilding the control field from %s", params_file.name)
    ok, lines = module.build_control_field(params_file)
    for line in lines:
        if verbose or not ok:
            log.info("  %s", line)
    if ok:
        wrote = [ln for ln in lines if ln.startswith("Wrote")]
        log.info("Control field rebuilt%s", f": {wrote[-1][6:]}" if wrote else "")
    else:
        log.error("Control field rebuild stopped with an error (output above)")
    return ok


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Initialize a Flexlink transmitter.")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (overrides flexlink_paths)")
    ap.add_argument("--dry-run", action="store_true", help="build the grid and compute counts, but don't write the JSON/.h5 or touch hardware")
    ap.add_argument("--no-control-field", action="store_true",
                    help="do not rebuild the control field after initializing")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    try:
        params_file = find_params_file(args.params)
        params = load_params(params_file)
        log.info("Loaded %s (sections: %s)", params_file.name, ", ".join(params))

        data_block_size = calculate_data_block_size(params)
        crc_len = calculate_crc_len(params)
        calculate_transport_block_size(params, data_block_size, crc_len)
        calculate_code_sizes(params)
        prepare_resource_grid(params_file, params, dry_run=args.dry_run)

        with FlexlinkTransmitter(params, dry_run=args.dry_run) as tx:
            tx.apply()
            if not tx.verify():
                log.error("Verification failed: transmitter readback does not match parameters")
                return 2

        if args.dry_run:
            log.info("Dry run: control field not rebuilt")
        elif args.no_control_field:
            log.info("--no-control-field: control field not rebuilt")
        elif not rebuild_control_field(params_file, args.verbose):
            return 4

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:  # unexpected - show the traceback
        log.exception("Transmitter initialization failed")
        return 3

    log.info("Transmitter initialized successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
