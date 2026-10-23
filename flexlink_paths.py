"""Central place for Flexlink_py project paths.

Import what you need in any script or module, e.g.
    from flexlink_paths import TX, CONSTRUCT_GRID, project_path
and pass the resulting Path objects into functions.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent          # .../Flexlink_py

# ---- Top-level folders ------------------------------------------------------
APP               = ROOT / "App"
DATABASE          = ROOT / "Database"
DOCS              = ROOT / "Docs"
REFERENCE_SIGNALS = ROOT / "ReferenceSignals"   # prebuilt Ref_P0_* / Ref_P1_* files
RX                = ROOT / "Rx"
TX                = ROOT / "Tx"
TX_CONTROL_SIGNALS = TX / "ControlSignals"         # Parameters_Control_Field.json and control field .h5 files

# ---- Flexlink_BasicModules --------------------------------------------------
BASIC_MODULES    = ROOT / "Flexlink_BasicModules"

BCC_ENCODER      = BASIC_MODULES / "BCC_Encoder"
CHANNEL_DEFECTS  = BASIC_MODULES / "Channel_Defects"
CONSTRUCT_CONTROL_FIELD = BASIC_MODULES / "Construct_Control_Field"
CONSTRUCT_GRID   = BASIC_MODULES / "Construct_Grid"
CRC_GEN          = BASIC_MODULES / "CRC_Gen"

INTERLEAVER                  = BASIC_MODULES / "Interleaver"
INTERLEAVER_INDEX_GENERATION = INTERLEAVER / "IndexGeneration"
INTERLEAVER_TEST_GENERATION  = INTERLEAVER / "TestGeneration"

LDPC_ENCODER                     = BASIC_MODULES / "LDPC_Encoder"
LDPC_ENCODER_MATRIX_GENERATION_G = LDPC_ENCODER / "MatrixGeneration_G"
LDPC_ENCODER_MATRIX_GENERATION_H = LDPC_ENCODER / "MatrixGeneration_H"

OFDM_MODULATOR = BASIC_MODULES / "OFDM_Modulator"

QAM_MAPPER = BASIC_MODULES / "QAM_Mapper"

RATE_MATCHING                         = BASIC_MODULES / "Rate_Matching"
RATE_MATCHING_CREATE_ORIGINAL_SEQUENCE = RATE_MATCHING / "CreateOriginalSequence"
RATE_MATCHING_TEST_GENERATION          = RATE_MATCHING / "TestGeneration"

SCRAMBLER                 = BASIC_MODULES / "Scrambler"
SCRAMBLER_TEST_GENERATION = SCRAMBLER / "TestGeneration"


def project_path(*parts) -> Path:
    """Build any path relative to the project root: project_path('Tx', 'x.json')."""
    return ROOT.joinpath(*parts)


# ---- Per-ray (receive antenna) file names ------------------------------------
MAX_RAYS = 4


def ray_file_name(name: str, ray: int) -> str:
    """File name for one ray: ('Grid_Payload_Rx.h5', 2) -> 'Grid_Payload_Rx_Ray2.h5'.

    Every per-ray Rx file carries its ray number, also when there is only one (_Ray1).
    """
    if not isinstance(ray, int) or not 1 <= ray <= MAX_RAYS:
        raise ValueError(f"ray {ray!r} must be 1 to {MAX_RAYS}")
    p = Path(name)
    return f"{p.stem}_Ray{ray}{p.suffix}"
