"""ConvolutionalEncoder.py

Binary convolutional code (BCC) encoder and Viterbi decoder, plain NumPy
(no commpy). Rate 1/n, any constraint length; Flexlink uses the IEEE 802.11
code: ConstraintLength 7, PolynomialsOct [133, 171], rate 1/2.

Conventions
-----------
- Polynomials are octal digits, MATLAB style (133 means 0o133). The most
  significant of the K polynomial bits taps the current input bit, the least
  significant taps the oldest bit (same as MATLAB poly2trellis and IEEE 802.11).
- Output is interleaved per input bit: [A0 B0 A1 B1 ...] for [g0 g1].
- Encoder state = the last K-1 input bits, most recent in the most significant
  position.

Functions
---------
ConvolutionalEncoder(InputBits, ConstraintLength, PolynomialsOct, TailBitingFlag, ModeString)
ViterbiDecoder(EncodedBits, ConstraintLength, PolynomialsOct, TailBitingBool, ModeString,
               EndState=None)
"""
import numpy as np

VALID_MODES = ("Hard", "SoftNonInverting", "SoftInverting")


# --------------------------------------------------------------------------- #
# Code description
# --------------------------------------------------------------------------- #
def _octal_gens(PolynomialsOct, K):
    """Octal digits (e.g. [133, 171]) -> list of integer generator masks, checked against K."""
    gens = []
    for p in np.asarray(PolynomialsOct).ravel():
        s = str(int(p))
        if any(ch not in "01234567" for ch in s):
            raise ValueError(f"Polynomial {s} is not an octal number")
        g = int(s, 8)
        if g >= (1 << K):
            raise ValueError(f"Polynomial {s} (octal) is longer than ConstraintLength {K}")
        if g == 0:
            raise ValueError("Polynomials must not be 0")
        gens.append(g)
    if not gens:
        raise ValueError("At least one polynomial is needed")
    return gens


def _trellis(K, gens):
    """Return (outputs, next_state).

    outputs[s, b, j]  output bit j (0/1) when input b arrives in state s
    next_state[s, b]  state after that input
    """
    S = 1 << (K - 1)
    s = np.arange(S)[:, None]
    b = np.arange(2)[None, :]
    reg = (b << (K - 1)) | s                    # current bit in the MSB, oldest in the LSB
    outputs = np.stack([np.bitwise_count(reg & g) & 1 if hasattr(np, "bitwise_count")
                        else _parity(reg & g) for g in gens], axis=-1).astype(np.int8)
    next_state = reg >> 1
    return outputs, next_state


def _parity(x):
    x = np.asarray(x, dtype=np.int64).copy()
    p = np.zeros_like(x)
    while np.any(x):
        p ^= x & 1
        x >>= 1
    return p


def _check_mode(ModeString):
    if ModeString not in VALID_MODES:
        raise ValueError(f"ModeString must be one of {VALID_MODES}, got {ModeString!r}")


# --------------------------------------------------------------------------- #
# Encoder
# --------------------------------------------------------------------------- #
def ConvolutionalEncoder(InputBits, ConstraintLength, PolynomialsOct, TailBitingFlag, ModeString):
    """Return EncodedBits: 1-by-N row vector, interleaved [A0 B0 A1 B1 ...].

    InputBits         bit vector
    ConstraintLength  constraint length K
    PolynomialsOct    generator polynomials as octal digits, e.g. [133 171]
    TailBitingFlag    True  -> tail-biting (initial state = last K-1 input bits)
                      False -> zero initial state, no tail bits added
                               (append K-1 zeros yourself to end in state 0)
    ModeString        "Hard"             -> bits 0/1
                      "SoftNonInverting" -> bit 0 -> -1, bit 1 -> +1
                      "SoftInverting"    -> bit 0 -> +1, bit 1 -> -1
    """
    _check_mode(ModeString)
    K = int(ConstraintLength)
    gens = _octal_gens(PolynomialsOct, K)
    bits = np.asarray(InputBits).ravel().astype(np.int64)
    if bits.size and not np.isin(bits, (0, 1)).all():
        raise ValueError("InputBits must contain only 0s and 1s")

    outputs, next_state = _trellis(K, gens)
    state = 0
    if TailBitingFlag:
        if bits.size < K - 1:
            raise ValueError("Tail-biting needs at least ConstraintLength-1 input bits")
        for b in bits[bits.size - (K - 1):]:           # oldest first; ends with the last bit in the MSB
            state = int(next_state[state, b])

    coded = np.empty((bits.size, len(gens)), dtype=np.int8)
    for i, b in enumerate(bits):
        coded[i] = outputs[state, b]
        state = int(next_state[state, b])
    coded = coded.ravel()

    if ModeString == "Hard":
        out = coded.astype(int)
    elif ModeString == "SoftNonInverting":
        out = 2.0 * coded - 1.0
    else:  # SoftInverting
        out = 1.0 - 2.0 * coded
    return out.reshape(1, -1)


# --------------------------------------------------------------------------- #
# Viterbi decoder
# --------------------------------------------------------------------------- #
def _viterbi(sym, K, gens, start_metric, end_state):
    """Maximum-correlation Viterbi.

    sym           (steps, n) float, positive = bit 1
    start_metric  (S,) initial path metrics
    end_state     None (best final state) or the state the path must end in
    Returns the decoded bits (steps,).
    """
    steps, n = sym.shape
    S = 1 << (K - 1)
    outputs, _ = _trellis(K, gens)
    signs = 2.0 * outputs - 1.0                             # (S, 2, n): +1 for output bit 1

    # Every next state ns has one input bit b = MSB of ns and two predecessors.
    ns = np.arange(S)
    b_of = ns >> (K - 2)
    pred = np.stack([((ns & ((1 << (K - 2)) - 1)) << 1) | x for x in (0, 1)], axis=1)   # (S, 2)

    metric = start_metric.astype(float).copy()
    choice = np.empty((steps, S), dtype=np.int8)
    for t in range(steps):
        bm = signs @ sym[t]                                  # (S, 2) branch metrics
        cand = metric[pred] + bm[pred, b_of[:, None]]        # (S, 2)
        c = np.argmax(cand, axis=1)
        choice[t] = c
        metric = cand[ns, c]
        metric -= metric.max()                               # keep numbers small

    state = int(np.argmax(metric)) if end_state is None else int(end_state)
    if end_state is not None and not (0 <= state < S):
        raise ValueError(f"EndState must be 0 .. {S - 1}, got {end_state}")
    dec = np.empty(steps, dtype=np.int8)
    for t in range(steps - 1, -1, -1):
        dec[t] = state >> (K - 2)
        state = int(pred[state, choice[t, state]])
    return dec


def ViterbiDecoder(EncodedBits, ConstraintLength, PolynomialsOct, TailBitingBool, ModeString,
                   EndState=None):
    """Return DecodedBits: 1-by-M row vector (M = len(EncodedBits) / number of polynomials).

    EncodedBits       encoded vector, interleaved [A0 B0 A1 B1 ...]
    ConstraintLength  constraint length K
    PolynomialsOct    generator polynomials as octal digits, e.g. [133 171]
    TailBitingBool    True  -> tail-biting block (wrap-around decoding)
                      False -> zero initial state; no tail bits are removed
    ModeString        "Hard"             -> input bits 0/1
                      "SoftNonInverting" -> input -1 = bit 0, +1 = bit 1 (e.g. scaled LLR log P1/P0)
                      "SoftInverting"    -> input +1 = bit 0, -1 = bit 1 (e.g. LLR log P0/P1)
                      Soft values may have any magnitude; larger means more reliable.
    EndState          only when TailBitingBool is False:
                      None -> the path may end in any state (default)
                      0    -> the path must end in state 0, i.e. the input ended with
                              K-1 zero tail bits (as in the Flexlink control field)
    """
    _check_mode(ModeString)
    K = int(ConstraintLength)
    gens = _octal_gens(PolynomialsOct, K)
    n = len(gens)
    S = 1 << (K - 1)

    rx = np.asarray(EncodedBits, dtype=float).ravel()
    if rx.size % n != 0:
        raise ValueError(f"EncodedBits length {rx.size} is not a multiple of {n} polynomials")
    if ModeString == "Hard":
        if rx.size and not np.isin(rx, (0, 1)).all():
            raise ValueError("Hard-mode EncodedBits must contain only 0s and 1s")
        sym = 2.0 * rx - 1.0
    elif ModeString == "SoftNonInverting":
        sym = rx
    else:
        sym = -rx
    sym = sym.reshape(-1, n)
    steps = sym.shape[0]
    if steps == 0:
        return np.zeros((1, 0), dtype=int)

    if TailBitingBool:
        if EndState is not None:
            raise ValueError("EndState is only used when TailBitingBool is False")
        # Wrap-around decoding: extend the block cyclically on both sides, start in
        # every state with equal metric, and keep the middle part.
        P = 5 * K
        idx = np.arange(-P, steps + P) % steps
        dec = _viterbi(sym[idx], K, gens, np.zeros(S), None)[P:P + steps]
    else:
        start = np.full(S, -1e18)
        start[0] = 0.0
        dec = _viterbi(sym, K, gens, start, EndState)
    return dec.astype(int).reshape(1, -1)


if __name__ == "__main__":
    TransportBlkSize = 54
    ZeroPad = np.array([[0, 0, 0, 0, 0, 0]])   # 1x6 row vector, like MATLAB [0 0 0 0 0 0]
    CRCdummy = np.array([[1, 1, 0, 0, 0, 1, 1, 0, 1, 1]])   # 1x10 row vector

    TransportBlk = np.array([[0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1,
                              0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1,
                              1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]])   # 1x54 row vector
    assert TransportBlk.shape == (1, TransportBlkSize)

    DataBlk = np.concatenate([TransportBlk, CRCdummy, ZeroPad], axis=1)   # [TransportBlk CRCdummy ZeroPad], 1x70

    PolynomialsOct = [133, 171]   # octal digits, as in MATLAB (not Python 0o literals)
    ConstraintLength = 7

    CodeBlk = ConvolutionalEncoder(DataBlk, ConstraintLength, PolynomialsOct, False, 'Hard')
    DecodedDataBlk = ViterbiDecoder(CodeBlk, ConstraintLength, PolynomialsOct, False, 'Hard')
    print("DataBlk and DecodedDataBlk are the same:", np.array_equal(DataBlk, DecodedDataBlk))

    # Impulse response must equal the generator taps (MSB first)
    imp = ConvolutionalEncoder([1, 0, 0, 0, 0, 0, 0], 7, [133, 171], False, 'Hard').ravel()
    assert imp[0::2].tolist() == [1, 0, 1, 1, 0, 1, 1] and imp[1::2].tolist() == [1, 1, 1, 1, 0, 0, 1]

    # Soft decoding through noise, tail-biting, and corrected hard errors
    rng = np.random.default_rng(1)
    msg = np.concatenate([rng.integers(0, 2, 64), np.zeros(6, int)])
    tx = ConvolutionalEncoder(msg, 7, [133, 171], False, 'SoftNonInverting').ravel()
    rx = tx + rng.normal(0, 0.6, tx.size)
    assert np.array_equal(ViterbiDecoder(rx, 7, [133, 171], False, 'SoftNonInverting', EndState=0).ravel(), msg)
    assert np.array_equal(ViterbiDecoder(-rx, 7, [133, 171], False, 'SoftInverting', EndState=0).ravel(), msg)
    tb = rng.integers(0, 2, 40)
    c = ConvolutionalEncoder(tb, 7, [133, 171], True, 'Hard').ravel()
    c[[3, 30, 61]] ^= 1
    assert np.array_equal(ViterbiDecoder(c, 7, [133, 171], True, 'Hard').ravel(), tb)
    print("Self-test passed")
