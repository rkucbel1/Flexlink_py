"""GenerateCRC.py
 
Compute the CRC remainder of a bit vector for one of several generator
polynomials (MATLAB-style coefficient vectors, highest power first).
"""
import numpy as np
 
CRC_POLYNOMIALS = {
    "Gen24": [1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
    "Gen16": [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    "Gen10": [1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1],
    "Gen3":  [1, 1, 0, 1],
}
 
 
def GenerateCRC(Message, CRC_Gen):
    """Return Remainder: the CRC of Message (bit vector, any length) as a
    numpy row vector of shape (1, n), MSB first, where n is the polynomial
    degree (24, 16, 10 or 3).
 
    Zero initial register, no input/output reflection, no final XOR
    (equivalent to MATLAB's crcgen / comm.CRCGenerator defaults).
    """
    if CRC_Gen not in CRC_POLYNOMIALS:
        raise ValueError(
            f"CRC_Gen must be one of {list(CRC_POLYNOMIALS)}, got {CRC_Gen!r}"
        )
 
    bits = np.asarray(Message).ravel()
    if bits.size and not np.isin(bits, (0, 1)).all():
        raise ValueError("Message must contain only 0s and 1s")
 
    poly_vec = CRC_POLYNOMIALS[CRC_Gen]
    n = len(poly_vec) - 1                                 # CRC width
    poly = int("".join(map(str, poly_vec[1:])), 2)        # drop implicit leading term
    mask = (1 << n) - 1
    top_bit = n - 1
 
    crc = 0
    for b in bits.astype(int):
        feedback = ((crc >> top_bit) & 1) ^ b
        crc = (crc << 1) & mask
        if feedback:
            crc ^= poly
 
    # Row vector of shape (1, n), like MATLAB's 1-by-n output
    Remainder = np.array([[(crc >> (n - 1 - i)) & 1 for i in range(n)]], dtype=int)
    return Remainder
 
 
if __name__ == "__main__":
    # Test every polynomial with the ASCII string '123456789'
    Message = np.unpackbits(np.frombuffer(b"123456789", dtype=np.uint8))  # 1x72 bits, MSB first
    print("Message (", Message.size, "bits ):", "".join(map(str, Message)))
    print()
 
    for gen in CRC_POLYNOMIALS:
        Remainder = GenerateCRC(Message, gen)
        bits = "".join(map(str, Remainder.ravel()))
        hex_width = (Remainder.size + 3) // 4
        value = int(bits, 2)
        # Appending the remainder to the message must give an all-zero remainder
        check = GenerateCRC(np.concatenate([Message, Remainder.ravel()]), gen)
        print(f"{gen}: shape={Remainder.shape}")
        print(f"  Remainder = [{' '.join(map(str, Remainder.ravel()))}]")
        print(f"  hex = 0x{value:0{hex_width}X}   zero-check: {'PASS' if not check.any() else 'FAIL'}")
