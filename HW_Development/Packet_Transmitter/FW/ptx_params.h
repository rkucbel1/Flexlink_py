/* ptx_params.h - Flexlink transmitter settings on the Pluto's ARM: the Defined sections of
 * Tx/Parameters_Tx_payload.json and Tx/ControlSignals/Parameters_Control_Field.json -> checks,
 * Calculated values, control-field message, and the 6-word parameter header of the PL
 * (RTL/ptx_core.v). Bit-true twin of PC/ptx_reference.py (TxConfig, param_header, cf_message);
 * no hardware dependency (FW/ptx_params_test.c checks it against the model on the PC). */
#ifndef PTX_PARAMS_H
#define PTX_PARAMS_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    /* Parameters_Tx_payload.json Defined */
    int mod;              /* 0 BPSK, 1 QPSK, 2 16QAM, 3 64QAM */
    int ref_sym, ref_sig, plen, n, rate;   /* rate code 0 1/2, 1 2/3, 2 3/4, 3 5/6 */
    int rep, port, c_init, client, antennas, use_pre, use_cf, clk_cnt;
    uint32_t id1, id2;
    /* Parameters_Control_Field.json Defined */
    uint32_t cf_id1, cf_cdat;
    /* transmission (not in the JSON files) */
    int gap;              /* idle samples after each packet */
    int gain_pre, gain_cf, gain_pl;        /* 0: the PL's own (13 dB backoff) */
} ptx_cfg;

typedef struct {
    int nb, nref, data_res, data_bits, k, tb_size, cw_size, ndb, zero_pad, words_per_block,
        words_per_packet, ref_p0, ref_p1, dc;
    int message[54];      /* ControlMessage, MSB first */
} ptx_derived;

/* Parse {"Tx": {...Defined...}, "ControlField": {...Defined...}, "Gap": n, "Gains": {...}}
 * (a whole parameter file - with "Defined" inside - is accepted too). Returns 0, or -1 with a
 * message in err. */
int ptx_cfg_from_json(const char *text, ptx_cfg *cfg, char *err, size_t errlen);
/* Checks the settings and computes everything else. Returns 0, or -1 with a message. */
int ptx_derive(const ptx_cfg *cfg, ptx_derived *d, char *err, size_t errlen);
/* The PL header for a record of n_packets packets. */
void ptx_header(const ptx_cfg *cfg, uint32_t n_packets, uint64_t w[6]);
/* The reply: the Calculated sections of both files plus what the PC needs to send blocks. */
int ptx_reply_json(const ptx_cfg *cfg, const ptx_derived *d, char *out, size_t outlen);

#endif
