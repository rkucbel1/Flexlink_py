/* ptx_params.c - see ptx_params.h */
#include "ptx_params.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------ */
/* minimal JSON (objects, arrays, strings, numbers, true / false / null)                       */

typedef enum { J_NULL, J_BOOL, J_NUM, J_STR, J_ARR, J_OBJ } jtype;
typedef struct jval {
    jtype t;
    double num;
    char *str;
    int n;
    struct jval *items;   /* n values (array / object) */
    char **keys;          /* n keys (object) */
} jval;

static void jfree(jval *v)
{
    int i;
    if (!v) return;
    for (i = 0; i < v->n; i++) {
        jfree(&v->items[i]);
        if (v->keys) free(v->keys[i]);
    }
    free(v->items);
    free(v->keys);
    free(v->str);
}

static void skip_ws(const char **p) { while (**p && isspace((unsigned char)**p)) (*p)++; }

static int parse_value(const char **p, jval *v, int depth);

static char *parse_string(const char **p)
{
    const char *s = *p + 1;
    size_t cap = 32, n = 0;
    char *out = malloc(cap);
    if (!out) return NULL;
    while (*s && *s != '"') {
        char c = *s++;
        if (c == '\\') {
            c = *s++;
            switch (c) {
            case 'n': c = '\n'; break;
            case 't': c = '\t'; break;
            case 'r': c = '\r'; break;
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'u': c = '?'; if (strlen(s) >= 4) s += 4; break;
            case 0: free(out); return NULL;
            default: break;   /* \" \\ \/ */
            }
        }
        if (n + 2 > cap) {
            char *t = realloc(out, cap *= 2);
            if (!t) { free(out); return NULL; }
            out = t;
        }
        out[n++] = c;
    }
    if (*s != '"') { free(out); return NULL; }
    out[n] = 0;
    *p = s + 1;
    return out;
}

static int parse_items(const char **p, jval *v, int depth, int obj)
{
    char close = obj ? '}' : ']';
    (*p)++;
    skip_ws(p);
    v->t = obj ? J_OBJ : J_ARR;
    if (**p == close) { (*p)++; return 0; }
    for (;;) {
        jval *it;
        char **kt;
        char *key = NULL;
        skip_ws(p);
        if (obj) {
            if (**p != '"' || !(key = parse_string(p))) return -1;
            skip_ws(p);
            if (**p != ':') { free(key); return -1; }
            (*p)++;
        }
        it = realloc(v->items, (v->n + 1) * sizeof *it);
        if (!it) { free(key); return -1; }
        v->items = it;
        memset(&v->items[v->n], 0, sizeof *it);
        if (obj) {
            kt = realloc(v->keys, (v->n + 1) * sizeof *kt);
            if (!kt) { free(key); return -1; }
            v->keys = kt;
            v->keys[v->n] = key;
        }
        v->n++;
        if (parse_value(p, &v->items[v->n - 1], depth + 1)) return -1;
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (**p == close) { (*p)++; return 0; }
        return -1;
    }
}

static int parse_value(const char **p, jval *v, int depth)
{
    char *end;
    if (depth > 16) return -1;
    skip_ws(p);
    switch (**p) {
    case '{': return parse_items(p, v, depth, 1);
    case '[': return parse_items(p, v, depth, 0);
    case '"': v->t = J_STR; v->str = parse_string(p); return v->str ? 0 : -1;
    case 't': if (strncmp(*p, "true", 4)) return -1; v->t = J_BOOL; v->num = 1; *p += 4; return 0;
    case 'f': if (strncmp(*p, "false", 5)) return -1; v->t = J_BOOL; v->num = 0; *p += 5; return 0;
    case 'n': if (strncmp(*p, "null", 4)) return -1; v->t = J_NULL; *p += 4; return 0;
    default:
        v->num = strtod(*p, &end);
        if (end == *p) return -1;
        v->t = J_NUM;
        *p = end;
        return 0;
    }
}

static const jval *jget(const jval *o, const char *key)
{
    int i;
    if (!o || o->t != J_OBJ) return NULL;
    for (i = 0; i < o->n; i++)
        if (!strcmp(o->keys[i], key)) return &o->items[i];
    return NULL;
}

/* ------------------------------------------------------------------------------------------ */

static void seterr(char *err, size_t len, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(err, len, fmt, ap);
    va_end(ap);
}

/* number (or numeric string) field with a default; -1 on a wrong type */
static int get_num(const jval *o, const char *key, double def, double *out)
{
    const jval *v = jget(o, key);
    if (!v) { *out = def; return 0; }
    if (v->t == J_NUM || v->t == J_BOOL) { *out = v->num; return 0; }
    if (v->t == J_STR) {
        char *end;
        *out = strtod(v->str, &end);
        return (end != v->str && *end == 0) ? 0 : -1;
    }
    return -1;
}

static const char *get_str(const jval *o, const char *key, const char *def)
{
    const jval *v = jget(o, key);
    if (!v) return def;
    return v->t == J_STR ? v->str : NULL;
}

static int is_int(double x, double lo, double hi) { return x == floor(x) && x >= lo && x <= hi; }

static int parse_tx(const jval *d, ptx_cfg *c, char *err, size_t el)
{
    static const char *mods[] = {"BPSK", "QPSK", "16QAM", "64QAM"};
    static const char *rates[] = {"1/2", "2/3", "3/4", "5/6"};
    const char *s;
    double x;
    int i;
#define NUM(key, def, lo, hi, dst) do { \
        if (get_num(d, key, def, &x) || !is_int(x, lo, hi)) { seterr(err, el, "%s: missing or invalid", key); return -1; } \
        dst = (int)x; } while (0)
    s = get_str(d, "Mod", NULL);
    for (c->mod = -1, i = 0; s && i < 4; i++) if (!strcmp(s, mods[i])) c->mod = i;
    if (c->mod < 0) { seterr(err, el, "Mod: missing or not BPSK, QPSK, 16QAM, 64QAM"); return -1; }
    s = get_str(d, "EncoderCodeRate", NULL);
    for (c->rate = -1, i = 0; s && i < 4; i++) if (!strcmp(s, rates[i])) c->rate = i;
    if (c->rate < 0) { seterr(err, el, "EncoderCodeRate: missing or not 1/2, 2/3, 3/4, 5/6"); return -1; }
    NUM("RefSymbolSpacing", -1, 0, 1e9, c->ref_sym);
    NUM("RefSignalSpacing", -1, 0, 1e9, c->ref_sig);
    NUM("PacketLength", -1, 0, 1e9, c->plen);
    NUM("EncoderBlockSize", -1, 0, 1e9, c->n);
    NUM("Repetition", -1, 0, 1e9, c->rep);
    NUM("Port", 0, 0, 1, c->port);
    NUM("C_init_Dat", -1, 0, 255, c->c_init);
    NUM("NumberOfAntennas", 1, 1, 2, c->antennas);
    NUM("Use_Preamble", 1, 0, 1, c->use_pre);
    NUM("Use_ControlField", 1, 0, 1, c->use_cf);
    NUM("Tx_Ref_Clk_Cnt", 0, 0, 16383, c->clk_cnt);
    if (get_num(d, "C_init_ID1", 129, &x) || !is_int(x, 0, 2147483647.0)) { seterr(err, el, "C_init_ID1: invalid"); return -1; }
    c->id1 = (uint32_t)x;
    if (get_num(d, "C_init_ID2", 128, &x) || !is_int(x, 0, 2147483647.0)) { seterr(err, el, "C_init_ID2: invalid"); return -1; }
    c->id2 = (uint32_t)x;
    s = get_str(d, "ClientFlag", "Service");
    if (!s || (strcmp(s, "Service") && strcmp(s, "Client"))) { seterr(err, el, "ClientFlag: not Service or Client"); return -1; }
    c->client = !strcmp(s, "Client");
    s = get_str(d, "CRC", "Gen24");
    if (!s || strcmp(s, "Gen24")) { seterr(err, el, "CRC: the hardware has Gen24 only"); return -1; }
    if (get_num(d, "SampleRate", 40e6, &x) || x != 40e6) { seterr(err, el, "SampleRate: the hardware runs at 40 Msps only"); return -1; }
#undef NUM
    return 0;
}

static int parse_cf(const jval *d, ptx_cfg *c, char *err, size_t el)
{
    struct { const char *key; double val; } nums[] = {
        {"ControlMessageSize", 54}, {"ConstraintLength", 7}, {"TailBiting", 0}, {"PacketLength", 4},
        {"RefSignalSpacing", 3}, {"RefSymbolSpacing", 3}, {"Port", 0}, {"SampleRate", 40e6}};
    struct { const char *key, *val; } strs[] = {
        {"CRC", "Gen10"}, {"Encoder", "BCC"}, {"EncoderCodeRate", "1/2"}, {"Mod", "BPSK"}};
    const jval *pol = jget(d, "Polynomials");
    double x;
    size_t i;
    for (i = 0; i < sizeof nums / sizeof nums[0]; i++) {
        if (get_num(d, nums[i].key, nums[i].val, &x) || x != nums[i].val) {
            seterr(err, el, "Parameters_Control_Field %s: the hardware supports %g only", nums[i].key, nums[i].val);
            return -1;
        }
    }
    for (i = 0; i < sizeof strs / sizeof strs[0]; i++) {
        const char *s = get_str(d, strs[i].key, strs[i].val);
        if (!s || strcmp(s, strs[i].val)) {
            seterr(err, el, "Parameters_Control_Field %s: the hardware supports %s only", strs[i].key, strs[i].val);
            return -1;
        }
    }
    if (pol && !(pol->t == J_ARR && pol->n == 2 && pol->items[0].t == J_NUM && pol->items[0].num == 133 &&
                 pol->items[1].t == J_NUM && pol->items[1].num == 171)) {
        seterr(err, el, "Parameters_Control_Field Polynomials: the hardware supports [133, 171] only");
        return -1;
    }
    if (get_num(d, "C_init_ID1", 129, &x) || !is_int(x, 0, 2147483647.0)) { seterr(err, el, "control field C_init_ID1: invalid"); return -1; }
    c->cf_id1 = (uint32_t)x;
    if (get_num(d, "C_init_Dat", 130, &x) || !is_int(x, 0, 2147483647.0)) { seterr(err, el, "control field C_init_Dat: invalid"); return -1; }
    c->cf_cdat = (uint32_t)x;
    return 0;
}

static const jval *defined(const jval *o)
{
    const jval *d = jget(o, "Defined");
    return d ? d : o;
}

int ptx_cfg_from_json(const char *text, ptx_cfg *c, char *err, size_t el)
{
    jval root;
    const char *p = text;
    const jval *tx, *cf, *g;
    double x;
    int rc = -1;
    memset(&root, 0, sizeof root);
    memset(c, 0, sizeof *c);
    c->cf_id1 = 129;
    c->cf_cdat = 130;
    c->gap = 4096;
    if (parse_value(&p, &root, 0) || root.t != J_OBJ) { seterr(err, el, "not a JSON object"); goto out; }
    tx = jget(&root, "Tx");
    if (!tx) { seterr(err, el, "no \"Tx\" (the Defined section of Parameters_Tx_payload.json)"); goto out; }
    if (parse_tx(defined(tx), c, err, el)) goto out;
    cf = jget(&root, "ControlField");
    if (cf && parse_cf(defined(cf), c, err, el)) goto out;
    if (get_num(&root, "Gap", 4096, &x) || !is_int(x, 0, 16.0 * 65535)) { seterr(err, el, "Gap: 0 .. 1048560 samples"); goto out; }
    c->gap = (int)x;
    g = jget(&root, "Gains");
    if (g) {
        if (get_num(g, "pre", 0, &x) || !is_int(x, 0, 65535)) { seterr(err, el, "Gains pre: 0 .. 65535"); goto out; }
        c->gain_pre = (int)x;
        if (get_num(g, "cf", 0, &x) || !is_int(x, 0, 65535)) { seterr(err, el, "Gains cf: 0 .. 65535"); goto out; }
        c->gain_cf = (int)x;
        if (get_num(g, "pl", 0, &x) || !is_int(x, 0, 65535)) { seterr(err, el, "Gains pl: 0 .. 65535"); goto out; }
        c->gain_pl = (int)x;
    }
    rc = 0;
out:
    jfree(&root);
    return rc;
}

/* ------------------------------------------------------------------------------------------ */

static int code_of(int v, const int *tab, int n)
{
    int i;
    for (i = 0; i < n; i++) if (tab[i] == v) return i;
    return -1;
}

int ptx_derive(const ptx_cfg *c, ptx_derived *d, char *err, size_t el)
{
    static const int rsyms[] = {1, 3, 6, 12}, rsigs[] = {3, 6, 12, 24}, plens[] = {37, 49, 73, 97, 121},
                     ns[] = {648, 1296, 1944}, reps[] = {1, 3, 5, 7, 11, 15, 19, 23, 31};
    static const int dref[] = {274, 544, 670, 726}, dcref[] = {7, 17, 31, 45}, nbs[] = {1, 2, 4, 6},
                     ms[] = {12, 8, 6, 4}, zs[] = {27, 54, 81};
    static const int plcode[] = {3, 7, 4, 6, 5};          /* PacketLength field: 011 111 100 110 101 */
    int rsc = code_of(c->ref_sym, rsyms, 4), sgc = code_of(c->ref_sig, rsigs, 4), plc = code_of(c->plen, plens, 5),
        nc = code_of(c->n, ns, 3), rmc = code_of(c->rep, reps, 9);
    int fields[13][2], i, j, pos;
    memset(d, 0, sizeof *d);
    if (rsc < 0) { seterr(err, el, "RefSymbolSpacing %d: 1, 3, 6 or 12", c->ref_sym); return -1; }
    if (sgc < 0) { seterr(err, el, "RefSignalSpacing %d: 3, 6, 12 or 24", c->ref_sig); return -1; }
    if (plc < 0) { seterr(err, el, "PacketLength %d: 37, 49, 73, 97 or 121", c->plen); return -1; }
    if (nc < 0) { seterr(err, el, "EncoderBlockSize %d: 648, 1296 or 1944", c->n); return -1; }
    if (rmc < 0) { seterr(err, el, "Repetition %d: 1, 3, 5, 7, 11, 15, 19, 23 or 31", c->rep); return -1; }
    d->nb = nbs[c->mod];
    d->nref = (c->plen - 1) / c->ref_sym + 1;
    d->data_res = d->nref * dref[sgc] + (c->plen - d->nref) * 826;
    d->data_bits = d->data_res * d->nb;
    d->k = (24 - ms[c->rate]) * zs[nc];
    d->tb_size = d->k - 24;
    d->cw_size = c->n * (c->rep + 1);
    d->ndb = d->data_bits / d->cw_size;
    d->zero_pad = d->data_bits - d->ndb * d->cw_size;
    d->words_per_block = (d->tb_size + 63) / 64;
    d->words_per_packet = d->ndb * d->words_per_block;
    d->ref_p0 = d->ref_p1 = d->nref * (840 / c->ref_sig);
    d->dc = d->nref * dcref[sgc] + (c->plen - d->nref) * 15;
    if (d->ndb < 1) { seterr(err, el, "no code word fits in the packet (CodeWordSize %d > NumberOfDataSigs %d)",
                             d->cw_size, d->data_bits); return -1; }
    if (d->ndb > 1023) { seterr(err, el, "AvailableDataBlocks %d does not fit NDB_A (10 bits)", d->ndb); return -1; }
    /* ControlMessage (Initialize_Control_Field.CONTROL_FIELD_CODING), MSB first */
    {
        int v[13] = {rsc, sgc, plcode[plc], c->mod, c->antennas - 1, nc, c->rate, rmc, d->ndb, c->client,
                     c->clk_cnt, c->c_init, 0};
        int w[13] = {2, 2, 3, 2, 1, 2, 2, 4, 10, 1, 14, 8, 3};
        for (i = 0; i < 13; i++) { fields[i][0] = v[i]; fields[i][1] = w[i]; }
    }
    for (pos = 0, i = 0; i < 13; i++)
        for (j = fields[i][1] - 1; j >= 0; j--) d->message[pos++] = (fields[i][0] >> j) & 1;
    return 0;
}

void ptx_header(const ptx_cfg *c, uint32_t n_packets, uint64_t w[6])
{
    int nc = (c->n == 648) ? 0 : (c->n == 1296) ? 1 : 2;
    w[0] = 0x50584C46ull | (2ull << 32);
    w[1] = (uint64_t)c->mod | (uint64_t)c->ref_sym << 2 | (uint64_t)c->ref_sig << 6 | (uint64_t)c->plen << 11 |
           (uint64_t)c->port << 18 | (uint64_t)(c->antennas - 1) << 19 | (uint64_t)c->client << 20 |
           (uint64_t)c->use_pre << 21 | (uint64_t)c->use_cf << 22 | (uint64_t)c->rate << 23 |
           (uint64_t)nc << 25 | (uint64_t)c->rep << 27 | (uint64_t)c->c_init << 32 | (uint64_t)c->clk_cnt << 40;
    w[2] = (uint64_t)c->id1 | (uint64_t)c->id2 << 31;
    w[3] = (uint64_t)c->gain_pre | (uint64_t)c->gain_cf << 16 | (uint64_t)c->gain_pl << 32 |
           (uint64_t)((c->gap + 15) / 16) << 48;
    w[4] = (uint64_t)n_packets | (uint64_t)40000 << 32;
    w[5] = (uint64_t)c->cf_id1 | (uint64_t)c->cf_cdat << 31;
}

int ptx_reply_json(const ptx_cfg *c, const ptx_derived *d, char *out, size_t len)
{
    char msg[55];
    int i, n;
    uint64_t w[6];
    for (i = 0; i < 54; i++) msg[i] = (char)('0' + d->message[i]);
    msg[54] = 0;
    ptx_header(c, 1, w);
    n = snprintf(out, len,
        "{\"ok\": true, \"BlocksPerPacket\": %d, \"TransportBlockSize\": %d, \"WordsPerBlock\": %d, "
        "\"WordsPerPacket\": %d, \"Gap\": %d, \"PacketSamples\": %d, \"ControlMessage\": \"%s\", "
        "\"Tx\": {\"Calculated\": {\"CodeBlockSize\": %d, \"DataBlockSize\": %d, \"CodeWordSize\": %d, "
        "\"AvailableDataBlocks\": %d, \"ZeroPadding\": %d, \"CRClen\": 24, \"TransportBlockSize\": %d, "
        "\"NumberOfRefSigs_P0\": %d, \"NumberOfRefSigs_P1\": %d, \"NumberOfDCSigs\": %d, "
        "\"NumberOfDataSigs\": %d, \"ScrambledSequenceSize\": %d}}, "
        "\"ControlField\": {\"Calculated\": {\"CRClen\": 10, \"DataBlockSize\": 64, \"TailBits\": 6, "
        "\"EncoderInputSize\": 70, \"CodeBlockSize\": 140, \"Repetition\": 14, \"CodeWordSize\": 2100, "
        "\"ZeroPadding\": 100, \"NumberOfRefSigs_P0\": 560, \"NumberOfRefSigs_P1\": 560, \"NumberOfDCSigs\": 44, "
        "\"NumberOfDataSigs\": 2200, \"ScrambledSequenceSize\": 2200}}, "
        "\"Header\": [\"%016llx\", \"%016llx\", \"%016llx\", \"%016llx\", \"%016llx\", \"%016llx\"]}",
        d->ndb, d->tb_size, d->words_per_block, d->words_per_packet, (c->gap + 15) / 16 * 16,
        (c->use_pre ? 12480 : 0) + (c->use_cf ? 9120 : 0) + c->plen * 2280, msg,
        c->n, d->k, d->cw_size, d->ndb, d->zero_pad, d->tb_size, d->ref_p0, d->ref_p1, d->dc,
        d->data_bits, d->data_bits,
        (unsigned long long)w[0], (unsigned long long)w[1], (unsigned long long)w[2],
        (unsigned long long)w[3], (unsigned long long)w[4], (unsigned long long)w[5]);
    return (n > 0 && (size_t)n < len) ? 0 : -1;
}
