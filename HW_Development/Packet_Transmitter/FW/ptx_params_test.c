/* ptx_params_test.c - ptx_params on the PC: reads a SET_PARAMS JSON (file or stdin) and prints the
 * reply the server would send, or "ERROR: ..." (FW/test_fw.py compares it with the model). */
#include <stdio.h>
#include <stdlib.h>
#include "ptx_params.h"

int main(int argc, char **argv)
{
    static char text[1 << 16], out[1 << 14];
    char err[256];
    FILE *f = argc > 1 ? fopen(argv[1], "r") : stdin;
    size_t n;
    ptx_cfg c;
    ptx_derived d;
    if (!f) { perror(argv[1]); return 2; }
    n = fread(text, 1, sizeof text - 1, f);
    text[n] = 0;
    if (ptx_cfg_from_json(text, &c, err, sizeof err) || ptx_derive(&c, &d, err, sizeof err)) {
        printf("ERROR: %s\n", err);
        return 1;
    }
    if (ptx_reply_json(&c, &d, out, sizeof out)) { printf("ERROR: reply too long\n"); return 1; }
    printf("%s\n", out);
    return 0;
}
