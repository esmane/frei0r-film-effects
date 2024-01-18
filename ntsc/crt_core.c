/*****************************************************************************/
/*
 * NTSC/CRT - integer-only NTSC video signal encoding / decoding emulation
 *
 *   by EMMIR 2018-2023
 *
 *   YouTube: https://www.youtube.com/@EMMIR_KC/videos
 *   Discord: https://discord.com/invite/hdYctSmyQJ
 */
/*****************************************************************************/
#include "crt_core.h"

#include <stdlib.h>
#include <string.h>

/* ensure negative values for x get properly modulo'd */
#define POSMOD(x, n)     (((x) % (n) + (n)) % (n))

static int sigpsin15[18] = { /* significant points on sine wave (15-bit) */
    0x0000,
    0x0c88,0x18f8,0x2528,0x30f8,0x3c50,0x4718,0x5130,0x5a80,
    0x62f0,0x6a68,0x70e0,0x7640,0x7a78,0x7d88,0x7f60,0x8000,
    0x7f60
};

static int
sintabil8(int n)
{
    int f, i, a, b;

    /* looks scary but if you don't change T14_2PI
     * it won't cause out of bounds memory reads
     */
    f = n >> 0 & 0xff;
    i = n >> 8 & 0xff;
    a = sigpsin15[i];
    b = sigpsin15[i + 1];
    return (a + ((b - a) * f >> 8));
}

/* 14-bit interpolated sine/cosine */
extern void
crt_sincos14(int *s, int *c, int n)
{
    int h;

    n &= T14_MASK;
    h = n & ((T14_2PI >> 1) - 1);

    if (h > ((T14_2PI >> 2) - 1)) {
        *c = -sintabil8(h - (T14_2PI >> 2));
        *s = sintabil8((T14_2PI >> 1) - h);
    } else {
        *c = sintabil8((T14_2PI >> 2) - h);
        *s = sintabil8(h);
    }
    if (n > ((T14_2PI >> 1) - 1)) {
        *c = -*c;
        *s = -*s;
    }
}

/*****************************************************************************/
/********************************* FILTERS ***********************************/
/*****************************************************************************/

/* convolution is much faster but the EQ looks softer, more authentic, and more analog */
#define USE_CONVOLUTION 0
#define USE_7_SAMPLE_KERNEL 1
#define USE_6_SAMPLE_KERNEL 0
#define USE_5_SAMPLE_KERNEL 0

#if (CRT_CC_SAMPLES != 4)
/* the current convolutions do not filter properly at > 4 samples */
#undef USE_CONVOLUTION
#define USE_CONVOLUTION 0
#endif

#if USE_CONVOLUTION

/* NOT 3 band equalizer, faster convolution instead.
 * eq function names preserved to keep code clean
 */
static struct EQF {
    int h[7];
} eqY, eqI, eqQ;

/* params unused to keep the function the same */
static void
init_eq(struct EQF *f,
        int f_lo, int f_hi, int rate,
        int g_lo, int g_mid, int g_hi)
{
    memset(f, 0, sizeof(struct EQF));
}

static void
reset_eq(struct EQF *f)
{
    memset(f->h, 0, sizeof(f->h));
}

static int
eqf(struct EQF *f, int s)
{
    int i;
    int *h = f->h;

    for (i = 6; i > 0; i--) {
        h[i] = h[i - 1];
    }
    h[0] = s;
#if USE_7_SAMPLE_KERNEL
    /* index : 0 1 2 3 4 5 6 */
    /* weight: 1 4 7 8 7 4 1 */
    return (s + h[6] + ((h[1] + h[5]) * 4) + ((h[2] + h[4]) * 7) + (h[3] * 8)) >> 5;
#elif USE_6_SAMPLE_KERNEL
    /* index : 0 1 2 3 4 5 */
    /* weight: 1 3 4 4 3 1 */
    return (s + h[5] + 3 * (h[1] + h[4]) + 4 * (h[2] + h[3])) >> 4;
#elif USE_5_SAMPLE_KERNEL
    /* index : 0 1 2 3 4 */
    /* weight: 1 2 2 2 1 */
    return (s + h[4] + ((h[1] + h[2] + h[3]) << 1)) >> 3;
#else
    /* index : 0 1 2 3 */
    /* weight: 1 1 1 1*/
    return (s + h[3] + h[1] + h[2]) >> 2;
#endif
}

#else

#define HISTLEN     3
#define HISTOLD     (HISTLEN - 1) /* oldest entry */
#define HISTNEW     0             /* newest entry */

#define EQ_P        16 /* if changed, the gains will need to be adjusted */
#define EQ_R        (1 << (EQ_P - 1)) /* rounding */
/* three band equalizer */
static struct EQF {
    int lf, hf; /* fractions */
    int g[3]; /* gains */
    int fL[4];
    int fH[4];
    int h[HISTLEN]; /* history */
} eqY, eqI, eqQ;

/* f_lo - low cutoff frequency
 * f_hi - high cutoff frequency
 * rate - sampling rate
 * g_lo, g_mid, g_hi - gains
 */
static void
init_eq(struct EQF *f,
        int f_lo, int f_hi, int rate,
        int g_lo, int g_mid, int g_hi)
{
    int sn, cs;

    memset(f, 0, sizeof(struct EQF));

    f->g[0] = g_lo;
    f->g[1] = g_mid;
    f->g[2] = g_hi;

    crt_sincos14(&sn, &cs, T14_PI * f_lo / rate);
#if (EQ_P >= 15)
    f->lf = 2 * (sn << (EQ_P - 15));
#else
    f->lf = 2 * (sn >> (15 - EQ_P));
#endif
    crt_sincos14(&sn, &cs, T14_PI * f_hi / rate);
#if (EQ_P >= 15)
    f->hf = 2 * (sn << (EQ_P - 15));
#else
    f->hf = 2 * (sn >> (15 - EQ_P));
#endif
}

static void
reset_eq(struct EQF *f)
{
    memset(f->fL, 0, sizeof(f->fL));
    memset(f->fH, 0, sizeof(f->fH));
    memset(f->h, 0, sizeof(f->h));
}

static int
eqf(struct EQF *f, int s)
{
    int i, r[3];

    f->fL[0] += (f->lf * (s - f->fL[0]) + EQ_R) >> EQ_P;
    f->fH[0] += (f->hf * (s - f->fH[0]) + EQ_R) >> EQ_P;

    for (i = 1; i < 4; i++) {
        f->fL[i] += (f->lf * (f->fL[i - 1] - f->fL[i]) + EQ_R) >> EQ_P;
        f->fH[i] += (f->hf * (f->fH[i - 1] - f->fH[i]) + EQ_R) >> EQ_P;
    }

    r[0] = f->fL[3];
    r[1] = f->fH[3] - f->fL[3];
    r[2] = f->h[HISTOLD] - f->fH[3];

    for (i = 0; i < 3; i++) {
        r[i] = (r[i] * f->g[i]) >> EQ_P;
    }

    for (i = HISTOLD; i > 0; i--) {
        f->h[i] = f->h[i - 1];
    }
    f->h[HISTNEW] = s;

    return (r[0] + r[1] + r[2]);
}

#endif

/*****************************************************************************/
/***************************** PUBLIC FUNCTIONS ******************************/
/*****************************************************************************/

extern void
crt_init(struct CRT *v, int w, int h)
{
    memset(v, 0, sizeof(struct CRT));

    // resize
    v->w = w;
    v->h = h;
    v->out = NULL;

    // reset
    v->hue = 0;
    v->saturation = 10;
    v->brightness = 0;
    v->contrast = 180;
    v->black_point = 0;
    v->white_point = 100;
    v->hsync = 0;
    v->vsync = 0;

    v->v_fac = 1; /* factor to stretch img vertically onto the output img */

    v->data = NULL; /* image data */
    v->field = 0;      /* 0 = even, 1 = odd */
    v->frame = 0;      /* 0 = even, 1 = odd */
    v->hue = 0;        /* 0-359 */
    v->xoffset = 0;    /* x offset in sample space. 0 is minimum value */
    v->yoffset = 0;    /* y offset in # of lines. 0 is minimum value */
    v->do_vhs = 0;     /* VHS only: 0 = no aberration, 1 = with aberration */


    v->rn = 194;

    /* kilohertz to line sample conversion */
#define kHz2L(kHz) (CRT_HRES * (kHz * 100) / L_FREQ)

    /* band gains are pre-scaled as 16-bit fixed point
     * if you change the EQ_P define, you'll need to update these gains too
     */
#if (CRT_CC_SAMPLES == 4)
    init_eq(&eqY, kHz2L(1500), kHz2L(3000), CRT_HRES, 65536, 8192, 9175);
    init_eq(&eqI, kHz2L(80),   kHz2L(1150), CRT_HRES, 65536, 65536, 1311);
    init_eq(&eqQ, kHz2L(80),   kHz2L(1000), CRT_HRES, 65536, 65536, 0);
#elif (CRT_CC_SAMPLES == 5)
    init_eq(&eqY, kHz2L(1500), kHz2L(3000), CRT_HRES, 65536, 12192, 7775);
    init_eq(&eqI, kHz2L(80),   kHz2L(1150), CRT_HRES, 65536, 65536, 1311);
    init_eq(&eqQ, kHz2L(80),   kHz2L(1000), CRT_HRES, 65536, 65536, 0);
#else
#error "NTSC-CRT currently only supports 4 or 5 samples per chroma period."
#endif

}

extern void
crt_demodulate(struct CRT *v, int noise)
{
    /* made static so all this data does not go on the stack */
    static struct {
        int y, i, q;
    } out[AV_LEN + 1], *yiqA, *yiqB;
    int i, j, line, rn;
    signed char *sig;
    int s = 0;
    int field, ratio;
    int *ccr; /* color carrier signal */
    int huesn, huecs;
    int xnudge = -3, ynudge = 3;
    int bright = v->brightness - (BLACK_LEVEL + v->black_point);
    int pitch;
    
    pitch = v->w * CRT_BPP;

    crt_sincos14(&huesn, &huecs, ((v->hue % 360) + 33) * 8192 / 180);
    huesn >>= 11; /* make 4-bit */
    huecs >>= 11;

    rn = v->rn;
#if !CRT_DO_VSYNC
    /* determine field before we add noise,
     * otherwise it's not reliably recoverable
     */
    for (i = -CRT_VSYNC_WINDOW; i < CRT_VSYNC_WINDOW; i++) {
        line = POSMOD(v->vsync + i, CRT_VRES);
        sig = v->analog + line * CRT_HRES;
        s = 0;
        for (j = 0; j < CRT_HRES; j++) {
            s += sig[j];
            if (s <= (CRT_VSYNC_THRESH * SYNC_LEVEL)) {
                goto found_field;
            }
        }
    }
found_field:
    /* if vsync signal was in second half of line, odd field */
    field = (j > (CRT_HRES / 2));
    v->vsync = -3;
#endif
    if(v->do_vhs) {
        line = ((rand() % 8) - 4) + 14;
    }
    for (i = 0; i < CRT_INPUT_SIZE; i++) {
        int nn = noise;
        if(v->do_vhs)
        {
            rn = rand();
            if (i > (CRT_INPUT_SIZE - CRT_HRES * (16 + ((rand() % 20) - 10))) &&
                i < (CRT_INPUT_SIZE - CRT_HRES * (5 + ((rand() % 8) - 4)))) {
                int ln, sn, cs;

                ln = (i * line) / CRT_HRES;
                crt_sincos14(&sn, &cs, ln * 8192 / 180);
                nn = cs >> 8;
            }
        }
        else
        {
            rn = (214019 * rn + 140327895);
        }
        /* signal + noise */
        s = v->analog[i] + (((((rn >> 16) & 0xff) - 0x7f) * nn) >> 8);
        if (s >  127) { s =  127; }
        if (s < -127) { s = -127; }
        v->inp[i] = s;
    }
    v->rn = rn;

#if CRT_DO_VSYNC
    /* Look for vertical sync.
     *
     * This is done by integrating the signal and
     * seeing if it exceeds a threshold. The threshold of
     * the vertical sync pulse is much higher because the
     * vsync pulse is a lot longer than the hsync pulse.
     * The signal needs to be integrated to lessen
     * the noise in the signal.
     */
    for (i = -CRT_VSYNC_WINDOW; i < CRT_VSYNC_WINDOW; i++) {
        line = POSMOD(v->vsync + i, CRT_VRES);
        sig = v->inp + line * CRT_HRES;
        s = 0;
        for (j = 0; j < CRT_HRES; j++) {
            s += sig[j];
            /* increase the multiplier to make the vsync
             * more stable when there is a lot of noise
             */
            if (s <= (CRT_VSYNC_THRESH * SYNC_LEVEL)) {
                goto vsync_found;
            }
        }
    }
vsync_found:
    v->vsync = line; /* vsync found (or gave up) at this line */
    /* if vsync signal was in second half of line, odd field */
    field = (j > (CRT_HRES / 2));
#endif

    /* ratio of output height to active video lines in the signal */
    ratio = (v->h << 16) / CRT_LINES;
    ratio = (ratio + 32768) >> 16;

    field = (field * (ratio / 2));

    for (line = CRT_TOP; line < CRT_BOT; line++) {
        unsigned pos, ln, scanR;
        int scanL, dx;
        int L, R;
        unsigned char *cL, *cR;
#if (CRT_CC_SAMPLES == 4)
        int wave[CRT_CC_SAMPLES];
#else
        int waveI[CRT_CC_SAMPLES];
        int waveQ[CRT_CC_SAMPLES];
#endif
        int dci, dcq; /* decoded I, Q */
        int xpos, ypos;
        int beg, end;
        int phasealign;
        int line_w;

        beg = (line - CRT_TOP + 0) * (v->h + v->v_fac) / CRT_LINES + field;
        end = (line - CRT_TOP + 1) * (v->h + v->v_fac) / CRT_LINES + field;

        if (beg >= v->h) { continue; }
        if (end > v->h) { end = v->h; }

        /* Look for horizontal sync.
         * See comment above regarding vertical sync.
         */
        ln = (POSMOD(line + v->vsync, CRT_VRES)) * CRT_HRES;
        sig = v->inp + ln + v->hsync;
        s = 0;
        for (i = -CRT_HSYNC_WINDOW; i < CRT_HSYNC_WINDOW; i++) {
            s += sig[SYNC_BEG + i];
            if (s <= (CRT_HSYNC_THRESH * SYNC_LEVEL)) {
                break;
            }
        }
#if CRT_DO_HSYNC
        v->hsync = POSMOD(i + v->hsync, CRT_HRES);
#else
        v->hsync = 0;
#endif

        xpos = POSMOD(AV_BEG + v->hsync + xnudge, CRT_HRES);
        ypos = POSMOD(line + v->vsync + ynudge, CRT_VRES);
        pos = xpos + ypos * CRT_HRES;

        ccr = v->ccf[ypos % CRT_CC_VPER];
#if (CRT_CC_SAMPLES == 4)
        sig = v->inp + ln + (v->hsync & ~3); /* faster */
#else
        sig = v->inp + ln + (v->hsync - (v->hsync % CRT_CC_SAMPLES));
#endif
        for (i = CB_BEG; i < CB_BEG + (CB_CYCLES * CRT_CB_FREQ); i++) {
            int p, n;
            p = ccr[i % CRT_CC_SAMPLES] * 127 / 128; /* fraction of the previous */
            n = sig[i];                 /* mixed with the new sample */
            ccr[i % CRT_CC_SAMPLES] = p + n;
        }

        phasealign = POSMOD(v->hsync, CRT_CC_SAMPLES);

#if (CRT_CC_SAMPLES == 4)
        /* amplitude of carrier = saturation, phase difference = hue */
        dci = ccr[(phasealign + 1) & 3] - ccr[(phasealign + 3) & 3];
        dcq = ccr[(phasealign + 2) & 3] - ccr[(phasealign + 0) & 3];

        wave[0] = ((dci * huecs - dcq * huesn) >> 4) * v->saturation;
        wave[1] = ((dcq * huecs + dci * huesn) >> 4) * v->saturation;
        wave[2] = -wave[0];
        wave[3] = -wave[1];
#elif (CRT_CC_SAMPLES == 5)
        {
            int dciA, dciB;
            int dcqA, dcqB;
            int ang = (v->hue % 360);
            int off180 = CRT_CC_SAMPLES / 2;
            int off90 = CRT_CC_SAMPLES / 4;
            int peakA = phasealign + off90;
            int peakB = phasealign + 0;
            dciA = dciB = dcqA = dcqB = 0;
            /* amplitude of carrier = saturation, phase difference = hue */
            dciA = ccr[(peakA) % CRT_CC_SAMPLES];
            /* average */
            dciB = (ccr[(peakA + off180) % CRT_CC_SAMPLES]
                  + ccr[(peakA + off180 + 1) % CRT_CC_SAMPLES]) / 2;
            dcqA = ccr[(peakB + off180) % CRT_CC_SAMPLES];
            dcqB = ccr[(peakB) % CRT_CC_SAMPLES];
            dci = dciA - dciB;
            dcq = dcqA - dcqB;
            /* create wave tables and rotate them by the hue adjustment angle */
            for (i = 0; i < CRT_CC_SAMPLES; i++) {
                int sn, cs;
                crt_sincos14(&sn, &cs, ang * 8192 / 180);
                waveI[i] = ((dci * cs + dcq * sn) >> 15) * v->saturation;
                /* Q is offset by 90 */
                crt_sincos14(&sn, &cs, (ang + 90) * 8192 / 180);
                waveQ[i] = ((dci * cs + dcq * sn) >> 15) * v->saturation;
                ang += (360 / CRT_CC_SAMPLES);
            }
        }
#endif
        sig = v->inp + pos;

            dx = ((AV_LEN - 1) << 12) / v->w;
            scanL = 0;
            scanR = (AV_LEN - 1) << 12;
            L = 0;
            R = AV_LEN;

        reset_eq(&eqY);
        reset_eq(&eqI);
        reset_eq(&eqQ);

#if (CRT_CC_SAMPLES == 4)
        for (i = L; i < R; i++) {
            out[i].y = eqf(&eqY, sig[i] + bright) << 4;
            out[i].i = eqf(&eqI, sig[i] * wave[(i + 0) & 3] >> 9) >> 3;
            out[i].q = eqf(&eqQ, sig[i] * wave[(i + 3) & 3] >> 9) >> 3;
        }
#else
        for (i = L; i < R; i++) {
            out[i].y = eqf(&eqY, sig[i] + bright) << 4;
            out[i].i = eqf(&eqI, sig[i] * waveI[i % CRT_CC_SAMPLES] >> 9) >> 3;
            out[i].q = eqf(&eqQ, sig[i] * waveQ[i % CRT_CC_SAMPLES] >> 9) >> 3;
        }
#endif

        cL = v->out + (beg * pitch);
        cR = cL + pitch;

        for (pos = scanL; pos < scanR && cL < cR; pos += dx) {
            int y, i, q;
            int r, g, b;
            int aa, bb;

            R = pos & 0xfff;
            L = 0xfff - R;
            s = pos >> 12;

            yiqA = out + s;
            yiqB = out + s + 1;

            /* interpolate between samples if needed */
            y = ((yiqA->y * L) >>  2) + ((yiqB->y * R) >>  2);
            i = ((yiqA->i * L) >> 14) + ((yiqB->i * R) >> 14);
            q = ((yiqA->q * L) >> 14) + ((yiqB->q * R) >> 14);

            /* YIQ to RGB */
            r = (((y + 3879 * i + 2556 * q) >> 12) * v->contrast) >> 8;
            g = (((y - 1126 * i - 2605 * q) >> 12) * v->contrast) >> 8;
            b = (((y - 4530 * i + 7021 * q) >> 12) * v->contrast) >> 8;

            if (r < 0) r = 0;
            if (g < 0) g = 0;
            if (b < 0) b = 0;
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;

            bb = (r << 16 | g << 8 | b);

            cL[0] = bb >> 16 & 0xff;
            cL[1] = bb >>  8 & 0xff;
            cL[2] = bb >>  0 & 0xff;
            cL += CRT_BPP;
        }

        /* duplicate extra lines */
        for (s = beg + 1; s < end; s++) {
            memcpy(v->out + s * pitch, v->out + (s - 1) * pitch, pitch);
        }
    }
}

/*****************************************************************************/
/*
 * NTSC/CRT - integer-only NTSC video signal encoding / decoding emulation
 *
 *   by EMMIR 2018-2023
 *
 *   YouTube: https://www.youtube.com/@EMMIR_KC/videos
 *   Discord: https://discord.com/invite/hdYctSmyQJ
 */
/*****************************************************************************/

#if (CRT_CHROMA_PATTERN == 1)
/* 227.5 subcarrier cycles per line means every other line has reversed phase */
#define CC_PHASE(ln)     (((ln) & 1) ? -1 : 1)
#else
#define CC_PHASE(ln)     (1)
#endif

#define EXP_P         11
#define EXP_ONE       (1 << EXP_P)
#define EXP_MASK      (EXP_ONE - 1)
#define EXP_PI        6434
#define EXP_MUL(x, y) (((x) * (y)) >> EXP_P)
#define EXP_DIV(x, y) (((x) << EXP_P) / (y))

static int e11[] = {
    EXP_ONE,
    5567,  /* e   */
    15133, /* e^2 */
    41135, /* e^3 */
    111817 /* e^4 */
};

/* fixed point e^x */
static int
expx(int n)
{
    int neg, idx, res;
    int nxt, acc, del;
    int i;

    if (n == 0) {
        return EXP_ONE;
    }
    neg = n < 0;
    if (neg) {
        n = -n;
    }
    idx = n >> EXP_P;
    res = EXP_ONE;
    for (i = 0; i < idx / 4; i++) {
        res = EXP_MUL(res, e11[4]);
    }
    idx &= 3;
    if (idx > 0) {
        res = EXP_MUL(res, e11[idx]);
    }

    n &= EXP_MASK;
    nxt = EXP_ONE;
    acc = 0;
    del = 1;
    for (i = 1; i < 17; i++) {
        acc += nxt / del;
        nxt = EXP_MUL(nxt, n);
        del *= i;
        if (del > nxt || nxt <= 0 || del <= 0) {
            break;
        }
    }
    res = EXP_MUL(res, acc);

    if (neg) {
        res = EXP_DIV(EXP_ONE, res);
    }
    return res;
}

/*****************************************************************************/
/********************************* FILTERS ***********************************/
/*****************************************************************************/

/* infinite impulse response low pass filter for bandlimiting YIQ */
static struct IIRLP {
    int c;
    int h; /* history */
} iirY, iirI, iirQ;

/* freq  - total bandwidth
 * limit - max frequency
 */
static void
init_iir(struct IIRLP *f, int freq, int limit)
{
    int rate; /* cycles/pixel rate */

    memset(f, 0, sizeof(struct IIRLP));
    rate = (freq << 9) / limit;
    f->c = EXP_ONE - expx(-((EXP_PI << 9) / rate));
}

static void
reset_iir(struct IIRLP *f)
{
    f->h = 0;
}

/* hi-pass for debugging */
#define HIPASS 0

static int
iirf(struct IIRLP *f, int s)
{
    f->h += EXP_MUL(s - f->h, f->c);
#if HIPASS
    return s - f->h;
#else
    return f->h;
#endif
}

extern void
crt_modulate(struct CRT *v)
{
    int x, y, xo, yo;
    int destw = AV_LEN;
    int desth = ((CRT_LINES * 64500) >> 16);
    int iccf[CRT_CC_SAMPLES];
    int ccmodI[CRT_CC_SAMPLES]; /* color phase for mod */
    int ccmodQ[CRT_CC_SAMPLES]; /* color phase for mod */
    int ccburst[CRT_CC_SAMPLES]; /* color phase for burst */
    int sn, cs, n, ph;
    int inv_phase = 0;
    int aberration = 0;

    if (!v->iirs_initialized) {
            if(v->do_vhs)
            {
                init_iir(&iirY, L_FREQ, Y_FREQ_VHS);
                init_iir(&iirI, L_FREQ, I_FREQ_VHS);
                init_iir(&iirQ, L_FREQ, Q_FREQ_VHS);
            }
            else
            {
                init_iir(&iirY, L_FREQ, Y_FREQ);
                init_iir(&iirI, L_FREQ, I_FREQ);
                init_iir(&iirQ, L_FREQ, Q_FREQ);
            }
        v->iirs_initialized = 1;
    }

    for (x = 0; x < CRT_CC_SAMPLES; x++) {
            n = v->hue + x * (360 / CRT_CC_SAMPLES);
            crt_sincos14(&sn, &cs, (n + 33) * 8192 / 180);
            ccburst[x] = sn >> 10;
            crt_sincos14(&sn, &cs, n * 8192 / 180);
            ccmodI[x] = sn >> 10;
            crt_sincos14(&sn, &cs, (n - 90) * 8192 / 180);
            ccmodQ[x] = sn >> 10;
    }


    xo = AV_BEG  + v->xoffset + (AV_LEN    - destw) / 2;
    yo = CRT_TOP + v->yoffset + (CRT_LINES - desth) / 2;

    v->field &= 1;
    v->frame &= 1;
    inv_phase = (v->field == v->frame);
    ph = CC_PHASE(inv_phase);

    /* align signal */
    xo = (xo & ~3);

    // vhs abberation
    if (v->do_vhs) {
        aberration = ((rand() % 12) - 8) + 14;
    }

    for (n = 0; n < CRT_VRES; n++) {
        int t; /* time */
        signed char *line = &v->analog[n * CRT_HRES];

        t = LINE_BEG;

        if (n <= 3 || (n >= 7 && n <= 9)) {
            /* equalizing pulses - small blips of sync, mostly blank */
            while (t < (4   * CRT_HRES / 100)) line[t++] = SYNC_LEVEL;
            while (t < (50  * CRT_HRES / 100)) line[t++] = BLANK_LEVEL;
            while (t < (54  * CRT_HRES / 100)) line[t++] = SYNC_LEVEL;
            while (t < (100 * CRT_HRES / 100)) line[t++] = BLANK_LEVEL;
        } else if (n >= 4 && n <= 6) {
            int even[4] = { 46, 50, 96, 100 };
            int odd[4] =  { 4, 50, 96, 100 };
            int *offs = even;
            if (v->field == 1) {
                offs = odd;
            }
            /* vertical sync pulse - small blips of blank, mostly sync */
            while (t < (offs[0] * CRT_HRES / 100)) line[t++] = SYNC_LEVEL;
            while (t < (offs[1] * CRT_HRES / 100)) line[t++] = BLANK_LEVEL;
            while (t < (offs[2] * CRT_HRES / 100)) line[t++] = SYNC_LEVEL;
            while (t < (offs[3] * CRT_HRES / 100)) line[t++] = BLANK_LEVEL;
        } else {
            int cb;
            if (n < (CRT_VRES - aberration)) {
                /* video line */
                while (t < SYNC_BEG) line[t++] = BLANK_LEVEL; /* FP */
                while (t < BW_BEG)   line[t++] = SYNC_LEVEL;  /* SYNC */
            }
            while (t < AV_BEG)   line[t++] = BLANK_LEVEL; /* BW + CB + BP */

            if (n < CRT_TOP) {
                while (t < CRT_HRES) line[t++] = BLANK_LEVEL;
            }

            /* CB_CYCLES of color burst at 3.579545 Mhz */
            for (t = CB_BEG; t < CB_BEG + (CB_CYCLES * CRT_CB_FREQ); t++) {
#if (CRT_CHROMA_PATTERN == 1)
                int off180 = CRT_CC_SAMPLES / 2;
                cb = ccburst[(t + inv_phase * off180) % CRT_CC_SAMPLES];
#else
                cb = ccburst[t % CRT_CC_SAMPLES];
#endif
                line[t] = (BLANK_LEVEL + (cb * BURST_LEVEL)) >> 5;
                iccf[t % CRT_CC_SAMPLES] = line[t];
            }
        }
    }
    /* reset hsync every frame so only the bottom part is warped */
    v->hsync = 0;

    for (y = 0; y < desth; y++) {
        int field_offset;
        int sy;

        field_offset = (v->field * v->h + desth) / desth / 2;
        sy = (y * v->h) / desth;

        sy += field_offset;

        if (sy >= v->h) sy = v->h;

        sy *= v->w;

        reset_iir(&iirY);
        reset_iir(&iirI);
        reset_iir(&iirQ);

        for (x = 0; x < destw; x++) {
            int fy, fi, fq;
            int rA, gA, bA;
            const unsigned char *pix;
            int ire; /* composite signal */
            int xoff;

            pix = v->data + ((((x * v->w) / destw) + sy) * CRT_BPP);
            rA = pix[0];
            gA = pix[1];
            bA = pix[2];

            /* RGB to YIQ */
            fy = (19595 * rA + 38470 * gA +  7471 * bA) >> 14;
            fi = (39059 * rA - 18022 * gA - 21103 * bA) >> 14;
            fq = (13894 * rA - 34275 * gA + 20382 * bA) >> 14;
            ire = BLACK_LEVEL + v->black_point;

            xoff = (x + xo) % CRT_CC_SAMPLES;
            /* bandlimit Y,I,Q */
            fy = iirf(&iirY, fy);
            fi = iirf(&iirI, fi) * ph * ccmodI[xoff] >> 4;
            fq = iirf(&iirQ, fq) * ph * ccmodQ[xoff] >> 4;
            ire += (fy + fi + fq) * (WHITE_LEVEL * v->white_point / 100) >> 10;
            if (ire < 0)   ire = 0;
            if (ire > 110) ire = 110;

            v->analog[(x + xo) + (y + yo) * CRT_HRES] = ire;
        }
    }

    if(v->do_vhs)
    {
        for (n = 0; n < CRT_CC_VPER; n++) {
            for (x = 0; x < CRT_CC_SAMPLES; x++) {
                v->ccf[n][x] = 0;
            }
        }
    }
    else
    {
        for (n = 0; n < CRT_CC_VPER; n++) {
            for (x = 0; x < CRT_CC_SAMPLES; x++) {
                v->ccf[n][x] = iccf[x] << 7;
            }
        }
    }
}
