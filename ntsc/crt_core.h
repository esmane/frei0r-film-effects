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

/* crt_ntsc.h
 *
 * An interface to convert a digital image to an analog NTSC signal.
 *
 */
/* 0 = vertical  chroma (228 chroma clocks per line) */
/* 1 = checkered chroma (227.5 chroma clocks per line) */
#ifndef _CRT_CORE_H_
#define _CRT_CORE_H_

#define CRT_CHROMA_PATTERN 1

/* chroma clocks (subcarrier cycles) per line */
#if (CRT_CHROMA_PATTERN == 1)
#define CRT_CC_LINE 2275
#else
/* this will give the 'rainbow' effect in the famous waterfall scene */
#define CRT_CC_LINE 2280
#endif

/* NOTE, in general, increasing CRT_CB_FREQ reduces blur and bleed */
#define CRT_CB_FREQ     4 /* carrier frequency relative to sample rate */
#define CRT_HRES        (CRT_CC_LINE * CRT_CB_FREQ / 10) /* horizontal res */
#define CRT_VRES        486                       /* vertical resolution */
#define CRT_INPUT_SIZE  (CRT_HRES * CRT_VRES)

#define CRT_TOP         2     /* first line with active video */
#define CRT_BOT         484    /* final line with active video */
#define CRT_LINES       (CRT_BOT - CRT_TOP) /* number of active video lines */

#define CRT_CC_SAMPLES  4 /* samples per chroma period (samples per 360 deg) */
#define CRT_CC_VPER     1 /* vertical period in which the artifacts repeat */

/* search windows, in samples */
#define CRT_HSYNC_WINDOW 8
#define CRT_VSYNC_WINDOW 8

/* accumulated signal threshold required for sync detection.
 * Larger = more stable, until it's so large that it is never reached in which
 *          case the CRT won't be able to sync
 */
#define CRT_HSYNC_THRESH 4
#define CRT_VSYNC_THRESH 94

/*
 *                      FULL HORIZONTAL LINE SIGNAL (~63500 ns)
 * |---------------------------------------------------------------------------|
 *   HBLANK (~10900 ns)                 ACTIVE VIDEO (~52600 ns)
 * |-------------------||------------------------------------------------------|
 *
 *
 *   WITHIN HBLANK PERIOD:
 *
 *   FP (~1500 ns)  SYNC (~4700 ns)  BW (~600 ns)  CB (~2500 ns)  BP (~1600 ns)
 * |--------------||---------------||------------||-------------||-------------|
 *      BLANK            SYNC           BLANK          BLANK          BLANK
 *
 */
#define LINE_BEG         0
#define FP_ns            1500      /* front porch */
#define SYNC_ns          4700      /* sync tip */
#define BW_ns            600       /* breezeway */
#define CB_ns            2500      /* color burst */
#define BP_ns            1600      /* back porch */
#define AV_ns            52600     /* active video */
#define HB_ns            (FP_ns + SYNC_ns + BW_ns + CB_ns + BP_ns) /* h blank */
/* line duration should be ~63500 ns */
#define LINE_ns          (FP_ns + SYNC_ns + BW_ns + CB_ns + BP_ns + AV_ns)

/* convert nanosecond offset to its corresponding point on the sampled line */
#define ns2pos(ns)       ((ns) * CRT_HRES / LINE_ns)
/* starting points for all the different pulses */
#define FP_BEG           ns2pos(0)
#define SYNC_BEG         ns2pos(FP_ns)
#define BW_BEG           ns2pos(FP_ns + SYNC_ns)
#define CB_BEG           ns2pos(FP_ns + SYNC_ns + BW_ns)
#define BP_BEG           ns2pos(FP_ns + SYNC_ns + BW_ns + CB_ns)
#define AV_BEG           ns2pos(HB_ns)
#define AV_LEN           ns2pos(AV_ns)

/* somewhere between 7 and 12 cycles */
#define CB_CYCLES   10

/* frequencies for bandlimiting */
#define L_FREQ           1431818 /* full line */
#define Y_FREQ           420000  /* Luma   (Y) 4.2  MHz of the 14.31818 MHz */
#define I_FREQ           150000  /* Chroma (I) 1.5  MHz of the 14.31818 MHz */
#define Q_FREQ           55000   /* Chroma (Q) 0.55 MHz of the 14.31818 MHz */

#define Y_FREQ_VHS       300000  /* Luma   (Y) 3.0  MHz of the 14.31818 MHz */
#define I_FREQ_VHS       62700   /* Chroma (I) 627  kHz of the 14.31818 MHz */
#define Q_FREQ_VHS       62700   /* Chroma (Q) 627  kHz of the 14.31818 MHz */

/* IRE units (100 = 1.0V, -40 = 0.0V) */
#define WHITE_LEVEL      100
#define BURST_LEVEL      20
#define BLACK_LEVEL      7
#define BLANK_LEVEL      0
#define SYNC_LEVEL      -40

/* crt_core.h
 *
 * The demodulator. This is also where you can define which system to emulate.
 *
 */

/* library version */
#define CRT_MAJOR 2
#define CRT_MINOR 3
#define CRT_PATCH 1

/* NOTE: this library does not use the alpha channel at all */
#define CRT_BPP         4

#define CRT_DO_VSYNC    1  /* look for VSYNC */
#define CRT_DO_HSYNC    1  /* look for HSYNC */

struct CRT {
    signed char analog[CRT_INPUT_SIZE];
    signed char inp[CRT_INPUT_SIZE]; /* CRT input, can be noisy */
    unsigned char *out; /* output image */

    int hue, brightness, contrast, saturation; /* common monitor settings */
    int black_point, white_point; /* user-adjustable */
    unsigned v_fac; /* factor to stretch img vertically onto the output img */

    /* internal data */
    int ccf[CRT_CC_VPER][CRT_CC_SAMPLES]; /* faster color carrier convergence */
    int hsync, vsync; /* keep track of sync over frames */
    int rn; /* seed for the 'random' noise */

    const unsigned char *data; /* image data */
    int w, h;       /* width and height of image */
    int field;      /* 0 = even, 1 = odd */
    int frame;      /* 0 = even, 1 = odd */
    int xoffset;    /* x offset in sample space. 0 is minimum value */
    int yoffset;    /* y offset in # of lines. 0 is minimum value */
    int do_vhs;     /* VHS only: 0 = no aberration, 1 = with aberration */
    /* make sure your NTSC_SETTINGS struct is zeroed out before you do anything */
    int iirs_initialized; /* internal state */
};

/* Initializes the library. Sets up filters.
 *   w   - width of the output image
 *   h   - height of the output image
 */
extern void crt_init(struct CRT *v, int w, int h);

extern void crt_modulate(struct CRT *v);

/* Demodulates the NTSC signal generated by crt_modulate()
 *   noise - the amount of noise added to the signal (0 - inf)
 */
extern void crt_demodulate(struct CRT *v, int noise);

/*****************************************************************************/
/*************************** FIXED POINT SIN/COS *****************************/
/*****************************************************************************/

#define T14_2PI           16384
#define T14_MASK          (T14_2PI - 1)
#define T14_PI            (T14_2PI / 2)

extern void crt_sincos14(int *s, int *c, int n);

#endif
