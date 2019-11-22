/* ---------------- */
/* --- nrutil.h --- */
/* ---------------- */
#ifndef __NRUTIL_H__
#define __NRUTIL_H__


#define NR_END 0
#define FREE_ARG char*

extern long nr_end;

/* ------------------------------- */
/* -- vector & matrix allocator -- */
/* ------------------------------- */

 float64* f64vector(long nl, long nh);
byte** bmatrix   (long nrl, long nrh, long ncl, long nch);
rgb8** rgb8matrix(long nrl, long nrh, long ncl, long nch);

void free_f64vector(float64 *v, long nl);
void free_bmatrix(byte **m, long nrl, long nrh, long ncl);

/* ------------------------------- */
/* -- PGM and PNM binary format -- */
/* ------------------------------- */

byte** LoadPGM_bmatrix(char *filename, long *nrl, long *nrh, long *ncl, long *nch);
void   MLoadPGM_bmatrix(char *filename, int nrl, int nrh, int ncl, int nch, byte **m);
void    SavePGM_bmatrix(byte **m,       long  nrl, long  nrh, long  ncl, long  nch, char *filename);

rgb8 ** LoadPPM_rgb8matrix(char *filename, long *nrl, long *nrh, long *ncl, long *nch);
void    SavePPM_rgb8matrix(rgb8 **m,       long  nrl, long  nrh, long  ncl, long  nch, char *filename);

#endif // __NRUTL_H__
