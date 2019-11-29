/* ------------------- */
/* --- SD_basics.c --- */
/* ------------------- */
/* ---- Author: ------ */
/* -- A. Manzanera --- */
/* ------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "def.h"
#include "nrutil.h"

float Freq;

/* ------------------ */
long long readTSC (void)
/* ------------------ */
{
    long long t;
    asm volatile (".byte 0x0f,0x31" : "=A" (t));

    return t;
}
/* ------------ */
double dtime(void)
/* ------------ */
{
    return (double) readTSC();
}
/* ----------------- */
void setFreq(float f)
/* --------------- */
{
    Freq = f;
    printf("Freq = %.0f MHz\n", Freq/1e6);
}
/* ---------------------------------------- */
void printCPP(char *str, double dt,  int size)
/* ---------------------------------------- */
{
    float cpp, ms, df;

    df = (float) dt;
    cpp = (float) dt / (float) size;
    ms = 1000.0f * (float) dt / (float) Freq;

    printf("%20s : ", str);
    printf("cycles = %10.0f ;", df);
    printf("cpp = %6.2f c/p ;", cpp);
    printf("t = %6.2f ms\n", ms);
}
/* --------------------------------------------------------------------------------------------------------------------------------- */
void generate_path_filename_k_ndigit_extension(char *path, char *filename, int k, int ndigit, char *extension, char *complete_filename)
/* --------------------------------------------------------------------------------------------------------------------------------- */
{
  char *format = "%s%s%d.%s";

  switch(ndigit) {
    case 0 : format = "%s%s%d.%s";   break;
    case 1 : format = "%s%s%01d.%s"; break;
    case 2 : format = "%s%s%02d.%s"; break;
    case 3 : format = "%s%s%03d.%s"; break;
    case 4 : format = "%s%s%04d.%s"; break;
    case 5 : format = "%s%s%05d.%s"; break;
    case 6 : format = "%s%s%06d.%s"; break;
    case 7 : format = "%s%s%07d.%s"; break;
    case 8 : format = "%s%s%08d.%s"; break;
    case 9 : format = "%s%s%09d.%s"; break;
  }
  sprintf(complete_filename, format, path, filename, k, extension);
}

byte sign(byte b0, byte b1) {
  if (b0<b1) return (byte)-1;
  else if (b0>b1) return 1;
  else return 0;
}

/* ------------------------------------------------------------------------------ */
void Routine_Put_Initial_Value(int i0, int i1, int j0, int j1, byte x_init, byte **X)
/* ------------------------------------------------------------------------------ */
{
  int i,j;
  for(i=i0; i<=i1; i++) {
    for(j=j0; j<=j1; j++) {
      X[i][j] = x_init;
    }
  }
}

/* ------------------------------------------------------------------------------ */
void routine_UpdateM(byte **X, int i0, int i1, int j0, int j1, byte **M)
/* ------------------------------------------------------------------------------ */
{
  int i,j;
  for(i=i0; i<=i1; i++) {
    for(j=j0; j<=j1; j++) {
      if (X[i][j] < M[i][j]) M[i][j]--;
      else if (X[i][j] > M[i][j]) M[i][j]++;
    }
  }
}

/* ------------------------------------------------------------------------------ */
void routine_UpdateV(byte **Y, int i0, int i1, int j0, int j1, int N, byte v_min, byte v_max, byte **V)
/* ------------------------------------------------------------------------------ */
{
  int i,j;
  for(i=i0; i<=i1; i++) {
    for(j=j0; j<=j1; j++) {
      if (N*Y[i][j] < V[i][j]) V[i][j]--;
      else V[i][j]++;
      if (V[i][j]>v_max) V[i][j]=v_max;
      else if (V[i][j]<v_min) V[i][j]=v_min;
    }
  }
}

/* ------------------------------------------------------------------------------ */
void routine_AbsoluteDifference(byte **I0, byte **I1, int i0, int i1, int j0, int j1, byte **D)
/* ------------------------------------------------------------------------------ */
{
  int i,j;
  for(i=i0; i<=i1; i++) {
    for(j=j0; j<=j1; j++) {
      D[i][j] = (byte) abs(I0[i][j] - I1[i][j]);
    }
  }
}

/* ------------------------------------------------------------------------------ */
void routine_ComputeFore(byte **D, byte **V, int i0, int i1, int j0, int j1, byte **E)
/* ------------------------------------------------------------------------------ */
{
  int i,j;
  for(i=i0; i<=i1; i++) {
    for(j=j0; j<=j1; j++) {
      if (D[i][j] < V[i][j]) E[i][j] = 0;
      else E[i][j] = 255;
    }
  }
}
