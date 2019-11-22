/* ----------------- */
/* --- SD_zipf.c --- */
/* ----------------- */
/* ---- Author: ---- */
/* - A. Manzanera -- */
/* ----------------- */

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
  if (b0<b1) return -1;
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
void routine_ConditionalUpdateM(byte **X, byte **V, byte seuil, int i0, int i1, int j0, int j1, byte **M)
/* ------------------------------------------------------------------------------ */
{
  int i,j;
  for(i=i0; i<=i1; i++) {
    for(j=j0; j<=j1; j++) {
      if (V[i][j] > seuil) {
	if (X[i][j] < M[i][j]) M[i][j]--;
	else if (X[i][j] > M[i][j]) M[i][j]++;
      }
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
      D[i][j] = abs(I0[i][j] - I1[i][j]);
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

/* ------------------------------------------------------------------------------ */
void routine_UpdateN(byte **E, int i0, int i1, int j0, int j1, float fg_pts_rate, float fg_isolated_pts_rate, int *N)
/* ------------------------------------------------------------------------------ */
{
  int i,j;
  int nb_pts = 0;
  int nb_pts_isoles = 0;
  for(i=i0; i<=i1; i++) {
    for(j=j0; j<=j1; j++) {
      if (E[i][j]) {
	nb_pts++;
	if (!(E[i-1][j] || E[i+1][j] || E[i][j-1] || E[i][j+1]))
	  nb_pts_isoles++;
      }
    }
  }
  if ((fg_pts_rate*(i1-i0)*(j1-j0) < nb_pts) || 
      (fg_isolated_pts_rate*(i1-i0)*(j1-j0) < nb_pts_isoles)) (*N)++;
  else (*N)--;
  // printf("N = %d\n",*N);
}


#define CHRONO(X, str) t1=dtime(); X; t2 = dtime(); dt = t2-t1; printCPP(str, dt, size); 

/* --------------------------- */
int main(int argc, char *argv[])
/* --------------------------- */
{
  setFreq(2.4e9);

  char *src_path;
  char *dst_path;
  
  char *filename;
  
  char cfilenameI[1024];
  char cfilenameO[1024];
  
  int i0, i1, j0, j1;
  int width, height;
  int ndigit;

  // Image variables
  byte **I, **M, **V, **E, **D;
  int t, tstart, tstop, tstep;
  int rank,seuil,pow2;

  int N = 1; // Number of deviation, adapted automatically

  // Image sequence parameters
  src_path = "../car3/"; // do not forget "/" at the end (for concatenation...)
  filename = "car_3";
  dst_path = "./SD2/"; 
  ndigit   = 3;
  width    = 320;
  height   = 240;
  // When no spatial processing needed
  int bord0     = 0;
  // Spatial processing: allocate extra border pixels
  int bord1     = 1;

  // Tunable parameters
  float max_fg_rate = 0.5; // Maximal rate of foreground points (N increment condition)
  float max_fg_isolated_rate = 0.01; // Maximal rate of isolated foreground points (N increment condition)
  byte v_min = 2;
  byte v_max = 255; 
  int p_v = 8; // update period of the background variance
  
  // for time measure
  long size;
  double  t1, t2, dt;
  
  printf("src_path = %s\n", src_path);
  printf("filename = %s\n", filename);
  printf("dst_path = %s\n", dst_path);
  
  i0 = 0; i1 = height-1;
  j0 = 0; j1 = width-1;
  
  // images are allocated outside the loop
  // do not allocate inside
  // malloc with borders to tackle border problem
  I = bmatrix(i0-bord0, i1+bord0, j0-bord0, j1+bord0); // current image
  D = bmatrix(i0-bord0, i1+bord0, j0-bord0, j1+bord0); // absolute difference
  M = bmatrix(i0-bord0, i1+bord0, j0-bord0, j1+bord0); // current SD-background
  V = bmatrix(i0-bord0, i1+bord0, j0-bord0, j1+bord0); // current SD-variance  
  E = bmatrix(i0-bord1, i1+bord1, j0-bord1, j1+bord1); // foreground label

  tstart = 0;
  tstop  = 199;
  tstep  = 1;
  
  generate_path_filename_k_ndigit_extension(src_path, filename, tstart, ndigit, "pgm", cfilenameI);
  puts(cfilenameI);
  // SD parameters initialization
  MLoadPGM_bmatrix (cfilenameI, i0, i1, j0, j1, M); // Multiple-Load without allocating matrix
  Routine_Put_Initial_Value(i0, i1, j0, j1, v_min, V);

  for(t=tstart+1; t<=tstop; t+=tstep) {

    // Compute the update threshold "seuil" 
    // according to the time index "t"
    rank = (t%256);
    pow2 = 1;
    seuil = 256;
    do {
      pow2 = pow2*2;
      seuil = seuil/2;
    } while (((rank%pow2)==0)&&(seuil>1));
    
    generate_path_filename_k_ndigit_extension(src_path, filename, t, ndigit, "pgm", cfilenameI); // generate filename of current image
      puts(cfilenameI);
    
      MLoadPGM_bmatrix (cfilenameI, i0, i1, j0, j1, I);  // load current image
      
      // routine of computation
      if (t<tstop) {
	routine_ConditionalUpdateM(I,V,seuil,i0,i1,j0,j1,M);
	routine_AbsoluteDifference(I,M,i0,i1,j0,j1,D);
	if (t % p_v == 0) routine_UpdateV(D,i0,i1,j0,j1,N,v_min,v_max,V);
	routine_ComputeFore(D,V,i0,i1,j0,j1,E);
	if (t % p_v == 0) routine_UpdateN(E,i0,i1,j0,j1,max_fg_rate,max_fg_isolated_rate,&N);
      } else {
	size = (j1-j0+1)*(i1-i0+1);
	CHRONO(routine_ConditionalUpdateM(I,V,seuil,i0,i1,j0,j1,M), "MAJ Conditionnelle Moyenne");
	CHRONO(routine_AbsoluteDifference(I,M,i0,i1,j0,j1,D), "Calcul Difference");
	CHRONO(routine_UpdateV(D,i0,i1,j0,j1,N,v_min,v_max,V), "MAJ Variance");
	CHRONO(routine_ComputeFore(D,V,i0,i1,j0,j1,E), "Calcul Label");
	CHRONO(routine_UpdateN(E,i0,i1,j0,j1,max_fg_rate,max_fg_isolated_rate,&N), "Adaptation du N");
      }
        
      // Put here the results you wish to save:
      // generate filename for output
      generate_path_filename_k_ndigit_extension(dst_path, "E_", t, ndigit, "pgm", cfilenameO);
      // write your stuff
      SavePGM_bmatrix(E, i0, i1, j0, j1, cfilenameO);
      // and so on
      //generate_path_filename_k_ndigit_extension(dst_path, "M_", t, ndigit, "pgm", cfilenameO);
      //SavePGM_bmatrix(M, i0, i1, j0, j1, cfilenameO);
      //generate_path_filename_k_ndigit_extension(dst_path, "V_", t, ndigit, "pgm", cfilenameO);
      //SavePGM_bmatrix(V, i0, i1, j0, j1, cfilenameO);
      //generate_path_filename_k_ndigit_extension(dst_path, "D_", t, ndigit, "pgm", cfilenameO);
      //SavePGM_bmatrix(D, i0, i1, j0, j1, cfilenameO);
  }
  free_bmatrix(I, i0-bord0, i1+bord0, j0-bord0);
  free_bmatrix(M, i0-bord0, i1+bord0, j0-bord0);
  free_bmatrix(V, i0-bord0, i1+bord0, j0-bord0);
  free_bmatrix(E, i0-bord1, i1+bord1, j0-bord1);
  free_bmatrix(D, i0-bord0, i1+bord0, j0-bord0);

  return 0;
}
