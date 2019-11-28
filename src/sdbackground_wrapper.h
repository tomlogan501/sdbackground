/*
 * Include to link external code of antoine without changing anything
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "extern/def.h"
#include "extern/nrutil.h"

//C source code
extern "C"
{
void Routine_Put_Initial_Value(int i0, int i1, int j0, int j1, byte x_init, byte **X);
void routine_ComputeFore(byte **D, byte **V, int i0, int i1, int j0, int j1, byte **E);
void routine_AbsoluteDifference(byte **I0, byte **I1, int i0, int i1, int j0, int j1, byte **D);
void routine_UpdateV(byte **Y, int i0, int i1, int j0, int j1, int N, byte v_min, byte v_max, byte **V);
void routine_UpdateM(byte **X, int i0, int i1, int j0, int j1, byte **M);
double dtime(void);
long long readTSC (void);
void printCPP(char *str, double dt,  int size);
};
