/*
 * Include to link external code of antoine without changing anything
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "extern/def.h"

extern "C"
{
    //C source code
    void Routine_Put_Initial_Value(int i0, int i1, int j0, int j1, byte x_init, byte **X);

};
