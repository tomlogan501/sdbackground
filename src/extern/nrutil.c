/* --------------- */
/* --- nralloc --- */
/* --------------- */

/*
 * Copyright (c) 2000 - 2003, Lionel Lacassagne
 * Ensta version
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <ctype.h> 
#include <string.h>
#include <math.h> /* fabs */

#include "def.h"
#include "nrutil.h"

//NR_END est maintenant defini dans nrutil.h

//#define NR_END 1
//#define FREE_ARG char*

long nr_end = NR_END;

/* ------------------------- */
void nrerror(char error_text[])
/* ------------------------- */
/* Numerical Recipes standard error handler */
{
  fprintf(stderr,"Numerical Recipes run-time error...\n");
  fprintf(stderr,"%s\n",error_text);
  fprintf(stderr,"...now exiting to system...\n");
  exit(1);
}
/* ------------------------------ */
float64* f64vector(long nl, long nh)
/* ------------------------------ */
{
  float64 *v;

  v=(float64 *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float64)));
  if (!v) nrerror("allocation failure in f64vector()");
  if(!v) return NULL;
  return v-nl+NR_END;
}
/* ------------------------------------------------ */
byte** bmatrix(long nrl, long nrh, long ncl, long nch)
/* ------------------------------------------------ */
/* allocate an byte matrix with subscript range m[nrl..nrh][ncl..nch] */
{
  long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
  byte **m;

  /* allocate pointers to rows */
  m=(byte **) malloc((size_t)((nrow+NR_END)*sizeof(byte*)));
  if (!m) nrerror("allocation failure 1 in bmatrix()");
  m += NR_END;
  m -= nrl;

  /* allocate rows and set pointers to them */
  m[nrl]=(byte *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(byte)));
  if (!m[nrl]) nrerror("allocation failure 2 in bmatrix()");
  m[nrl] += NR_END;
  m[nrl] -= ncl;

  for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

  /* return pointer to array of pointers to rows */
  return m;
}
/* --------------------------------------------------- */
rgb8** rgb8matrix(long nrl, long nrh, long ncl, long nch)
/* -------------------------------------------------- */
/* allocate an rgb8 matrix with subscript range m[nrl..nrh][ncl..nch] */
{
  long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
  rgb8 **m;

  /* allocate pointers to rows */
  m=(rgb8**) malloc((size_t)((nrow+NR_END)*sizeof(rgb8*)));
  if (!m) nrerror("allocation failure 1 in rgb8matrix()");
  m += NR_END;
  m -= nrl;

  /* allocate rows and set pointers to them */
  m[nrl]=(rgb8*) malloc((size_t)((nrow*ncol+NR_END)*sizeof(rgb8)));
  if (!m[nrl]) nrerror("allocation failure 2 in rgb8matrix()");
  m[nrl] += NR_END;
  m[nrl] -= ncl;

  for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

  /* return pointer to array of pointers to rows */
  return m;
}
/* ------------------------------------------- */
void free_f64vector(float64 *v, long nl)
/* ------------------------------------------- */
/* free a double vector allocated with f64vector() */
{
  free((FREE_ARG) (v+nl-NR_END));
}
/* ------------------------------------------------------------- */
void free_bmatrix(byte **m, long nrl, long nrh, long ncl)
/* ------------------------------------------------------------- */
/* free an uchar matrix allocated by bmatrix() */
{
  free((FREE_ARG) (m[nrl]+ncl-NR_END));
  free((FREE_ARG) (m+nrl-NR_END));
}
/* ---------------------------------------------------------------- */
void free_rgb8matrix(rgb8 **m, long nrl, long nrh, long ncl)
/* ---------------------------------------------------------------- */
/* free an uchar matrix allocated by rgb8matrix() */
{
  free((FREE_ARG) (m[nrl]+ncl-NR_END));
  free((FREE_ARG) (m+nrl-NR_END));
}
/* ------------------------ */
/* -- PGM IO for bmatrix -- */
/* ------------------------ */

char *readitem   (FILE *file, char *buffer);
void  ReadPGMrow (FILE *file, long width, byte  *line);
void  WritePGMrow(byte *line, long width, FILE  *file);

/* --------------------------------- */
char *readitem(FILE *file,char *buffer)
/* --------------------------------- */
/* lecture d'un mot */
{
  char *aux;
  int k;

  k=0;
  aux=buffer;
  while (!feof(file))
    {
      *aux=fgetc(file);
      switch(k)
        {
        case 0:
          if (*aux=='#') k=1;
          if (isalnum(*aux)) k=2,aux++;
          break;
        case 1:
          if (*aux==0xA) k=0;
          break;
        case 2:
          if (!isalnum(*aux))
            {
              *aux=0;
              return buffer;
            }
          aux++;
          break;
        }
    }
  *aux=0;
  return buffer;
}
/* ---------------------------------------------- */
void ReadPGMrow(FILE *file, long width, byte  *line)
/* ---------------------------------------------- */
{
    /* Le fichier est ouvert (en lecture) et ne sera pas ferme a la fin */
     fread(&(line[0]), sizeof(byte), width, file);
}
/* ----------------------------------------------- */
void WritePGMrow(byte *line, long width, FILE  *file)
/* ----------------------------------------------- */
{
/* Le fichier est deja ouvert et ne sera pas ferme a la fin */

   fwrite(&(line[0]), sizeof(byte), width, file);
}
/* ---------------------------------------------------------------------------- */
byte** LoadPGM_bmatrix(char *filename, long *nrl, long *nrh, long *ncl, long *nch)
/* ---------------------------------------------------------------------------- */
{
  /* cette version ne lit plus que le type P5 */

  long height, width, gris;
  byte **m;
  FILE *file;
  /*int   format;*/

  char *buffer;
  /*char  c;*/
  int i;
  
  buffer = (char*) calloc(80, sizeof(char));
  /* ouverture du fichier */
  file = fopen(filename,"rb");
  if (file==NULL)
    nrerror("ouverture du fichier impossible\n");
    //nrerror("ouverture du fichier %s impossible\n", filename);

  /* lecture de l'entete du fichier pgm */
  readitem(file, buffer);
  /*fscanf(fichier, "%s", buffer);*/
  if(strcmp(buffer, "P5") != 0)
    nrerror("entete du fichier %s invalide\n");
    //nrerror("entete du fichier %s invalide\n", filename);

  width  = atoi(readitem(file, buffer));
  height = atoi(readitem(file, buffer));
  gris   = atoi(readitem(file, buffer));

  *nrl = 0;
  *nrh = height - 1;
  *ncl = 0;
  *nch = width - 1;
  m = bmatrix(*nrl, *nrh, *ncl, *nch);
  
  for(i=0; i<height; i++) {
    ReadPGMrow(file, width, m[i]);
  }

  fclose(file);
  free(buffer);

  return m;
}
/* ----------------------------------------------------------------------------- */
void MLoadPGM_bmatrix(char *filename, int nrl, int nrh, int ncl, int nch, byte **m)
/* ----------------------------------------------------------------------------- */
{
    /* cette version ne lit plus que le type P5 */
    
    int height, width, gris;
    FILE *file;

    char *buffer;
    int i;

    buffer = (char*) calloc(80, sizeof(char));
    /* ouverture du fichier */
    file = fopen(filename,"rb");
    if (file==NULL)
        nrerror("ouverture du fichier impossible\n");
    //nrerror("ouverture du fichier %s impossible\n", filename);

    /* lecture de l'entete du fichier pgm */
    readitem(file, buffer);
    /*fscanf(fichier, "%s", buffer);*/
    if(strcmp(buffer, "P5") != 0)
        nrerror("entete du fichier %s invalide\n");
    //nrerror("entete du fichier %s invalide\n", filename);

    width  = atoi(readitem(file, buffer));
    height = atoi(readitem(file, buffer));
    gris   = atoi(readitem(file, buffer));
    
    for(i=0; i<height; i++) {
        ReadPGMrow(file, width, &(m[i][ncl]));
        //ReadPGMrow(file, width,m[i]);
    }
    
    fclose(file);
    free(buffer);
}
/* -------------------------------------------------------------------------------- */
void SavePGM_bmatrix(byte **m, long nrl, long nrh, long ncl, long nch, char *filename)
/* -------------------------------------------------------------------------------- */
{
  long nrow = nrh-nrl+1;
  long ncol = nch-ncl+1;

  char buffer[80];
  
  FILE *file;
  int  i;

  file = fopen(filename, "wb");
  if (file == NULL)
    //nrerror("ouverture du fichier %s impossible dans SavePGM_bmatrix\n", filename);
    nrerror("ouverture du fichier %s impossible dans SavePGM_bmatrix\n");

  /* enregistrement de l'image au format rpgm */

  sprintf(buffer,"P5\n%d %d\n255\n",ncol, nrow);
  fwrite(buffer,strlen(buffer),1,file);
  for(i=nrl; i<=nrh; i++)
    WritePGMrow(m[i], ncol, file);

  /* fermeture du fichier */
  fclose(file);
}
/* --------------------------- */
/* -- PNM IO for rgb8matrix -- */
/* --------------------------- */

/* ----------------------------------------------- */
void ReadPNMrow(FILE  *file, long width, byte  *line)
/* ----------------------------------------------- */
{
    /* Le fichier est ouvert (en lecture) et ne sera pas ferme a la fin */
     fread(&(line[0]), sizeof(byte), 3*sizeof(byte)*width, file);
}
/* ------------------------------------------------ */
void WritePNMrow(byte  *line, long width, FILE  *file)
/* ------------------------------------------------ */
{
/* Le fichier est deja ouvert et ne sera pas ferme a la fin */

   fwrite(&(line[0]), sizeof(byte), 3*sizeof(byte)*width, file);
}
/* ------------------------------------------------------------------------------- */
rgb8** LoadPPM_rgb8matrix(char *filename, long *nrl, long *nrh, long *ncl, long *nch)
/* ------------------------------------------------------------------------------- */
{
  /* cette version ne lit plus que le type P6 */

  long height, width, gris;
  rgb8 **m;
  FILE *file;
  /*int   format;/**/

  char *buffer;
  /*char  c;/**/
  int i;
  
  buffer = (char*) calloc(80, sizeof(char));
  /* ouverture du fichier */
  file = fopen(filename,"rb");
  if (file==NULL)
    nrerror("ouverture du fichier impossible\n");
    //nrerror("ouverture du fichier %s impossible\n", filename);

  /* lecture de l'entete du fichier pgm */
  readitem(file, buffer);
  /*fscanf(fichier, "%s", buffer);*/
  if(strcmp(buffer, "P6") != 0)
    nrerror("entete du fichier %s invalide\n");
    //nrerror("entete du fichier %s invalide\n", filename);

  width  = atoi(readitem(file, buffer));
  height = atoi(readitem(file, buffer));
  gris   = atoi(readitem(file, buffer));

  *nrl = 0;
  *nrh = height - 1;
  *ncl = 0;
  *nch = width - 1;
  m = rgb8matrix(*nrl, *nrh, *ncl, *nch);
  
  for(i=0; i<height; i++) {
    ReadPNMrow(file, width, (byte*)m[i]);
  }

  fclose(file);
  free(buffer);

  return m;
}
/* ----------------------------------------------------------------------------------- */
void SavePPM_rgb8matrix(rgb8 **m, long nrl, long nrh, long ncl, long nch, char *filename)
/* ----------------------------------------------------------------------------------- */
{
  long nrow = nrh-nrl+1;
  long ncol = nch-ncl+1;

  char buffer[80];
  
  FILE *file;
  int  i;

  file = fopen(filename, "wb");
  if (file == NULL)
    //nrerror("ouverture du fichier %s impossible dans SavePGM_bmatrix\n", filename);
    nrerror("ouverture du fichier %s impossible dans SavePPM_bmatrix\n");

  /* enregistrement de l'image au format rpgm */

  sprintf(buffer,"P6\n%d %d\n255\n",ncol, nrow);
  fwrite(buffer,strlen(buffer),1,file);
  for(i=nrl; i<=nrh; i++)
    WritePNMrow((byte*)m[i], ncol, file);

  /* fermeture du fichier */
  fclose(file);
}
