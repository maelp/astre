#ifndef _VISION_MINIMEGAWAVE_H
#define _VISION_MINIMEGAWAVE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>      /* For variadic functions */
#include "cbase/cbase.h"

/* Since the original code has been developped with MegaWave, some of
 * its functions and structures need to be included. */

/* ERROR LOGGING */
/* ------------------------------------------------- */
/* Made to be coherent with MegaWave mwerror */
#define WARNING  0
#define ERROR    1
#define FATAL    2
#define USAGE    3
#define INTERNAL 4
void mini_mwerror( int code, int exit_code, char* fmt, ... );

/* Rawdata */
/* ------------------------------------------------- */
typedef struct rawdata {
  int size;               /* Number of samples */
  unsigned char *data;    /* data field */
} *Rawdata;

Rawdata mw_new_rawdata(void);
Rawdata mw_alloc_rawdata(Rawdata, int);
void mw_delete_rawdata(Rawdata);
Rawdata mw_change_rawdata(Rawdata, int);
void mw_copy_rawdata(Rawdata, Rawdata);

/* Allocation functions */
/* ------------------------------------------------- */
Rawdata change_rawdata_or_die(Rawdata rd, int newsize);
Rawdata alloc_rawdata_or_die(Rawdata rd, int size);
Rawdata new_rawdata_or_die();

/* Loading and saving a Rawdata file */
/* ------------------------------------------------- */
int save_rawdata( Rawdata rd, char* fname );
Rawdata load_rawdata( char* fname );

#endif
