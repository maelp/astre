#ifndef _VISION_CORE_H
#define _VISION_CORE_H

/*
    ASTRE a-contrario single trajectory extraction
    Copyright (C) 2011 Mael Primet (mael.primet AT gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* TODO: check what libraries are no longer required */
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>        /* Math functions */
#include "cbase/cbase.h" /* General utilities */
#include "vision/mini_megawave.h" /* MegaWave functions and structures */

/* Typedefs for long type names */
typedef unsigned int uint ;
typedef unsigned char uchar ;

#define swap(a,b) { \
  __typeof__(b) _m_swap_t = (b) ; \
  b = (a) ; \
  a = _m_swap_t ; \
}

/* BASIC FUNCTIONS AND MACROS */
/* ------------------------------------------------- */
static inline double max_d(double a, double b) { return (a>b)?a:b ; }
static inline float  max_f(float a, float b) { return (a>b)?a:b ; }
static inline int    max_i(int a, int b) { return (a>b)?a:b ; }
static inline uint   max_ui(uint a, uint b) { return (a>b)?a:b ; }

static inline double min_d(double a, double b) { return (a<b)?a:b ; }
static inline float  min_f(float a, float b) { return (a<b)?a:b ; }
static inline int    min_i(int a, int b) { return (a<b)?a:b ; }
static inline uint   min_ui(uint a, uint b) { return (a<b)?a:b ; }

static inline double abs_d(double x) { return (x>=0)?x:-x ; }
static inline float  abs_f(float x) { return (x>=0)?x:-x ; }
static inline int    abs_i(int x) { return (x>=0)?x:-x ; }

static inline double clamp_d(double v, double m, double M) { return min_d(M, max_d(m, v)) ; }
static inline float  clamp_f(float v, float m, float M) { return min_f(M, max_f(m, v)) ; }
static inline int    clamp_i(int v, int m, int M) { return min_i(M, max_i(m, v)) ; }
static inline uint   clamp_ui(uint v, uint m, uint M) { return min_ui(M, max_ui(m, v)) ; }

static inline int    in_int_d(double x, double a, double b) { return x >= a && x <= b ; }
static inline int    in_int_f(float x, float a, float b) { return x >= a && x <= b ; }
static inline int    in_int_i(int x, int a, int b) { return x >= a && x <= b ; }
static inline int    in_int_ui(uint x, uint a, uint b) { return x >= a && x <= b ; }
static inline int    in_frame( int x, int y, int nc, int nr ) { return x >= 0 && x < nc && y >= 0 && y < nr ; }

/* index <-> coordinates */
static inline uint idx_coords(uint x, uint y, uint nc) { return (y*nc + x) ; }
static inline uint coordx_idx(uint i, uint nc) { return (i % nc) ; }
static inline uint coordy_idx(uint i, uint nc) { return (i / nc) ; }
#define coords_idx(i,x,y,nc) { \
  uint core_tmp_coords_idx = i ; \
  x = coordx_idx(core_tmp_coords_idx, nc) ; \
  y = coordy_idx(core_tmp_coords_idx, nc) ; \
}

/* Endianness conversion */
#define BYTESWAP16(n) \
  ( ((((uint16_t) n) << 8) & 0xFF00) | \
    ((((uint16_t) n) >> 8) & 0x00FF) )

#define BYTESWAP32(n) \
  ( ((((uint32_t) n) << 24) & 0xFF000000) | \
    ((((uint32_t) n) << 8) & 0x00FF0000) | \
    ((((uint32_t) n) >> 8) & 0x0000FF00) | \
    ((((uint32_t) n) >> 24) & 0x000000FF) )

/* Init random generator with current time and process pid */
void init_rgen();

/* Quicksort comparison helpers */
static int
compar_i_ascend(const void *a, const void *b)
{
  return a - b ;
}
static int
compar_i_descend(const void *a, const void *b)
{
  return b - a ;
}
static int
compar_f_ascend(const void *a, const void *b)
{
  if( *(float*)a > *(float*)b ) return 1 ;
  else if( *(float*)a == *(float*)b ) return 0 ;
  else return -1 ;
}
static int
compar_f_descend(const void *a, const void *b)
{
  if( *(float*)a > *(float*)b ) return -1 ;
  else if( *(float*)a == *(float*)b ) return 0 ;
  else return 1 ;
}
static int
compar_d_ascend(const void *a, const void *b)
{
  if( *(double*)a > *(double*)b ) return 1 ;
  else if( *(double*)a == *(double*)b ) return 0 ;
  else return -1 ;
}
static int
compar_d_descend(const void *a, const void *b)
{
  if( *(double*)a > *(double*)b ) return -1 ;
  else if( *(double*)a == *(double*)b ) return 0 ;
  else return 1 ;
}

/* DIRECT INPUT / OUTPUT FUNCTIONS */
/* ------------------------------------------------- */
char file_is_readable(char *filename);
char file_is_writeable(char *filename);

/* ALLOCATION FUNCTIONS */
/* ------------------------------------------------- */
static inline void * malloc_or_die(size_t n) {
  void *p = malloc(n) ;
  if (p == NULL){ C_log_error("Not enough memory !\n"); exit(-1); }
  return p ;
}
static inline void * calloc_or_die(size_t nmemb, size_t size) {
  void *p = calloc(nmemb, size) ;
  if (p == NULL){ C_log_error("Not enough memory !\n"); exit(-1); }
  return p ;
}
static inline void * realloc_or_die(void *ptr, size_t n) {
  void *p = realloc(ptr,n) ;
  if (p == NULL){ C_log_error("Not enough memory !\n"); exit(-1); }
  return p ;
}

#endif
