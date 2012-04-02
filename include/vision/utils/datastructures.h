#ifndef _VISION_DATASTRUCTURES_H
#define _VISION_DATASTRUCTURES_H

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

#include <vision/core.h>

/* Quick and dirty RESIZEABLE BUFFER structure */
/* ------------------------------------------------- */
typedef struct st_resizable_buf {
  int allocated_size ;
  int size ;
  char *data ;
} *resizable_buf ;

static const int rb_resize_step = 4*200 ; /* add 4*200 bytes */

resizable_buf rb_empty();

resizable_buf rb_new(int init_size);

void rb_free(resizable_buf b);
void rb_ensure_space(resizable_buf b, int n);
int rb_get_endpos(resizable_buf b);

#define rb_pack_c    rb_pack_i8
#define rb_pack_uc   rb_pack_ui8
#define rb_pack_i    rb_pack_i32
#define rb_pack_ui   rb_pack_ui32

void rb_pack_i8(resizable_buf b, int8_t data);
void rb_pack_ui8(resizable_buf b, uint8_t data);
void rb_pack_i16(resizable_buf b, int16_t data);
void rb_pack_ui16(resizable_buf b, uint16_t data);
void rb_pack_i32(resizable_buf b, int32_t data);
void rb_pack_ui32(resizable_buf b, uint32_t data);
void rb_pack_f(resizable_buf b, float data);
void rb_pack_d(resizable_buf b, double data);
/* Include the terminating NULL character */
void rb_pack_s(resizable_buf b, char *data);
/* Do not include terminating NULL character */
void rb_pack_text(resizable_buf b, char *data);
void rb_pack_array(resizable_buf b, char *array, uint n_elts, uint size_elt);
/* Assume there is enough space to store the data */
void rb_store_ui(resizable_buf b, int pos, uint data);

/* Quick and dirty FILE LIST structure */
/* ------------------------------------------------- */
typedef struct st_filelist {
  int n ;
  char ** filenames ;
} *filelist ;

filelist filelist_new ();
filelist filelist_load(Rawdata input, char *header);
void filelist_save( filelist f, char *header, Rawdata output );
void filelist_delete(filelist A);
int filelist_save_rawdata(filelist A,int n,Rawdata from);
Rawdata filelist_load_rawdata(filelist A,int n);

/* Quick and dirty PARSE ARRAY structure */
/* ------------------------------------------------- */
typedef struct st_parse_array {
  uint size ;
  char *data ;
  char *ptr ;
} *parse_array ;

parse_array pa_new(uint size, char *data);
/* do not free the data array, only the allocated structure */
void pa_free_structure(parse_array pa);
/* Int 8, 16, 32, 64 */
int pa_read_i8(parse_array pa, int8_t *data);
int pa_skip_i8(parse_array pa);
int pa_read_i16(parse_array pa, int16_t *data);
int pa_skip_i16(parse_array pa);
int pa_read_i32(parse_array pa, int32_t *data);
int pa_skip_i32(parse_array pa);
/* UInt 8, 16, 32, 64 */
int pa_read_ui8(parse_array pa, uint8_t *data);
int pa_skip_ui8(parse_array pa);
int pa_read_ui16(parse_array pa, uint16_t *data);
int pa_skip_ui16(parse_array pa);
int pa_read_ui32(parse_array pa, uint32_t *data);
int pa_skip_ui32(parse_array pa);

#define pa_read_c      pa_read_i8
#define pa_skip_c      pa_skip_i8
#define pa_read_uc     pa_read_ui8
#define pa_skip_uc     pa_skip_ui8
#define pa_read_i      pa_read_i32
#define pa_skip_i      pa_skip_i32
#define pa_read_ui     pa_read_ui32
#define pa_skip_ui     pa_skip_ui32

int pa_read_f(parse_array pa, float *data);
int pa_skip_f(parse_array pa);
int pa_read_d(parse_array pa, double *data);
int pa_skip_d(parse_array pa);
#define pa_skip_d_be pa_skip_d
int pa_read_array(parse_array pa, char **data, uint *n_elts);
int pa_skip_array(parse_array pa);
#define pa_skip_array_be pa_skip_array

/* returns non-null int if buffer too small or string not matched, updated the
 * pointer (go past the null-byte) */
int pa_expect_s(parse_array pa, char *str);
int pa_expect_text(parse_array pa, char *str);

/* Return the size of the string in parse_array (including the '\n' character),
 * copy the string in str and increment pa->ptr if buffer_len permits,
 * returns -1 if there is nothing more to read */
int pa_read_line(parse_array pa, char* str, int buffer_len);
/* Make sure the array is big enough to read the string, and add a null terminator after
 * the string */
int pa_read_line_clean( parse_array pa, char** p_str, int* p_buffer_len );

#endif
