#include <vision/core.h>
#include <vision/utils/datastructures.h>

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

/* Quick and dirty RESIZEABLE BUFFER structure */
/* ------------------------------------------------- */

resizable_buf
rb_empty() {
  resizable_buf b = (resizable_buf) malloc_or_die(sizeof(struct st_resizable_buf)) ;
  b->allocated_size = b->size = 0 ;
  b->data = NULL ;
  return b ;
}

resizable_buf
rb_new(int init_size) {
  resizable_buf b = (resizable_buf) malloc_or_die(sizeof(struct st_resizable_buf)) ;
  b->allocated_size = init_size ;
  b->size = 0 ;
  b->data = (char*) malloc_or_die(init_size) ;
  return b ;
}

void
rb_free(resizable_buf b) {
  if (b->data) free (b->data) ;
  free (b) ;
}

void
rb_ensure_space(resizable_buf b, int n) {
  if (b->size + n > b->allocated_size) {
    int new_size = b->allocated_size + max_i(rb_resize_step,b->size + n - b->allocated_size) ;
    b->data = (char*)realloc_or_die((void*)b->data,new_size) ;
    b->allocated_size = new_size ;
  }
}

int
rb_get_endpos(resizable_buf b) {
  return b->size ;
}

/* Include the terminating NULL character */
void rb_pack_s(resizable_buf b, char *data) {
  int str_size = strlen(data) +1 ; /* include the null character */
  rb_ensure_space(b,str_size) ;
  memcpy(b->data + b->size, data, str_size) ;
  b->size += str_size ;
}
/* Do not include terminating NULL character */
void rb_pack_text(resizable_buf b, char *data) {
  int str_size = strlen(data) ; /* do not include the null character */
  rb_ensure_space(b,str_size) ;
  memcpy(b->data + b->size, data, str_size) ;
  b->size += str_size ;
}

/* Quick and dirty FILE LIST structure */
/* ------------------------------------------------- */
filelist
filelist_new ()
{
  filelist t = (filelist) malloc_or_die( sizeof(struct st_filelist) );
  t->n = 0 ;
  t->filenames = (char**)NULL ;

  return t ;
}

filelist
filelist_load(Rawdata input, char *header)
{
    char file_buffer[BUFSIZ] ;
    char *p = input->data ;
    char *lst = input->data + input->size ;
    filelist list = (filelist) malloc_or_die(sizeof(struct st_filelist)) ;

    /* read a line in file_buffer, and return 0 if everything has been read */
      int readline() {
      /*{{{*/
        if (p == NULL || p == lst) return FALSE ;
        char *s = p ;
        while (p < lst && *p != '\n') p++ ;
        char *dest = file_buffer ;
        while (s < p) {
          if (dest == file_buffer + BUFSIZ - 1)
          {
            C_log_error("Buffer overflow...\n") ;
            exit(-1);
          }
          *(dest++) = *(s++) ;
        }
        *dest = 0 ;
        p++ ;
        return TRUE ;
      /*}}}*/
      }
    /* Error function */
      void malformed() {
        C_log_error("[filelist_load] Malformed %s file...\n", header);
        exit(-1);
      }

    /* Parse FileList */
      if (!readline()) malformed() ;
        if (strcmp(file_buffer,header) != 0) malformed() ;
      if (!readline()) malformed() ;
        list->n = atoi(file_buffer) ;

    if (list->n <= 0 || list->n > 10000)
    {
      C_log_error("[filelist_load] Invalid number of frames : %d\n",list->n);
      exit(-1);
    }

    list->filenames = (char**)malloc_or_die(list->n*sizeof(char*)) ;

    for (int i = 0 ; i < list->n ; i++) {
      if (!readline()) malformed() ;
      list->filenames[i] = (char*) malloc_or_die(strlen(file_buffer)+1) ;
      strcpy(list->filenames[i],file_buffer) ;
    }

    return list ;
}

void
filelist_save( filelist f, char *header, Rawdata output )
{
  resizable_buf rb = rb_new(30*sizeof(char)*(f->n+1)) ;

  char str[50] ;

  rb_pack_text( rb, header );
  rb_pack_text( rb, "\n" );

  sprintf( str, "%d", f->n );
  rb_pack_text( rb, str );
  rb_pack_text( rb, "\n" );

  for( int k = 0 ; k < f->n ; k++ )
  {
    rb_pack_text( rb, f->filenames[k] );
    rb_pack_text( rb, "\n" );
  }

  output = change_rawdata_or_die( output, rb->size );
  memcpy( output->data, rb->data, rb->size );
  rb_free( rb );
}

void
filelist_delete(filelist A)
{
  for( int k = 0 ; k < A->n ; k++ )
  {
    if( A->filenames[k] )
    {
      free( A->filenames[k] );
      A->filenames[k] = (char*)NULL ;
    }
  }
  free(A->filenames) ; A->filenames = (char**)NULL ;
  free(A) ;

  return ;
}
int
filelist_save_rawdata(filelist A,int n,Rawdata from)
{
  if (n < 0 || n >= A->n) mini_mwerror(FATAL,1,"[filelist_save_rawdata] Invalid frame number!\n") ;
  char *filename = A->filenames[n] ;
  if (!file_is_writeable(filename)) {
    perror("[filelist_save_rawdata]") ;
    mini_mwerror(FATAL,1,"Cannot write file %s!\n",filename) ;
  }
  return save_rawdata(from, A->filenames[n]) ;
}
Rawdata
filelist_load_rawdata(filelist A,int n)
{
  if (n < 0 || n >= A->n) mini_mwerror(FATAL,1,"[filelist_load_rawdata] Invalid frame number!\n") ;
  char *filename = A->filenames[n] ;
  if (!file_is_readable(filename)) {
    perror("[filelist_load_rawdata]") ;
    mini_mwerror(FATAL,1,"Cannot read file %s!\n",filename) ;
  }
  return load_rawdata(A->filenames[n]) ;
}

/* Quick and dirty PARSE ARRAY structure */
/* ------------------------------------------------- */
parse_array pa_new(uint size, char *data) {
  parse_array pa = (parse_array)malloc(sizeof(struct st_parse_array)) ;
  pa->size = size ;
  pa->data = data ;
  pa->ptr = data ;
  return pa ;
}
parse_array pa_new_bigendian(uint size, char *data) {
  parse_array pa = (parse_array)malloc(sizeof(struct st_parse_array)) ;
  pa->size = size ;
  pa->data = data ;
  pa->ptr = data ;
  return pa ;
}
/* do not free the data array, only the allocated structure */
void pa_free_structure(parse_array pa) {
  free(pa) ;
}

/* returns non-null int if buffer too small or string not matched, updated the
 * pointer (go past the null-byte) */
int
pa_expect_s(parse_array pa, char *str) {
  uint len = strlen(str) ;
  if (pa->size < pa->ptr - pa->data + len + 1) return -1 ;
  int ret = strcmp(pa->ptr,str) ;
  pa->ptr += len+1 ;
  return ret ;
}
int
pa_expect_text(parse_array pa, char *str) {
  uint len = strlen(str) ;
  if (pa->size < pa->ptr - pa->data + len) return -1 ;
  int ret = strncmp(pa->ptr,str,len) ;
  pa->ptr += len ;
  return ret ;
}

/* Return the size of the string in parse_array (including the '\n' character),
 * copy the string in str and increment pa->ptr if buffer_len permits,
 * returns -1 if there is nothing more to read */
int
pa_read_line(parse_array pa, char* str, int buffer_len) {
  if( (int)pa->size <= pa->ptr - pa->data ) return -1 ;

  int len = 0 ;
  char* cur_ptr = pa->ptr ;

  while( (int)pa->size > cur_ptr - pa->data )
  {
    char c = *(cur_ptr++) ;
    if( c == '\r' ) continue ;
    len++ ;
    if( c == '\n' ) break ;
  }
  
  if( buffer_len < len ) return len ;

  while( (int)pa->size > pa->ptr - pa->data )
  {
    char c = *(pa->ptr++) ;
    if( c == '\r' ) continue ;
    *(str++) = c ;
    if( c == '\n' ) break ;
  }

  return len ;
}
/* Make sure the array is big enough to read the string, and add a null terminator after
 * the string */
int
pa_read_line_clean( parse_array pa, char** p_str, int* p_buffer_len )
{
  while( TRUE )
  {
    int len = pa_read_line( pa, *p_str, *p_buffer_len-1 );
    
    if( len < 0 ) return len ;

    if( *p_buffer_len < len+1 )
    {
      *p_buffer_len = max_i( *p_buffer_len*2, len+1 );
      *p_str = (char*)realloc_or_die( *p_str, *p_buffer_len );
    }
    else
    {
      (*p_str)[len] = '\0' ;
      return len ;
    }
  }
}
