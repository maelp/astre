#include <vision/core.h>
#include <vision/utils/string.h>
#include <vision/utils/datastructures.h>
#include <vision/formats/descfile.h>

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

/******************************************************************************

        desc_file_new

        Create an empty desc_file structure

******************************************************************************/
desc_file
desc_file_new()
{
/*{{{*/
  desc_file df = (desc_file)malloc_or_die( sizeof(struct st_desc_file) );

  df->n_headers = 0 ;
  df->header_captions = (char**)NULL ;
  df->header_contents = (char**)NULL ;
  df->n_lines = 0 ;
  df->n_fields = 0 ;
  df->lines = (double**)NULL ;
  df->tags = (char**)NULL ;

  return df ;
/*}}}*/
}

/******************************************************************************

        desc_file_free_all

        Free a desc_file structure and all its inner fields recursively

******************************************************************************/
void
desc_file_free_all( desc_file* pdf )
{
/*{{{*/
  if( !pdf ) return ;

  desc_file df = *pdf ;
  if( !df ) return ;

  if( df->header_captions )
  {
    for( int k = 0 ; k < df->n_headers ; k++ )
    {
      free( df->header_captions[k] ); df->header_captions[k] = (char*)NULL ;
    }
    free( df->header_captions ); df->header_captions = (char**)NULL ;
  }
  if( df->header_contents )
  {
    for( int k = 0 ; k < df->n_headers ; k++ )
    {
      free( df->header_contents[k] ); df->header_contents[k] = (char*)NULL ;
    }
    free( df->header_contents ); df->header_contents = (char**)NULL ;
  }

  if( df->lines )
    for( int k = 0 ; k < df->n_lines ; k++ )
    {
      free( df->lines[k] ); df->lines[k] = (double*)NULL ;
    }
  free( df->lines ); df->lines = (double**)NULL ;
  if( df->tags )
  {
    for( int k = 0 ; k < df->n_fields ; k++ )
    {
      free( df->tags[k] ); df->tags[k] = (char*)NULL ;
    }
    free( df->tags ); df->tags = (char**)NULL ;
  }

  free( df ); *pdf = (desc_file)NULL ;

  return ;
/*}}}*/
}

/******************************************************************************

        desc_file_load

        Create a desc_file structure from [raw_in]

******************************************************************************/
desc_file
desc_file_load( Rawdata raw_in )
{
/*{{{*/
  desc_file df = desc_file_new();

  char has_seen_DATA = FALSE ;

  parse_array pa = pa_new(0,NULL) ;
  pa->size = raw_in->size ;
  pa->data = raw_in->data ;
  pa->ptr = pa->data ;

  void malformed () { C_log_error("DescFile file malformed!\n"); exit(-1); }

  int buf_size = 1024 ;
  char* buf = (char*)calloc_or_die( buf_size, sizeof(char) );

  int len = -1, n = -1 ;

  c_vector_t* vec_captions = C_vector_start(100);
  c_vector_t* vec_contents = C_vector_start(100);

  inline void add_header( char* header )
  {
    /* is this a header? */
    int n = str_index_of_char( header, '=' );
    if( n <= 0 )
    {
      printf("Header \"%s\" is not \"caption=content\"\n",header);
      malformed();
    }
    else
    {
      char* caption = header ;
      char* content = header+n+1 ;
      *(header+n) = '\0' ;
      caption = trim_whitespace(caption);
      content = trim_whitespace(content);

      C_vector_store(vec_captions, C_string_dup(caption));
      C_vector_store(vec_contents, C_string_dup(content));
    }
  }

  int dsize = 0 ;
  inline void add_data( char* line )
  {
    /* Count fields */
    int n_fields = 0 ;
    char* cur = line ;
    while( *cur != '\0' )
    {
      while( *cur != '\0' && is_whitespace(*cur) ) cur++ ;
      if( *cur == '\0' ) break ;
      n_fields++ ;
      while( *cur != '\0' && !is_whitespace(*cur) ) cur++ ;
    }

    if( n_fields == 0 )
    {
      printf(" Error, line \"%s\" has no fields!\n", line );
      malformed();
    }

    if( df->n_fields == 0 )
    {
      df->n_fields = n_fields ;
      df->tags = (char**)calloc_or_die( n_fields, sizeof(char*) );
      for( int k = 0 ; k < n_fields ; k++ )
        df->tags[k] = (char*)NULL ;
    }
    else
    {
      if( df->n_fields != n_fields )
      {
        printf( "Error, line \"%s\" has %d fields instead of %d!\n",line, n_fields, df->n_fields );
        malformed();
      }
    }

    if( df->n_lines >= dsize )
    {
      dsize = max_i( 2*dsize+1, df->n_lines+1 );
      df->lines = (double**)realloc_or_die( df->lines, dsize*sizeof(double*) );
    }
    df->lines[df->n_lines] = (double*)calloc_or_die( n_fields, sizeof(double) );

    /* Extract fields */
    cur = line ;
    int curfield = 0 ;
    while( *cur != '\0' )
    {
      while( *cur != '\0' && is_whitespace(*cur) ) cur++ ;
      if( *cur == '\0' ) break ;

      char* start = cur ;
      while( *cur != '\0' && !is_whitespace(*cur) ) cur++ ;
      char is_end_of_string = *cur == '\0' ;
      *cur = '\0' ;

      char* tag = (char*)NULL ;

      int p = str_index_of_char( start, ':' );
      if( p >= 0 )
      {
        tag = start ;
        *(start+p) = '\0' ;
        start = start + p + 1 ;
      }

      char* check_cur = start ;
      while( *check_cur != '\0' && (
          (*check_cur >= '0' && *check_cur <= '9') ||
          *check_cur == 'e' || *check_cur == 'E' || *check_cur == '+' || *check_cur == '-'
          || *check_cur == '.' ) ) check_cur++ ;
      if( *check_cur != '\0' )
      {
        printf("Malformed field value \"%s\"!\n", start );
        malformed();
      }

      double f = -1.0 ; int q = sscanf( start, "%lf", &f );

      if( q != 1 )
      {
        printf("Malformed field value \"%s\"!\n", start );
        malformed();
      }

      df->lines[df->n_lines][curfield] = f ;
      if( tag )
      {
        if( df->tags[curfield] == (char*)NULL )
        {
          df->tags[curfield] = strdup(tag);
        }
        else if( strcmp(df->tags[curfield], tag) != 0 )
        {
          printf("Line \"%s\", field %d has tag %s instead of %s!\n", line, curfield, tag, df->tags[curfield] );
          malformed();
        }
      }
      curfield++ ;

      if( is_end_of_string ) break ;
      else cur++ ;
    }
    df->n_lines++ ;
  }

  while( (len = pa_read_line_clean( pa, &buf, &buf_size )) >= 0 )
  {
    char *str = trim_whitespace( buf );
    if( str[0] == '#' || str[0] == '\0' ) continue ; /* Comment or empty line */
    if( strcmp(str,"DATA") == 0 )
    {
      if( has_seen_DATA )
      {
        printf("File has many DATA sections!");
        malformed();
      }
      else
      {
        /* Enter data section */
        has_seen_DATA = TRUE ;
        continue ;
      }
    }

    if( !has_seen_DATA )
    {
      add_header( str );
    }
    else
    {
      add_data( str );
    }
  }

  df->header_captions = C_vector_end(vec_captions, &(df->n_headers));
  df->header_contents = C_vector_end(vec_contents, NULL);

  free( buf );
  pa_free_structure(pa) ;

  return df ;
/*}}}*/
}

/******************************************************************************

        desc_file_save

        Save the desc_file structure in [raw_out]

******************************************************************************/
void
desc_file_save( Rawdata raw_out, desc_file df )
{
/*{{{*/
  char buf[255] ;

  resizable_buf rb = rb_new( 10000*sizeof(int) );

  /* Headers */
  for( int k = 0 ; k < df->n_headers ; k++ )
  {
    rb_pack_text( rb, df->header_captions[k] );
    rb_pack_text( rb, " = " );
    rb_pack_text( rb, df->header_contents[k] );
    rb_pack_text( rb, "\n" );
  }

  rb_pack_text( rb, "DATA\n" );

  /* Data */
  for( int k = 0 ; k < df->n_lines ; k++ )
  {
    for( int p = 0 ; p < df->n_fields ; p++ )
    {
      if( df->tags[p] )
      {
        rb_pack_text( rb, df->tags[p] );
        rb_pack_text( rb, ":" );
      }

      sprintf( buf, "%g ", df->lines[k][p] );
      rb_pack_text( rb, buf );
    }
    rb_pack_text( rb, "\n" );
  }

  raw_out = change_rawdata_or_die( raw_out, rb->size );
  memcpy( raw_out->data, rb->data, rb->size );
  rb_free( rb );
/*}}}*/
}

/******************************************************************************

        desc_file_print

        Print the desc_file structure

******************************************************************************/
void
desc_file_print( desc_file df )
{
/*{{{*/
  /* Headers */
  for( int k = 0 ; k < df->n_headers ; k++ )
  {
    printf( "%s = %s\n", df->header_captions[k], df->header_contents[k] );
  }

  printf( "DATA\n" );

  /* Contents */
  for( int k = 0 ; k < df->n_lines ; k++ )
  {
    for( int p = 0 ; p < df->n_fields ; p++ )
    {
      if( df->tags[p] )
      {
        printf("%s:", df->tags[p] );
      }

      printf( "%g ", df->lines[k][p] );
    }
    printf( "\n" );
  }
/*}}}*/
}

/******************************************************************************

        desc_file_add_header

        Adds an header, does not copy the string caption and content but
        simply reference them

******************************************************************************/
void
desc_file_add_header( desc_file df, char* caption, char* content )
{
/*{{{*/
  df->header_captions = (char**)realloc_or_die( df->header_captions,
      (df->n_headers+1)*sizeof(char*) );
  df->header_contents = (char**)realloc_or_die( df->header_contents,
      (df->n_headers+1)*sizeof(char*) );
  df->header_captions[df->n_headers] = caption ;
  df->header_contents[df->n_headers] = content ;
  df->n_headers++ ;
  return ;
/*}}}*/
}

/******************************************************************************

        desc_file_find_header

        Returns the index of the first header matching [str], or -1 if there
        is no matching header

******************************************************************************/
int
desc_file_find_header( desc_file df, char* str )
{
/*{{{*/
  for( int k = 0 ; k < df->n_headers ; k++ )
  {
    if( strcmp(df->header_captions[k], str) == 0 )
      return k ;
  }
  return -1 ;
/*}}}*/
}

/******************************************************************************

        desc_file_read_header_value

        Return true if the value could be read

******************************************************************************/
char
desc_file_read_header_value( desc_file df, char* caption, char* fmt, void* res )
{
  int p = desc_file_find_header( df, caption );
  if( p < 0 ) return FALSE ;

  int q = sscanf( df->header_contents[p], fmt, res );
  return q != 0 ;
}
