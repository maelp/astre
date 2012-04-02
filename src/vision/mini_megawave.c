#include "vision/mini_megawave.h"

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

void mini_mwerror( int code, int exit_code, char* fmt, ... )
{
  va_list marker;
  char buf[BUFSIZ];
  char* message ;

  va_start( marker, fmt );

  int n = vsnprintf( buf, BUFSIZ, fmt, marker );
  if( n < BUFSIZ )
  {
    message = buf ;
  }
  else
  {
    message = (char*)malloc( n+1 );
    if( message == NULL )
    {
      fprintf( stderr, "ERROR: not enough memory\n" );
      exit(-1);
    }
    vsnprintf( message, BUFSIZ, fmt, marker );
  }

  switch(code)
  {
    case FATAL:
      C_log_error( message );
      if( n >= BUFSIZ ) free( message );
      exit( exit_code );
      break ;
    default:
      C_log_warning( message );
      if( n >= BUFSIZ ) free( message );
      break ;
  }
}

/* Rawdata */
/* ------------------------------------------------- */
/* creates a new rawdata structure */
Rawdata mw_new_rawdata()
{
  Rawdata rd;

  if(!(rd = (Rawdata) (malloc(sizeof(struct rawdata)))))
    {
      mini_mwerror(ERROR, 0, "[mw_new_rawdata] Not enough memory\n");
      return(NULL);
    }

  rd->size = 0;
  rd->data = NULL;
  return (rd);
}

/* allocates the data array */ 
Rawdata mw_alloc_rawdata(rd,newsize)
     Rawdata rd;
     int newsize;
{
  if (rd == NULL)
    {
      mini_mwerror(ERROR, 0, 
	      "[mw_alloc_rawdata] cannot alloc data : rawdata structure is NULL\n");
      return(NULL);
    }
  
  if (rd->data != NULL)
    {
      mini_mwerror(ERROR, 0,
	      "[mw_alloc_rawdata] Attempts to alloc a rawdata which is already allocated\n");
      return(NULL);
    }

  rd->data = (unsigned char *) malloc(newsize);
  if (rd->data == NULL)
    {
      rd->size = 0;
      mini_mwerror(ERROR, 0,"[mw_alloc_rawdata] Not enough memory\n");
      return(NULL);
    }
  rd->size = newsize;  
  return(rd);
}

/* desallocate the data array in the rawdata structure and the 
   structure itself 
*/
void mw_delete_rawdata(rd)
     Rawdata rd;

{
  if (rd == NULL)
    {
      mini_mwerror(ERROR, 0,
	      "[mw_delete_rawdata] cannot delete : rawdata structure is NULL\n");
      return;
    }
  if (rd->data != NULL) free(rd->data);
  rd->data = NULL;
  free(rd);
  rd=NULL;
}


/* Change the size of the allocated data array */
/* May define the struct if not defined */
/* So you have to call it with rd = mw_change_rawdata(rd,...) */
Rawdata mw_change_rawdata(rd, newsize)
     Rawdata rd;
     int newsize;
{
  if (rd == NULL) rd = mw_new_rawdata();
  if (rd == NULL) return(NULL);

  if (newsize != rd->size)
    {
      if (rd->data != NULL) 
	{
	  free(rd->data);  
	  rd->data = NULL;
	}
      if (mw_alloc_rawdata(rd,newsize) == NULL)
	{
	  mw_delete_rawdata(rd);
	  return(NULL);
	}
    }
  else 
    rd->size = newsize;
  return(rd);
}

/* Copy the data of a rawdata into another rawdata */
void mw_copy_rawdata(in, out)
Rawdata in,out;
{
  if ((!in) || (!out) || (!in->data) || (!out->data) 
      || (in->size != out->size))
    {
      mini_mwerror(ERROR, 0,
	      "[mw_copy_rawdata] NULL input or output rd or rawdatas of different sizes\n");
      return;
    }
  memcpy(out->data, in->data, in->size);
}

/* Allocation functions */
/* ------------------------------------------------- */
Rawdata change_rawdata_or_die(Rawdata rd, int newsize) {
  Rawdata t = mw_change_rawdata(rd,newsize) ;
  if (t == NULL) mini_mwerror(FATAL, 1, "Not enough memory !\n");
  return t ;
}
Rawdata alloc_rawdata_or_die(Rawdata rd, int size) {
  Rawdata t = mw_alloc_rawdata(rd,size) ;
  if (t == NULL) mini_mwerror(FATAL, 1, "Not enough memory !\n");
  return t ;
}
Rawdata new_rawdata_or_die() {
  Rawdata t = mw_new_rawdata() ;
  if (t == NULL) mini_mwerror(FATAL, 1, "Not enough memory !\n");
  return t ;
}

/* Loading and saving a Rawdata file */
/* ------------------------------------------------- */
int
save_rawdata( Rawdata rd, char* fname )
{
/*{{{*/
  FILE *fp ;

  if (rd == NULL)
    mini_mwerror(INTERNAL,1,"[save_rawdata] Cannot create file: Rawdata structure is NULL\n");

  if (rd->size <= 0)
    mini_mwerror(INTERNAL,1,"[save_rawdata] Cannot create file: Rawdata structure's size is %d !\n",rd->size);

  fp = fopen(fname, "w");
  if (fp == NULL) return(-1);
  
  if ((int)fwrite(rd->data,1,rd->size,fp) != rd->size)
    {
      mini_mwerror(ERROR, 0,"Error while writing rawdata file \"%s\" !\n",fname);
      fclose(fp);
      return(-1);
    }
  fclose(fp);
  return(0);
/*}}}*/
}

Rawdata
load_rawdata( char* fname )
{
/*{{{*/
  FILE    *fp;
  Rawdata rd;
  struct stat buf;
  int fsize;

  if ( (!(fp = fopen(fname, "r"))) || (fstat(fileno(fp),&buf) != 0) )
    {
      mini_mwerror(ERROR, 0,"File \"%s\" not found or unreadable\n",fname);
      fclose(fp);
      return(NULL);
    }
  /* Size of the file = size of the data, in bytes */
  fsize = buf.st_size; 
  if (!(rd=mw_change_rawdata(NULL,fsize)))
    {
      mini_mwerror(ERROR, 0,"Not enough memory to load rawdata file \"%s\" (%d bytes) !\n",fname,fsize);
      fclose(fp);
      return(NULL);
    }
  if ((int)fread(rd->data,1,fsize,fp) != fsize)
    {
      mini_mwerror(ERROR, 0,"Error while reading rawdata file \"%s\" !\n",fname);
      fclose(fp);
      mw_delete_rawdata(rd);
      return(NULL);
    }

  fclose(fp);
  return(rd);
/*}}}*/
}
