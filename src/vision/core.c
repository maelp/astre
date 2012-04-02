#include "vision/core.h"

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

void
init_rgen ()
{
  uint u = (long int)time(NULL)+(long int)getpid() ;
  srandom(u) ;
  srand48(u) ;
}

/* TODO: recode this */
char
file_is_readable(char *filename)
{
  FILE *fp = fopen(filename,"r") ;
  if (!fp) return FALSE ;
  else {
    fclose(fp) ;
    return TRUE ;
  }
}

/* TODO: recode this */
char
file_is_writeable(char *filename)
{
  FILE *fp = fopen(filename,"w") ;
  if (!fp) return FALSE ;
  else {
    fclose(fp) ;
    return TRUE ;
  }
}

