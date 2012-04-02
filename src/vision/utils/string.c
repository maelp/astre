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

#include <string.h>

/* Trim whitespace */
char
is_whitespace( char c )
{
  return( c == '\n' || c == '\r' || c == '\t' || c == ' ' );
}
char*
trim_whitespace( char* str )
{
  char* start = str ;
  while( *start != '\0' && is_whitespace(*start) ) start++ ;
  char* end = start ;
  while( *end != '\0' ) end++ ;
  end-- ;
  while( end >= start && is_whitespace(*end) ) end-- ;
  *(end+1) = '\0' ;
  return start ;
}
/* locate a character */
int
str_index_of_char( char* str, char c )
{
  char* pos = str ;
  while( *pos != '\0' && *pos != c ) pos++ ;
  if( *pos == '\0' ) return -1 ;
  else return pos-str ;
}

void
str_rchomp( char* str, const char* trim )
{
  if( str && trim )
  {
    int len = strlen(str);
    for( char* p = str+len-1 ; len-- ; p-- )
    {
      if( strchr(trim, *p) != NULL )
      {
        *p = '\0' ;
      }
      else
      {
        break ;
      }
    }
  }
}
