#ifndef _VISION_UTILS_ARGPARSER_H
#define _VISION_UTILS_ARGPARSER_H

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

#include <argtable2.h>

typedef struct st_arg_parser
{
  int alloc ;
  int nargs ;
  void** argtable ;
  char* info ;
}*arg_parser ;

/* Create a new structure */
arg_parser arg_parser_new();
/* Free the structure */
void arg_parser_free_all( arg_parser* ap );
/* Set general help informations */
void arg_parser_set_info( arg_parser ap, char* info );
/* Add an argument */
arg_parser arg_parser_add( arg_parser ap, void* arg );
/* Parse the arguments */
void arg_parser_handle( arg_parser ap, int ARGC, char** ARGV );

/*
    Example usage

int main( int ARGC, char** ARGV )
{

  arg_parser ap = arg_parser_new();

  int K ;
  struct arg_int *k = arg_int0( "k", "maxnum", "<k>",
      "Maximal number to add to linked list (default 10)" );
  if( k ) k->ival[0] = 10 ;
  arg_parser_add( ap, k );

  arg_parser_handle( ap, ARGC, ARGV );

  printf( "k = %d\n", k->ival[0] );

  arg_parser_free_all( &ap );

  return 0 ;
}
*/

#endif
