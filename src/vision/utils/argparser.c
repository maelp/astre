#include <vision/core.h>
#include <vision/utils/argparser.h>
#include <vision/utils/string.h>

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

arg_parser
arg_parser_new()
{
  arg_parser t = (arg_parser)malloc_or_die( sizeof(struct st_arg_parser) );

  t->alloc = 50 ;
  t->nargs = 0 ;
  t->argtable = (void*)calloc_or_die( t->alloc, sizeof(void*) );
  t->info = (char*)NULL ;

  return t ;
}

void
arg_parser_set_info( arg_parser ap, char* info )
{
  if( info )
  {
    info = strdup(info);
    const char* term = "\n\r " ;
    str_rchomp( info, term );
  }
  ap->info = info ;
}

arg_parser
arg_parser_add( arg_parser ap, void* arg )
{
  if( arg == NULL )
    mini_mwerror( FATAL, 1, "[arg_parser_add] Null arg!\n" );

  if( ap->nargs >= ap->alloc )
  {
    ap->alloc += 50 ;
    ap->argtable = (void**)realloc_or_die( ap->argtable, ap->alloc*sizeof(void*) );
  }

  ap->argtable[ap->nargs] = arg ;
  ap->nargs++ ;
}

void
arg_parser_free_all( arg_parser* ap )
{
  arg_freetable( (*ap)->argtable, (*ap)->nargs );
  free( (*ap)->info );
  free( *ap ); *ap = (arg_parser)NULL ;
}

void
arg_parser_handle( arg_parser ap, int ARGC, char** ARGV )
{
  struct arg_lit *help = arg_lit0( NULL, "help", "Print this help and exit" );
  arg_parser_add( ap, help );
  struct arg_end *end = arg_end(20);
  arg_parser_add( ap, end );

  int nerrors = arg_parse( ARGC, ARGV, ap->argtable );

  if( help->count > 0 )
  {
    if( ap->info ) fprintf( stderr, "%s\n\n", ap->info );
    fprintf( stderr, "Usage: %s", ARGV[0] );
    arg_print_syntax( stderr, ap->argtable, "\n" );
    arg_print_glossary( stderr, ap->argtable, " %-25s %s\n" );
    exit( 0 );
  }

  if( nerrors > 0 )
  {
    if( ARGC > 1 )
    {
      C_log_info( "Try '%s --help' for more information.\n", ARGV[0] );
      arg_print_errors( stderr, end, NULL );
    }
    else
    {
      if( ap->info ) fprintf( stderr, "%s\n\n", ap->info );
      fprintf( stderr, "Usage: %s", ARGV[0] );
      arg_print_syntax( stderr, ap->argtable, "\n" );
      arg_print_glossary( stderr, ap->argtable, " %-25s %s\n" );
    }
    exit( -1 );
  }
}
