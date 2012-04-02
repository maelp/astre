#ifndef _VISION_TRAJS_TRAJS_H
#define _VISION_TRAJS_TRAJS_H

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

#define PRTYPE_NONE 0 /* Trajectory does not appear in the frame */
#define PRTYPE_REF  1 /* Trajectory corresponds to a point in the frame */
#define PRTYPE_ADD  2 /* Trajectory has been interpolated in the frame */

typedef struct st_t_add_point t_add_point ;
struct st_t_add_point
{
  float x, y ;
} ;
typedef union u_ref_point ref_point ;
union u_ref_point
{
  int r ;
  t_add_point p ;
} ;

typedef struct st_traj traj ;
struct st_traj
{
  int starting_frame ;
  int length ;

  int *type ; /* PRTYPE_NON, PRTYPE_REF, PRTYPE_ADD */
  ref_point *points ;

  void *data ;
};

typedef struct st_trajs_file *trajs_file ;
struct st_trajs_file
{
  int num_of_trajs ;
  traj *trajs ;
};

trajs_file trajs_file_new ();
void trajs_file_free_all (trajs_file *p_tf);

/*******************************************************************************

        Print a trajectory as a trajectory descriptor.

        Often useful for debugging purposes.

*******************************************************************************/

void print_traj( traj* t );

/*******************************************************************************

        Parse a trajectory descriptor into a traj.

        Often useful for debugging purposes.

TODO:
 - write some general string parsing functions
 - write some general trajectory editing function (add hole, etc.)
 - rewrite this function

*******************************************************************************/

static traj*
read_trajectory_descriptor( char* str )
{
/*{{{*/
  traj* tt = (traj*)malloc_or_die( sizeof(traj) );

  void invalid() { mini_mwerror( FATAL, 1, "Invalid trajectory descriptor\n" ); }

  int startingFrame = -1 ;
  int* types = (int*)NULL ;
  ref_point* point_refs = (ref_point*)NULL ;

  int n_allocated_frames = 0 ;

  void grow_arrays()
  {
    n_allocated_frames += 50 ;
    types = (int*)realloc_or_die( types, n_allocated_frames*sizeof(int) );
    point_refs = (ref_point*)realloc_or_die( point_refs, n_allocated_frames*sizeof(ref_point) );
  }
  grow_arrays();

  int cur_traj_idx = 0 ;
  void check_grow()
  {
    if( cur_traj_idx >= n_allocated_frames )
      grow_arrays();
  }

  void add_point( int ref )
  {
    check_grow();
    types[cur_traj_idx] = PRTYPE_REF ;
    point_refs[cur_traj_idx].r = ref ;
    cur_traj_idx++ ;
  }

  void add_hole( int len )
  {
    for( int k = 0 ; k < len ; k++ )
    {
      check_grow();
      types[cur_traj_idx] = PRTYPE_NONE ;
      cur_traj_idx++ ;
    }
  }

  void add_interp( float x, float y )
  {
    check_grow();
    types[cur_traj_idx] = PRTYPE_ADD ;
    point_refs[cur_traj_idx].p.x = x ;
    point_refs[cur_traj_idx].p.y = y ;
    cur_traj_idx++ ;
  }

  int cur_str_idx = 0 ;
  int str_len = strlen( str );

  char track_started = FALSE ;

  void eat_spaces()
  {
    while( TRUE )
    {
      if( cur_str_idx >= str_len ) break ; 

      char c = str[cur_str_idx] ;
      if( c == ' ' || c == '\n' || c == '\r' || c == '\t' )
      {
        cur_str_idx++ ;
      }
      else
      {
        break ;
      }
    }
  }

  int eat_int()
  {
    eat_spaces();
    int res = atoi( &(str[cur_str_idx]) );
    while( TRUE )
    {
      if( cur_str_idx >= str_len ) break ; 

      char c = str[cur_str_idx] ;
      if( (c >= '0' && c <= '9') || c == '+' || c == '-'
          || c == '\n' || c == '\r' || c == '\t' )
      {
        cur_str_idx++ ;
      }
      else
      {
        break ;
      }
    }
    return res ;
  }

  float eat_float()
  {
    eat_spaces();
    float res = atof( &(str[cur_str_idx]) );
    while( TRUE )
    {
      if( cur_str_idx >= str_len ) break ; 

      char c = str[cur_str_idx] ;
      if( (c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E'
          || c == '\n' || c == '\r' || c == '\t' )
      {
        cur_str_idx++ ;
      }
      else
      {
        break ;
      }
    }
    return res ;
  }

  while( cur_str_idx < str_len )
  {
    char c = str[cur_str_idx] ;
    if( c == ' ' || c == '\n' || c == '\r' || c == '\t' )
    {
      cur_str_idx++ ; continue ;
    }

    if( c == ';' )
    {
      break ;
    }

    if( c != 'S' && c != 'P' && c != 'H' && c != 'F' && c != 'C' && c != 'A' ) invalid();
    if( !track_started && c != 'S' ) invalid() ;
    if( track_started && c == 'S' ) invalid() ;

    cur_str_idx++ ;

    switch( c )
    {
      case 'S':
      {
        startingFrame = eat_int() ;
        track_started = TRUE ;
        break ;
      }
      case 'P':
      {
        add_point( eat_int() );
        break ;
      }
      case 'H':
      {
        add_hole( eat_int() );
        break ;
      }
      case 'F':
      {
        int curFrame = startingFrame + cur_traj_idx ;
        int nxtFrame = eat_int() ;
        if( nxtFrame <= curFrame )
        {
          mini_mwerror( FATAL, 1, "[...of_trajectory_str] Trying to advance to a frame <= curFrame+1\n" );
        }
        else
        {
          int hole_len = nxtFrame - curFrame ;
          add_hole( hole_len );
        }
        break ;
      }
      case 'C':
      {
        int curFrame = startingFrame + cur_traj_idx ;
        int f = eat_int() ;
        if( curFrame != f )
        {
          mini_mwerror( FATAL, 1, "[...of_trajectory_str] Assertion curFrame = %d violated (curFrame=%d)!\n", f, curFrame );
        }
        break ;
      }
      case 'A':
      {
        float x = eat_float() ; 
        float y = eat_float() ;

        add_interp( x, y );
      }
    }
  }

  if( track_started )
  {
    int length = cur_traj_idx ;
    tt->starting_frame = startingFrame ;
    tt->length = length ;
    tt->type = types ;
    tt->points = point_refs ;

    return tt ;
  }
  else
    invalid();
/*}}}*/
}

/*******************************************************************************

        Trajectory store variables.  This maintains a resizable buffer
        of trajectories.

*******************************************************************************/

typedef struct st_traj_store {
  int num_trajs ;
  int allocated_trajs ;
  traj *trajs ;
} *traj_store ;

traj_store traj_store_create(int ninit);

/*******************************************************************************

        Add a trajectory to the store.

        Convenient way to add a new trajectory to the list of all
        trajectories.  The array resize itself as the number of
        trajectories increase. The trajectories are validated if ALL_CHECKS
        is defined, and are interpolated to fill their holes.

        WARNING:
          We do not copy the points and type arrays, but simply reference them,
          so they should not be deleted!

*******************************************************************************/

void
add_traj( traj_store store,
          int starting_frame, int length, int *type, union u_ref_point *points, void* data );

#endif
