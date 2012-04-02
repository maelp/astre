#include <vision/trajs/trajs.h>

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

trajs_file
trajs_file_new ()
{
  trajs_file tf = (trajs_file)malloc_or_die( sizeof(struct st_trajs_file) );
  tf->num_of_trajs = 0 ;
  tf->trajs = (traj*) NULL ;

  return tf ;
}

void
trajs_file_free_all (trajs_file *p_tf)
{
  if( !p_tf ) return ;

  trajs_file tf = *p_tf ;
  for( int k = 0 ; k < tf->num_of_trajs ; k++ )
  {
    if( tf->trajs[k].type ) {
      free( tf->trajs[k].type );
      tf->trajs[k].type = (int*)NULL ;
    }
    if( tf->trajs[k].points ) {
      free( tf->trajs[k].points );
      tf->trajs[k].points = (ref_point*)NULL ;
    }
  }
  free( tf->trajs ); tf->trajs = (traj*) NULL ;
  free( tf );

  *p_tf = (trajs_file)NULL ;
  return ;
}

void
print_traj( traj* t )
{
  printf( "S%d ", t->starting_frame );
  for( int p = 0 ; p < t->length ; p++ )
  {
    if( t->type[p] == PRTYPE_REF )
      printf("P%d ", t->points[p].r );
    else if( t->type[p] == PRTYPE_NONE )
      printf("H1 ");
    else
      printf("A ");
  }
  printf("\n");
}

/*******************************************************************************

        Trajectory store variables.  This maintains a resizable buffer
        of trajectories.

*******************************************************************************/

traj_store traj_store_create(int ninit)
{
  C_assert( ninit >= 0 );
  traj_store t = (traj_store)malloc_or_die(sizeof(struct st_traj_store));
  t->num_trajs = 0 ;
  t->allocated_trajs = ninit ;
  t->trajs = (traj*)calloc_or_die(ninit, sizeof(traj));
  return t ;
}

/*******************************************************************************

        Add a trajectory to the store.

        Convenient way to add a new trajectory to the list of all
        trajectories.  The array resize itself as the number of
        trajectories increase.

        WARNING:
          We do not copy the points and type arrays, but simply reference them,
          so they should not be deleted!

*******************************************************************************/

void
add_traj( traj_store store,
          int starting_frame, int length, int *type, union u_ref_point *points, void* data )
{
/*{{{*/
  if( store->allocated_trajs <= store->num_trajs )
  {
    store->allocated_trajs += 50 ;
    store->trajs = (traj*) realloc_or_die( store->trajs, store->allocated_trajs*sizeof(traj) );
  }
  store->trajs[store->num_trajs].starting_frame = starting_frame ;
  store->trajs[store->num_trajs].length = length ;
  store->trajs[store->num_trajs].type = type ;
  store->trajs[store->num_trajs].points = points ;
  store->trajs[store->num_trajs].data = data ;

  store->num_trajs++ ;
/*}}}*/
}
