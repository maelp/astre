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
#include <vision/utils/argparser.h>
#include <vision/trajs/pointsdesc.h>
#include <vision/trajs/trajs.h>
#define P printf

static int K ;                                  /* number of frames */

static points_desc r_pd, f_pd ;
static trajs_file  r_tf, f_tf ;

/* The criteria are as follows :
 *  - RL : #real links, CL : #correct found links, FL : #found links
 */

/* Utility functions */
static int
count_trajectory_links( traj* t )
{
  int nlinks = 0 ;

  for( int p = 0 ; p < t->length-1 ; p++ )
  {
    if( t->type[p] == PRTYPE_REF )
    {
      while( p+1 < t->length && t->type[p+1] != PRTYPE_REF ) p++ ;
      if( p+1 < t->length ) nlinks++ ;
    }
  }

  return nlinks ;
}

/*                                              MAIN FUNCTION */
/* ---------------------------------------------------------- */
/* tstats
 *
 * rd1   : Rawdata with input Pointsdesc file
 * rd2   : Rawdata with secondary input Pointsdesc file
 *         if rd2 is NULL, rd2 = rd1
 * rt_col: 1-based index of the real trajectory column in rd1
 * ft_col: 1-based index of the found trajectory column in rd2
 */
static void
tstats
(
    Rawdata rd1,
    Rawdata rd2,
    int rt_col,
    int ft_col
)
{
  /* Load points and trajectories */
  r_pd = points_desc_load( rd1 );
  if( rd2 )
  {
    f_pd = points_desc_load( rd2 );
    if( r_pd->uid != f_pd->uid )
      mini_mwerror( FATAL, 1, "Pointsdesc file UID do not match!\n" );
  }
  else
    f_pd = r_pd ;

  /* original column number = n_fields+1 since we do not take frame numbers
   * into account */
  if( rt_col < 0 ) rt_col += r_pd->n_fields+1 ;
  if( ft_col < 0 ) ft_col += f_pd->n_fields+1 ;

  if( rt_col < 3 || rt_col >= r_pd->n_fields+1 )
    mini_mwerror( FATAL, 1, "Index rt invalid!" );
  if( ft_col < 3 || ft_col >= f_pd->n_fields+1 )
    mini_mwerror( FATAL, 1, "Index ft invalid!" );

  rt_col -= 1;
  ft_col -= 1;

  /* Extract trajectories in a more convenient form */
  r_tf = points_desc_extract_trajs( r_pd, rt_col, TRUE ); /* Relabel point tags */
  f_tf = points_desc_extract_trajs( f_pd, ft_col, TRUE ); /* Relabel point tags */

  /* Set K */
  K = r_pd->n_frames ;

  int num_found_trajs = f_tf->num_of_trajs ;
  traj* found_trajs = f_tf->trajs ;

  int num_real_trajs = r_tf->num_of_trajs ;
  traj* real_trajs = r_tf->trajs ;

  /**********************************************************
   *                   COMPUTE CRITERIA                     *
   **********************************************************/

  float RL = 0, FL = 0, CL = 0 ; /* Real, found, correct links */

  /* Compute RL, FL */
  /*{{{*/
  for( int k = 0 ; k < num_real_trajs ; k++ )
  {
    traj *t = &(real_trajs[k]) ;
    RL += (float)count_trajectory_links( t );
  }
  for( int k = 0 ; k < num_found_trajs ; k++ )
  {
    traj *t = &(found_trajs[k]) ;
    FL += (float)count_trajectory_links( t );
  }
  /*}}}*/

  /* Compute CL */
  /*{{{*/
  for( int k = 0 ; k < num_found_trajs ; k++ )
  {
    traj *ft = &(found_trajs[k]) ;

    for( int i = 0 ; i < ft->length-1 ; i++ )
    {
      if( ft->type[i] == PRTYPE_REF )
      {
        int s = ft->starting_frame + i ; /* Start of the link */
        int rs = ft->points[i].r ;
        /* Find the length of the link */
        while( i+1 < ft->length && ft->type[i+1] != PRTYPE_REF ) i++ ;
        if( i+1 >= ft->length ) break ; /* No link, only NONE or ADD points */
        int e = ft->starting_frame + i + 1 ; /* End of the link */
        int re = ft->points[i+1].r ;

        int real_traj_idx = (int)r_pd->points[s][rs*r_pd->n_fields+rt_col] ;
        if( real_traj_idx < 0 || real_traj_idx != (int)r_pd->points[e][re*r_pd->n_fields+rt_col] )
          continue ; /* Not possibly a correct link */

        traj *rt = &(real_trajs[real_traj_idx]);
        char matching_link = TRUE ;
        /* Check that the real trajectory has no other point between s and e */
        for( int j = s-rt->starting_frame+1 ; j < e-rt->starting_frame ; j++ )
        {
          if( rt->type[j] == PRTYPE_REF )
          {
            matching_link = FALSE ;
            break ;
          }
        }

        if( matching_link )
        {
          CL++ ;
        }
      }
    }
  }
  /*}}}*/

  /**********************************************************
   *                    OUTPUT CRITERIA                     *
   **********************************************************/

  float EL = 0 ; if( RL > 0 ) EL = CL/RL ;
  float EL2 = 0 ; if( FL > 0 ) EL2 = CL/FL ;

  P( "[MD] {" ); /* Metadata */

  if( RL > 0 ) P( "'recall': %g, ", EL );
  else P( "'recall': None, " );

  if( FL > 0 ) P( "'precision': %g, ", EL2 );
  else P( "'precision': None, " );

  P( "'num_detected_trajs': %d, ", num_found_trajs );

  P( "}\n" );
}

int main( int ARGC, char** ARGV )
{
  arg_parser ap = arg_parser_new();

  arg_parser_set_info( ap,
"Statistics on trajectory detection results.\n\n"
" - When the real and found trajectories are in the same file, use:\n"
"    tstats pf -r 3 -f 4  # Real trajs on column 3, found trajs on column 4\n\n"
" - When the real and found trajectories are in different files, use:\n"
"    tstats pf_real pf_found -r 3 -f 3 # Trajs on column 3, in each file\n");

  struct arg_str *p_file1 = arg_str1( NULL, NULL, "pf",
      "Pointdesc file" );
  arg_parser_add( ap, p_file1 );

  struct arg_str *p_file2 = arg_str0( NULL, NULL, "[pf2]",
      "Pointdesc file" );
  arg_parser_add( ap, p_file2 );

  struct arg_int *p_rt = arg_int0( "r", "real-traj", "<r>", "Column index of real trajs (indexing starts at 0, default 3)" );
  if( p_rt ) p_rt->ival[0] = 3 ;
  arg_parser_add( ap, p_rt );

  struct arg_int *p_ft = arg_int0( "f", "found-traj", "<f>", "Column index of found trajs (indexing starts at 0, default -1)" );
  if( p_ft ) p_ft->ival[0] = -1 ;
  arg_parser_add( ap, p_ft );

  /* Handle arguments */
  arg_parser_handle( ap, ARGC, ARGV );

  /* Check arguments */
  char* file1 = (char*)p_file1->sval[0]; C_assert( file1 && strlen(file1) > 0 );
  char* file2 = (char*)NULL ;
  if( p_file2->count > 0 )
  {
    file2 = (char*)p_file2->sval[0]; C_assert( file2 && strlen(file2) > 0 );
  }
  int rt = p_rt->ival[0];
  int ft = p_ft->ival[0];

  Rawdata rd1 = load_rawdata( file1 );

  Rawdata rd2 = (Rawdata)NULL ;
  if( file2 )
  {
    rd2 = load_rawdata( file2 );
  }

  tstats( rd1, rd2, rt, ft );

  mw_delete_rawdata( rd1 );
  if( rd2 ) mw_delete_rawdata( rd2 );

  /* Clean memory */
  arg_parser_free_all( &ap );
}
