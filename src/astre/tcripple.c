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

/* MAIN FUNCTION */
/* ------------------------------------------------------ */
/* tcripple
 *
 * i_pd : input Rawdata containing the source Pointsdesc file
 * o_pd : output Rawdata
 * i_r  : probability of removing a point
 * t_col: 1-based index of trajectory column in the source Pointsdesc file
 */
void
tcripple
(
    Rawdata i_pd,
    Rawdata o_pd,
    int i_r,
    int t_col
)
{
  points_desc pd = points_desc_load ( i_pd ) ;
  points_desc pd_crippled = points_desc_new();

  if( i_r < 0 || i_r > 100 )
    mini_mwerror( FATAL, 1, "[tcripple] r should be 0 <= r <= 100!\n" );
  float alpha = (float)i_r / 100.0f ;

  int n_cols = pd->n_fields+1; /* we removed the frame number */

  if( t_col < 0 ) t_col += n_cols ;

  if( t_col < 3 || t_col >= n_cols )
    mini_mwerror( FATAL, 1, "[tcripple] invalid column index %d!\n", t_col );

  t_col -= 1 ;

  trajs_file tf = points_desc_extract_trajs( pd, t_col, TRUE );

  int K = pd->n_frames ;

  /* Copy the infos */
  points_desc_copy_infos( pd, pd_crippled );
  points_desc_set_uid( pd_crippled );

  pd_crippled->n_frames = K ;
  pd_crippled->points = (double**)calloc_or_die( K, sizeof(double*) );
  pd_crippled->n_points_in_frame = (int*)calloc_or_die( K, sizeof(int) );
  for( int k = 0 ; k < K ; k++ )
  {
    pd_crippled->n_points_in_frame[k] = 0 ;
    pd_crippled->points[k] = (double*)calloc_or_die( pd->n_points_in_frame[k]*pd->n_fields, sizeof(double) );
  }

  init_rgen();

  for( int k = 0 ; k < K ; k++ )
  {
    for( int p = 0 ; p < pd->n_points_in_frame[k] ; p++ )
    {
      if( i_r > 0 )
      {
        int t_idx = (int)pd->points[k][p*pd->n_fields+t_col];
        if( t_idx >= 0 )
        {
          traj* tt = &(tf->trajs[t_idx]);
          int p = k-tt->starting_frame ;
          if( p >= 2 && p <= tt->length-3 )
          {
            float r = (float)drand48() ;
            if( r <= alpha )
            {
              /* Remove the trajectory point */
              continue ;
            }
          }
        }
      }

      /* Keep the trajectory point */
      double *pf = &(pd->points[k][p*pd->n_fields+0]);
      double *pt = &(pd_crippled->points[k][pd_crippled->n_points_in_frame[k]*pd_crippled->n_fields+0]);
      for( int u = 0 ; u < pd->n_fields ; u++ )
      {
        *(pt++) = *(pf++) ;
      }
      for( int u = pd->n_fields ; u < pd_crippled->n_fields ; u++ )
      {
        *(pt++) = 0.0 ;
      }
      pd_crippled->n_points_in_frame[k]++ ;
    }
  }

  /*                                  Save the trajectories */
  /* ------------------------------------------------------ */
  points_desc_save( o_pd, pd_crippled );

  /*                                            Free memory */
  /* ------------------------------------------------------ */
  points_desc_free_all( &pd );
  points_desc_free_all( &pd_crippled );
  trajs_file_free_all( &tf );
}

int main( int ARGC, char** ARGV )
{
  arg_parser ap = arg_parser_new();

  arg_parser_set_info( ap,
"Cripple a points file by removing points.\n\n"
"The points in the first two and last two frames of a trajectory are kept.\n"
"Only trajectory points are affected.\n" );

  struct arg_str *p_in = arg_str1( NULL, NULL, "in",
      "Input points file" );
  arg_parser_add( ap, p_in );

  struct arg_str *p_out = arg_str1( NULL, NULL, "out",
      "Output points file" );
  arg_parser_add( ap, p_out );

  struct arg_int *p_r = arg_int0( "r", NULL, "<r>", "Probability that a trajectory point is removed (default: 20)" );
  if( p_r ) p_r->ival[0] = 20 ;
  arg_parser_add( ap, p_r );

  struct arg_int *p_t = arg_int0( "t", "traj-col", "<t>", "Index of trajectory column (0-based, default: -1)" );
  if( p_t ) p_t->ival[0] = -1 ;
  arg_parser_add( ap, p_t );

  /* Handle arguments */
  arg_parser_handle( ap, ARGC, ARGV );

  /* Check arguments */
  char* in = (char*)p_in->sval[0]; C_assert( in && strlen(in) > 0 );
  char* out = (char*)p_out->sval[0]; C_assert( out && strlen(out) > 0 );
  int r = p_r->ival[0];
  int t = p_t->ival[0];

  Rawdata rd_in = load_rawdata( in );
  Rawdata rd_out = new_rawdata_or_die();

  tcripple( rd_in, rd_out, r, t );

  save_rawdata( rd_out, out );

  mw_delete_rawdata( rd_in );
  mw_delete_rawdata( rd_out );

  /* Clean memory */
  arg_parser_free_all( &ap );
}
