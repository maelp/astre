#include <vision/formats/descfile.h>
#include <vision/trajs/pointsdesc.h>

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

points_desc
points_desc_new()
{
/*{{{*/
  points_desc pd = (points_desc)malloc_or_die( sizeof(struct st_points_desc) );

  pd->width = -1 ;
  pd->height = -1 ;
  pd->uid = -1 ;

  pd->orig_first_frame = -1 ;

  pd->n_headers = 0 ;
  pd->header_captions = (char**)NULL ;
  pd->header_contents = (char**)NULL ;

  pd->n_frames = 0 ;
  pd->n_points_in_frame = (int*)NULL ;
  pd->n_fields = 0 ;
  pd->tags = (char**)NULL ;
  pd->points = (double**)NULL ;

  return pd ;
/*}}}*/
}

void
points_desc_free_all( points_desc* ppd )
{
/*{{{*/
  if( ppd == NULL ) return ;

  points_desc pd = *ppd ;
  if( pd == NULL ) return ;

  if( pd->header_captions )
  {
    for( int k = 0 ; k < pd->n_headers ; k++ )
    {
      free( pd->header_captions[k] ); pd->header_captions[k] = (char*)NULL ;
    }
    free( pd->header_captions ); pd->header_captions = (char**)NULL ;
  }
  if( pd->header_contents )
  {
    for( int k = 0 ; k < pd->n_headers ; k++ )
    {
      free( pd->header_contents[k] ); pd->header_contents[k] = (char*)NULL ;
    }
    free( pd->header_contents ); pd->header_contents = (char**)NULL ;
  }

  free( pd->n_points_in_frame ); pd->n_points_in_frame = (int*)NULL ;
  if( pd->points )
  {
    for( int k = 0 ; k < pd->n_frames ; k++ )
    {
      free( pd->points[k] ); pd->points[k] = (double*)NULL ;
    }
    free( pd->points ); pd->points = (double**)NULL ;
  }
  /* Some tags can be NULL so we must rely on pd->n_fields to free
   * the whole array */
  if( pd->tags )
  {
    for( int k = 0 ; k < pd->n_fields ; k++ )
    {
      free( pd->tags[k] ); pd->tags[k] = (char*)NULL ;
    }
    free( pd->tags ); pd->tags = (char**)NULL ;
  }
  if( pd->points )
  {
    for( int k = 0 ; k < pd->n_fields ; k++ )
    {
      free( pd->points[k] ); pd->points[k] = (double*)NULL ;
    }
    free( pd->points ); pd->points = (double**)NULL ;
  }

  free( pd );
  *ppd = (points_desc)NULL ;

  return ;
/*}}}*/
}

/* Set the uid */
void
points_desc_set_uid( points_desc pd )
{
  pd->uid = (int)time(NULL) + (int)getpid();
}

/* Load and add an additional column */
points_desc
points_desc_load_ext( Rawdata raw_in, int n_additional_fields )
{
/*{{{*/
  /* load rawdata descriptor */
  desc_file df = desc_file_load( raw_in );
  points_desc pd = points_desc_new();

  void malformed () { C_log_error("PointsDescFile file malformed!\n"); exit(-1); }

  int p = desc_file_find_header( df, "type" );
  if( p < 0 ) malformed();
  if( strcmp(df->header_contents[p], "PointsFile v.1.0") != 0 )
  {
    printf( "Wrong PointsFile version: %s\n", df->header_contents[p] );
    malformed();
  }

  if( !desc_file_read_header_value( df, "width", "%d", &(pd->width) ) )
  {
    printf( "Couldn't read width!\n" ); malformed();
  }
  if( !desc_file_read_header_value( df, "height", "%d", &(pd->height) ) )
  {
    printf( "Couldn't read height!\n" ); malformed();
  }
  if( !desc_file_read_header_value( df, "uid", "%d", &(pd->uid) ) )
  {
    printf( "Couldn't read uid!\n" ); malformed();
  }

  if( df->n_fields < 3 )
  {
    printf( "Not enough fields in data lines (need at least frame, x, y)!\n" );
    malformed();
  }

  /* Move headers */
  pd->n_headers = df->n_headers ;
  pd->header_captions = df->header_captions ; df->header_captions = (char**)NULL ;
  pd->header_contents = df->header_contents ; df->header_contents = (char**)NULL ;

  inline char
  double_is_integer( double f )
  {
    double diff = f - (int)f ;
    return diff <= 1E15 ;
  }

  /* Move tags, first field is the frame number */
  pd->n_fields = df->n_fields - 1 + n_additional_fields ;
  pd->tags = (char**)calloc_or_die( pd->n_fields, sizeof(char*) );
  for( int k = 0 ; k < pd->n_fields ; k++ )
    pd->tags[k] = (char*)NULL ;
  for( int k = 1 ; k < df->n_fields ; k++ )
  {
    pd->tags[k-1] = df->tags[k] ; df->tags[k] = (char*)NULL ;
  }

  /* Find number of frames and original first frame */
  char original_initialized = FALSE ;
  pd->orig_first_frame = -1 ;
  int max_frame = - 1 ;
  for( int k = 0 ; k < df->n_lines ; k++ )
  {
    if( !double_is_integer( df->lines[k][0] ) )
    {
      printf( "Error on line %d, frame number %f is not an integer!\n", k, df->lines[k][0] );
      malformed();
    }

    int f = (int)df->lines[k][0] ;
    if( !original_initialized )
    {
      pd->orig_first_frame = f ;
      max_frame = f ;
      original_initialized = TRUE ;
    }

    if( f < pd->orig_first_frame )
      pd->orig_first_frame = f ;
    if( f > max_frame )
      max_frame = f ;
  }
  pd->n_frames = max_frame - pd->orig_first_frame + 1 ;

  /* Get the number of points in frame */
  pd->n_points_in_frame = (int*)calloc_or_die( pd->n_frames, sizeof(int) );
  for( int k = 0 ; k < pd->n_frames ; k++ )
  {
    pd->n_points_in_frame[k] = 0 ;
  }
  for( int k = 0 ; k < df->n_lines ; k++ )
  {
    int f = (int)df->lines[k][0] ;
    pd->n_points_in_frame[f-pd->orig_first_frame]++ ;
  }

  /* Copy fields */
  pd->points = (double**)calloc_or_die( pd->n_frames, sizeof(double*) );
  for( int k = 0 ; k < pd->n_frames ; k++ )
  {
    pd->points[k] = (double*)calloc_or_die( pd->n_points_in_frame[k]*pd->n_fields,
        sizeof(double) );
    for( int p = 0 ; p < pd->n_points_in_frame[k]*pd->n_fields ; p++ )
    {
      pd->points[k][p] = 0.0f ;
    }
  }
  /* Reinit number of points in frames, to help us store the points */
  for( int k = 0 ; k < pd->n_frames ; k++ )
  {
    pd->n_points_in_frame[k] = 0 ;
  }
  for( int k = 0 ; k < df->n_lines ; k++ )
  {
    int f = (int)df->lines[k][0] - pd->orig_first_frame ;
    /* First field is the frame number */
    for( int p = 1 ; p < df->n_fields ; p++ )
    {
      pd->points[f][pd->n_points_in_frame[f]*pd->n_fields + p-1] = df->lines[k][p] ;
    }
    pd->n_points_in_frame[f]++ ;
  }

  desc_file_free_all( &df );

  return pd ;
/*}}}*/
}

/* Load a pointsdesc file from a Rawdata structure */
points_desc
points_desc_load( Rawdata raw_in )
{
  return points_desc_load_ext( raw_in, 0 );
}

void
points_desc_save( Rawdata raw_out, points_desc pd )
{
/*{{{*/
  desc_file df = desc_file_new();

  /* Copy headers */
  df->n_headers = pd->n_headers ;
  df->header_captions = (char**)calloc_or_die(df->n_headers, sizeof(char**));
  df->header_contents = (char**)calloc_or_die(df->n_headers, sizeof(char**));
  for( int k = 0 ; k < df->n_headers ; k++ )
  {
    df->header_captions[k] = C_string_dup(pd->header_captions[k]);
    df->header_contents[k] = C_string_dup(pd->header_contents[k]);
  }

  /* modify/add the type, uid, width, height headers */
  /* We should not make copies of the strings when adding/modifying headers
   * since we won't free them afterwards */
  inline void
  modify_or_add_header( desc_file df, char* caption, char* content )
  {
    int p = desc_file_find_header( df, caption );
    if( p < 0 )
    {
      desc_file_add_header( df, C_string_dup(caption), C_string_dup(content) );
    }
    else
    {
      free( df->header_contents[p] ); df->header_contents[p] = (char*)NULL ;
      df->header_contents[p] = C_string_dup(content) ;
    }
  }

  char* buf_type = "PointsFile v.1.0" ;
  modify_or_add_header( df, "type", buf_type );
  
  char buf_uid[255] ; sprintf( buf_uid, "%d", pd->uid );
  modify_or_add_header( df, "uid", buf_uid );
  
  char buf_width[255] ; sprintf( buf_width, "%d", pd->width );
  modify_or_add_header( df, "width", buf_width );

  char buf_height[255] ; sprintf( buf_height, "%d", pd->height );
  modify_or_add_header( df, "height", buf_height );

  /* Copy tags */
  df->n_fields = pd->n_fields + 1 ; /* add the frame number */
  df->tags = (char**)calloc_or_die( df->n_fields, sizeof(char*) );
  df->tags[0] = C_string_dup("f");
  for( int k = 0 ; k < pd->n_fields ; k++ )
  {
    df->tags[k+1] = C_string_dup(pd->tags[k]);
  }
  
  /* Compute number of lines */
  df->n_lines = 0 ;
  for( int k = 0 ; k < pd->n_frames ; k++ )
  {
    df->n_lines += pd->n_points_in_frame[k] ;
  }

  /* Copy data */
  df->lines = (double**)calloc_or_die( df->n_lines, sizeof(double*) );
  for( int k = 0 ; k < df->n_lines ; k++ )
  {
    df->lines[k] = (double*)calloc_or_die( df->n_fields, sizeof(double) );
  }

  int cur_line = 0 ;
  for( int k = 0 ; k < pd->n_frames ; k++ )
  {
    for( int p = 0 ; p < pd->n_points_in_frame[k] ; p++ )
    {
      double* fields = &(pd->points[k][p*pd->n_fields]) ;
      df->lines[cur_line][0] = (double)(k+pd->orig_first_frame) ;
      for( int q = 0 ; q < pd->n_fields ; q++ )
        df->lines[cur_line][q+1] = fields[q] ;
      cur_line++ ;
    }
  }

  desc_file_save( raw_out, df );

  /* Free the desc_file */
  desc_file_free_all( &df );

  return ;
/*}}}*/
}

points_desc
points_desc_copy( points_desc pd, int n_new_fields )
{
/*{{{*/
  C_assert( n_new_fields >= 0 );

  points_desc pn = points_desc_new();

  pn->width = pd->width ;
  pn->height = pd->height ;
  pn->uid = pd->uid ;
  pn->orig_first_frame = pd->orig_first_frame ;
  pn->n_headers = pd->n_headers ;

  pn->header_captions = (char**)calloc_or_die( pn->n_headers, sizeof(char*) );
  pn->header_contents = (char**)calloc_or_die( pn->n_headers, sizeof(char*) );
  for( int k = 0 ; k < pn->n_headers ; k++ )
  {
    pn->header_captions[k] = C_string_dup( pd->header_captions[k] );
    pn->header_contents[k] = C_string_dup( pd->header_contents[k] );
  }

  pn->n_frames = pd->n_frames ;
  pn->n_points_in_frame = (int*)calloc_or_die( pn->n_frames, sizeof(int) );
  for( int k = 0 ; k < pn->n_frames ; k++ )
  {
    pn->n_points_in_frame[k] = pd->n_points_in_frame[k] ;
  }

  pn->n_fields = pd->n_fields + n_new_fields ;
  pn->tags = (char**)calloc_or_die( pn->n_fields, sizeof(char*) );
  for( int k = 0 ; k < pd->n_fields ; k++ )
  {
    pn->tags[k] = C_string_dup( pd->tags[k] );
  }
  for( int k = pd->n_fields ; k < pn->n_fields ; k++ )
  {
    pn->tags[k] = NULL ;
  }
  pn->points = (double**)calloc_or_die( pn->n_frames, sizeof(double*) );
  for( int k = 0 ; k < pn->n_frames ; k++ )
  {
    pn->points[k] = (double*)calloc_or_die( pn->n_points_in_frame[k]*pn->n_fields, sizeof(double) );
    double* cur_in_pos = pd->points[k] ;
    double* cur_out_pos = pn->points[k] ;
    for( int j = 0 ; j < pn->n_points_in_frame[k] ; j++ )
    {
      for( int p = 0 ; p < pd->n_fields ; p++ )
      {
        *(cur_out_pos++) = *(cur_in_pos++) ;
      }
      for( int p = pd->n_fields ; p < pn->n_fields ; p++ )
      {
        *(cur_out_pos++) = 0.0 ;
      }
    }
  }

  return pn ;
/*}}}*/
}

/* Set a field value for all points */
void
points_desc_set_field( points_desc pd, int n_field, double v )
{
/*{{{*/
  if( n_field < 0 ) n_field += pd->n_fields ;
  C_assert( n_field >= 0 && n_field < pd->n_fields );

  for( int k = 0 ; k < pd->n_frames ; k++ )
  {
    double *cur = &(pd->points[k][n_field]);
    for( int p = 0 ; p < pd->n_points_in_frame[k] ; p++ )
    {
      *cur = v ;
      cur += pd->n_fields ;
    }
  }
/*}}}*/
}

/* Tag the points in pd with corresponding index of trajectory in field n_field */
void
points_desc_set_traj_tags( points_desc pd, trajs_file tf, int n_field )
{
/*{{{*/
  if( n_field < 0 ) n_field += pd->n_fields ;
  C_assert( n_field >= 2 && n_field < pd->n_fields ); /* Two first fields are x, y */

  points_desc_set_field( pd, n_field, -1.0 );
  
  for( int k = 0 ; k < tf->num_of_trajs ; k++ )
  {
    traj* tt = &(tf->trajs[k]);
    for( int p = 0, f = tt->starting_frame ; p < tt->length ; p++, f++ )
    {
      if( tt->type[p] == PRTYPE_REF )
      {
        int r = tt->points[p].r ;
        pd->points[f][r*pd->n_fields + n_field] = (double)k ;
      }
    }
  }
/*}}}*/
}

/* Convenience function that add a trajectory field and save trajectories */
void
points_desc_save_with_new_trajs( Rawdata raw_out, points_desc pd, trajs_file tf )
{
/*{{{*/
  points_desc pn = points_desc_copy( pd, 1 );

  int n_field = pn->n_fields-1 ;
  pn->tags[n_field] = C_string_dup("t");

  /* Tag trajectories */
  points_desc_set_traj_tags( pn, tf, n_field );

  /* Save points_desc */
  points_desc_save( raw_out, pn );

  /* Free memory */
  points_desc_free_all( &pn );
/*}}}*/
}

/* Extract tags in field <n_field> and construct trajectories, trajectories
 * indices in trajs_file do not necessarily correspond to point indices,
 * however if relabel_trajs is set, the point indices will be relabeled so that
 * it is the case */
trajs_file
points_desc_extract_trajs( points_desc pd, int n_field, char relabel_trajs )
{
/*{{{*/
  if( n_field < 0 ) n_field += pd->n_fields ;
  C_assert( n_field >= 2 && n_field < pd->n_fields );

  trajs_file tf = trajs_file_new();

  int max_traj = -1 ;
  for( int k = 0 ; k < pd->n_frames ; k++ )
  {
    for( int p = 0 ; p < pd->n_points_in_frame[k] ; p++ )
    {
      double dt = pd->points[k][p*pd->n_fields + n_field] ;
      if( dt - (int)dt >= 1E-10 )
      {
        mini_mwerror( FATAL, 1, "Points %d in frame %d has non-integer trajectory tag (%g)!\n",
            p, k, dt );
      }
      int it = (int)dt ;
      max_traj = max_i( max_traj, it );
    }
  }

  /* TODO: implement a more efficient code, without allocating max_traj trajectories */
  if( max_traj >= 0 )
  {
    traj *trajs = (traj*)calloc_or_die( max_traj+1, sizeof(traj) );
    for( int k = 0 ; k <= max_traj ; k++ )
    {
      trajs[k].length = 0 ;
      trajs[k].type = (int*)NULL ;
      trajs[k].points = (ref_point*)NULL ;
    }

    /* Compute trajectories lengths */
    for( int k = 0 ; k < pd->n_frames ; k++ )
    {
      for( int p = 0 ; p < pd->n_points_in_frame[k] ; p++ )
      {
        int it = (int)pd->points[k][p*pd->n_fields + n_field] ;
        if( it < 0 ) continue ;

        if( trajs[it].length == 0 )
        {
          trajs[it].starting_frame = k ;
          trajs[it].length = 1 ;
        }
        else
        {
          trajs[it].length = k - trajs[it].starting_frame + 1 ;
        }
      }
    }

    /* Allocate points arrays */
    int num_trajs = 0 ;
    for( int k = 0 ; k <= max_traj ; k++ )
    {
      if( trajs[k].length > 0 )
      {
        num_trajs++ ;
        trajs[k].type = (int*)calloc_or_die( trajs[k].length, sizeof(int) );
        trajs[k].points = (ref_point*)calloc_or_die( trajs[k].length, sizeof(ref_point) );
        for( int p = 0 ; p < trajs[k].length ; p++ )
        {
          trajs[k].type[p] = PRTYPE_NONE ;
        }
      }
    }

    /* Fill points array */
    for( int k = 0 ; k < pd->n_frames ; k++ )
    {
      for( int p = 0 ; p < pd->n_points_in_frame[k] ; p++ )
      {
        int it = (int)pd->points[k][p*pd->n_fields + n_field] ;
        if( it < 0 ) continue ;

        if( trajs[it].type[k - trajs[it].starting_frame] != PRTYPE_NONE )
        {
            C_log_error("PointsDescFile file malformed! Two points in the same image have the same trajectory id\n");
            C_log_error("In image %d, several points have trajectory id %d\n",
                    k, p);
            exit(-1);
        }
        trajs[it].type[k - trajs[it].starting_frame] = PRTYPE_REF ;
        trajs[it].points[k - trajs[it].starting_frame].r = p ;
      }
    }

    /* Copy non-null trajectories to tf */
    tf->num_of_trajs = num_trajs ;
    if( num_trajs == max_traj+1 )
    {
      /* No null trajs */
      tf->trajs = trajs ;

      /* No need for relabelling */
    }
    else
    {
      tf->trajs = (traj*)calloc_or_die( num_trajs, sizeof(traj) );
      int cur_traj = 0 ;
      for( int k = 0 ; k <= max_traj ; k++ )
      {
        if( trajs[k].length == 0 ) continue ;
        tf->trajs[cur_traj] = trajs[k] ;
        cur_traj++ ;
      }

      /* Free trajectories */
      free( trajs );

      if( relabel_trajs )
      {
        /* Tag trajectories */
        points_desc_set_traj_tags( pd, tf, n_field );
      }
    }
  }

  return tf ;
/*}}}*/
}

/* Copy basic informations from the source points_desc to the destination
 * points_desc */
void
points_desc_copy_infos( points_desc from, points_desc to )
{
  to->width = from->width ;
  to->height = from->height ;
  to->uid = from->uid ;

  to->orig_first_frame = from->orig_first_frame ;

  for( int k = 0 ; k < to->n_headers ; k++ )
  {
    free( to->header_captions[k] );
    free( to->header_contents[k] );
  }
  free( to->header_captions );
  free( to->header_contents );

  to->n_headers = from->n_headers ;
  to->header_captions = (char**)calloc_or_die( to->n_headers, sizeof(char*) );
  to->header_contents = (char**)calloc_or_die( to->n_headers, sizeof(char*) );
  for( int k = 0 ; k < from->n_headers ; k++ )
  {
    to->header_captions[k] = C_string_dup( from->header_captions[k] );
    to->header_contents[k] = C_string_dup( from->header_contents[k] );
  }

  to->n_fields = from->n_fields ;
  to->tags = (char**)calloc_or_die( to->n_fields, sizeof(char*) );
  for( int k = 0 ; k < from->n_fields ; k++ )
  {
    to->tags[k] = C_string_dup( from->tags[k] );
  }
}
