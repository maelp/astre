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
#include <vision/math/base.h>

/* Two points (whether trajectory or noise points) cannot be generated at the
   same location */

/* ------------------------------------------------------ */
#define P printf
#define ROUND(p) (int)((p)+0.5)

static int nr, nc ;                            /* number of rows, cols */
static int K,n,N ;                             /* number of frames, trajs, noise */
static char r ;                                /* non-constant number of noise points ? */
static char F ;                                /* do not force points to stay in the area */
static float sigma_acc, sigma_orientation_acc ;
static float mean_speed_init = 5;
static float sigma_speed_init = 0.5;

static float max_speed = 0.0;
static float max_accel = 0.0;

static points_desc pd ;

/*******************************************************************************

        Generate two independant gaussian drawings and store them in a and b

*******************************************************************************/
inline static void
gaussian( float mu, float sigma, float *a, float *b )
{
  float r = (float)drand48() ;
  float t = (float)drand48()*M_TWO_PI ;

  if( a ) *a = sigma*sqrt(-2*log(r))*cos(t)+mu ;
  if( b ) *b = sigma*sqrt(-2*log(r))*sin(t)+mu ;
}

/*******************************************************************************

        Check that no point already lies at the position.

*******************************************************************************/
static char
point_is_free( int k, int px, int py )
{
  for( int p = 0 ; p < pd->n_points_in_frame[k] ; p++ )
  {
    if( abs_f(pd->points[k][p*pd->n_fields+0] - px) < 0.0001 &&
        abs_f(pd->points[k][p*pd->n_fields+1] - py) < 0.0001 )
    {
      return FALSE ;
    }
  }
  return TRUE ;
}

/*******************************************************************************

        Check that the point is inside the image.

*******************************************************************************/
char
point_is_in_frame( int px, int py )
{
  return px >= 0 && px < nc && py >= 0 && py < nr ;
}

/*******************************************************************************

        Choose random starting parameters.

*******************************************************************************/
static void
init_trajectory_pos_uniform( float* px, float* py )
{
  *px = (float)drand48()*(float)nc ;
  *py = (float)drand48()*(float)nr ;
}
static void
init_trajectory_pos_on_border( float* px, float* py )
{
  int k = random()%4 ;
  *px = (float)drand48()*(float)nc ;
  *py = (float)drand48()*(float)nr ;
  switch( k )
  {
    case 0: *py = 0 ; break ;
    case 1: *py = nr-1 ; break ;
    case 2: *px = 0 ; break ;
    case 3: *px = nc-1 ; break ;
  }
}
static void
init_trajectory_speed( float* vmag, float* vdir )
{
  gaussian( mean_speed_init, sigma_speed_init, vmag, NULL );
  *vdir = (float)drand48()*2*M_PI ;
}

/*******************************************************************************

        Update trajectory parameters.

*******************************************************************************/
static void
update_trajectory_parameters( float* px, float* py, float* vmag, float* vdir )
{
  float vx = *vmag * cos(*vdir) ;
  float vy = *vmag * sin(*vdir) ;

  *px += vx ;
  *py += vy ;

  gaussian( *vmag, sigma_acc, vmag, NULL );
  gaussian( *vdir, sigma_orientation_acc, vdir, NULL );
}

/*******************************************************************************

        Add a trajectory.

*******************************************************************************/
static int cur_traj_idx = 0 ;
static void
add_trajectory( int starting_frame, int length, float* px, float* py )
{
  for( int p = 0 ; p < length ; p++ )
  {
    int k = starting_frame+p ;
    int n = pd->n_points_in_frame[k] ;
    pd->points[k][n*pd->n_fields+0] = px[p] ;
    pd->points[k][n*pd->n_fields+1] = py[p] ;
    pd->points[k][n*pd->n_fields+2] = cur_traj_idx ;

    pd->n_points_in_frame[k]++ ;
  }
  cur_traj_idx++ ;
}

/*******************************************************************************

        Generate a trajectory.

        If it leaves the image, we wrap it to continue it.

*******************************************************************************/
static void
generate_trajectory( int n )
{
  float* ppx = (float*)calloc_or_die( K, sizeof(float) );
  float* ppy = (float*)calloc_or_die( K, sizeof(float) );

  int starting_frame = 0 ;
  int cur_frame = 0 ;

  float px = -1.0f, py = -1.0f ;
  float vmag = -1.0f, vdir = -1.0f ;

start_trajectory_generation:
  {
    /* generate a trajectory starting at starting_frame */
    cur_frame = starting_frame ;

    /* if we cannot put a trajectory having at least three images, stop */
    if( K-starting_frame < 3 )
      goto done_trajectory_generation ;

    /* If we start on the first frame, uniform distribution */
    if( starting_frame == 0 )
      init_trajectory_pos_uniform( &px, &py );
    /* Otherwise, the trajectory must enter on a border */
    else
      init_trajectory_pos_on_border( &px, &py );

    /* Random initial speed */
    init_trajectory_speed( &vmag, &vdir );

    while( cur_frame < K )
    {
      /* Update trajectory (so that trajectories do not start exactly on the
       * border) */
      update_trajectory_parameters( &px, &py, &vmag, &vdir );

      int ix = ROUND( px );
      int iy = ROUND( py );

      if( !point_is_free(cur_frame, ix, iy) )
      {
        /* restart generation */
        P( "Current trajectory location already occupied by another one, restarting generation!\n" );
        goto start_trajectory_generation ;
      }

      /* Exiting frame */
      if( !point_is_in_frame(ix, iy) )
      {
        if( F )
        {
          int length = cur_frame - starting_frame ;
          /* If we have at least three points, we save trajectory, otherwise we
           * restart with same position, but random speed (to avoid getting locked
           * in some positions) */
          if( length >= 3 )
          {
            add_trajectory( starting_frame, length, ppx, ppy );
            /* We generate a new random trajectory that enters the frame */
            starting_frame = cur_frame ;
            goto start_trajectory_generation ;
          }
          else
          {
            /* We restart the trajectory generation */
            P( "Current trajectory too short, restarting generation!\n" );
            goto start_trajectory_generation ;
          }
        }
        else
        {
          P( "Current trajectory has left the image, restarting generation!\n" );
          goto start_trajectory_generation ;
        }
      }

      ppx[cur_frame-starting_frame] = ix ;
      ppy[cur_frame-starting_frame] = iy ;

      cur_frame++ ;
    }
  }
done_trajectory_generation: ;

  int length = cur_frame - starting_frame ;
  /* If we have at least three points, we save trajectory, otherwise we
   * restart with same position, but random speed (to avoid getting locked
   * in some positions) */
  if( length >= 3 )
  {
    add_trajectory( starting_frame, length, ppx, ppy );

    float traj_max_speed = 0.0;
    float traj_max_accel = 0.0;

    for(int k = 0; k < length; k++)
    {
      if( k > 0 )
      {
        float p0x = ppx[k-1];
        float p0y = ppy[k-1];
        float p1x = ppx[k-0];
        float p1y = ppy[k-0];
        float dx = p1x-p0x;
        float dy = p1y-p0y;
        float speed = sqrt(dx*dx+dy*dy);
        traj_max_speed = max_f(traj_max_speed, speed);
      }
      if( k > 1 )
      {
        float p0x = ppx[k-2];
        float p0y = ppy[k-2];
        float p1x = ppx[k-1];
        float p1y = ppy[k-1];
        float p2x = ppx[k-0];
        float p2y = ppy[k-0];
        float ax = p2x - 2.0*p1x + p0x;
        float ay = p2y - 2.0*p1y + p0y;
        float accel = sqrt(ax*ax+ay*ay);
        traj_max_accel = max_f(traj_max_accel, accel);
      }
    }

    max_speed = max_f(max_speed, traj_max_speed);
    max_accel = max_f(max_accel, traj_max_accel);
  }

  free( ppx ); ppx = (float*)NULL ;
  free( ppy ); ppy = (float*)NULL ;
}

/* MAIN FUNCTION */
/* ------------------------------------------------------ */
/* tpsmg
 *
 * i_K    : number of frames
 * i_n    : number of trajectories
 * o_out  : Rawdata where Pointsdesc file is saved
 * i_w, i_h : width, height
 * i_a    : acceleration magnitude variance
 * i_v    : mean initial speed of trajectories
 * i_V    : variance for the initial speed of trajectories
 * i_N    : number of additional noise points in frames
 * i_r    : if i_r is true, we add a random number of noise points 0 <= r_k <= i_N in frame k
 * i_F    : if i_F is true, trajectories can leave the image (and a new trajectory is added
 *          when this happens)
 */
static void
tpsmg
(
    int i_K, int i_n,
    Rawdata o_out,
    int i_w, int i_h,
    float i_a, float i_o, float i_v, float i_V,
    int i_N, char i_r, char i_F
)
{

  /*                         Verification of the parameters */
  /* ------------------------------------------------------ */
  K = i_K ; C_assert( K > 0 );
  n = i_n ; C_assert( n >= 0 );
  nc = i_w ; C_assert( nc > 0 );
  nr = i_h ; C_assert( nr > 0 );

  N = i_N ; C_assert( N >= 0 );
  r = i_r ;
  F = i_F ;
  
  sigma_acc = i_a ; C_assert( sigma_acc >= 0 );
  sigma_orientation_acc = i_o ; C_assert( sigma_orientation_acc >= 0 );
  mean_speed_init = i_v ; C_assert( mean_speed_init >= 0 );
  sigma_speed_init = i_V ; C_assert( sigma_speed_init >= 0 );

  init_rgen () ;

  /*                                   Creation of the file */
  /* ------------------------------------------------------ */
  pd = points_desc_new();

  pd->width = nc ;
  pd->height = nr ;
  pd->uid = (int)time(NULL)+(int)getpid();
  pd->orig_first_frame = 0 ;
  pd->n_headers = 0 ;
  pd->n_frames = K ;
  pd->n_points_in_frame = (int*)calloc_or_die( K, sizeof(int) );
  pd->n_fields = 3 ;
  pd->tags = (char**)calloc_or_die( pd->n_fields, sizeof(char*) );
  pd->tags[0] = strdup("x");
  pd->tags[1] = strdup("y");
  pd->tags[2] = strdup("t");
  pd->points = (double**)calloc_or_die( K, pd->n_fields*sizeof(double*) );

  for( int k = 0 ; k < K ; k++ )
  {
    pd->n_points_in_frame[k] = 0 ;
    pd->points[k] = (double*)calloc_or_die( (n+N)*pd->n_fields, sizeof(double) );
  }

  /*                              Generate all trajectories */
  /* ------------------------------------------------------ */
  for( int j = 0 ; j < n ; j++ )
  {
    generate_trajectory( j );
  }

  /*               Generate the noise points for each frame */
  /* ------------------------------------------------------ */
  for( int k = 0 ; k < K ; k++ )
  {
    int nrands = r ? random()%(N+1) : N ;
    for( int i = 0 ; i < nrands ; i++ )
    {
      double *p = &(pd->points[k][pd->n_points_in_frame[k]*pd->n_fields]) ;
restart_point: ;
      double px = (double)(random()%nc) ;
      double py = (double)(random()%nr) ;

      /* Avoid generating a point over an already existing one */
      for( int j = 0 ; j < pd->n_points_in_frame[k] ; j++ )
      {
        double* q = &(pd->points[k][j*pd->n_fields]);
        double qx = *q ;
        double qy = *(q+1) ;
        if( abs_d(qx-px) < 0.1 && abs_d(qy-py) < 0.1 )
          goto restart_point ;
      }

      *(p++) = px ;
      *(p++) = py ;
      *(p++) = -1.0 ;

      pd->n_points_in_frame[k]++ ;
    }
  }

  /*                         Random shuffling of the points */
  /* ------------------------------------------------------ */
  /* Just to be sure we avoid correlation in all detection algorithms
   * between the detected trajectory and the position of the points in
   * the file, we randomly shuffle the points */
  for( int k = 0 ; k < K ; k++ )
  {
    for( int j = 1 ; j < pd->n_points_in_frame[k] ; j++ )
    {
      int r = random()%(j+1) ;
      if( j != r )
      {
        double *pj = &(pd->points[k][j*pd->n_fields]);
        double *pr = &(pd->points[k][r*pd->n_fields]);

        for( int u = 0 ; u < 3 ; u++ )
        {
          double dj = *(pj);
          double dr = *(pr);
          *pj = dr ; *pr = dj ;
          pj++ ; pr++ ;
        }
      }
    }
  }

  /* Output metadata */
  P("[MD] { 'max_speed': %g, 'max_accel': %g }\n", max_speed, max_accel);

  points_desc_save( o_out, pd );
  points_desc_free_all( &pd );
}

int main( int ARGC, char** ARGV )
{
  arg_parser ap = arg_parser_new();

  arg_parser_set_info( ap,
"Point Set Motion Generator, generate smooth trajectories with optional noise points" );

  struct arg_int *p_K = arg_int1( NULL, NULL, "K",
      "Number of frames" );
  arg_parser_add( ap, p_K );

  struct arg_int *p_n = arg_int1( NULL, NULL, "n",
      "Number of trajectories" );
  arg_parser_add( ap, p_n );

  struct arg_str *p_out = arg_str1( NULL, NULL, "out",
      "Output file" );
  arg_parser_add( ap, p_out );

  struct arg_int *p_w = arg_int0( "w", "width", "<w>", "Image width (default: 100)" );
  if( p_w ) p_w->ival[0] = 100 ;
  arg_parser_add( ap, p_w );

  struct arg_int *p_h = arg_int0( "h", "height", "<h>", "Image height (default: 100)" );
  if( p_h ) p_h->ival[0] = 100 ;
  arg_parser_add( ap, p_h );

  struct arg_dbl *p_a = arg_dbl0( "a", "acc", "<a>", "Variance of the speed amplitude update (default: 0.2)" );
  if( p_a ) p_a->dval[0] = 0.2 ;
  arg_parser_add( ap, p_a );

  struct arg_dbl *p_o = arg_dbl0( "o", "acc_orientation", "<a>", "Variance of the speed orientation update (default: 0.2)" );
  if( p_o ) p_o->dval[0] = 0.2 ;
  arg_parser_add( ap, p_o );

  struct arg_dbl *p_v = arg_dbl0( "v", "mean-initial-speed", "<v>", "Mean initial speed amplitude (default: 5.0)" );
  if( p_v ) p_v->dval[0] = 5.0 ;
  arg_parser_add( ap, p_v );

  struct arg_dbl *p_V = arg_dbl0( "V", "var-initial-speed", "<V>", "Initial speed variance (default: 0.5)" );
  if( p_V ) p_V->dval[0] = 0.5 ;
  arg_parser_add( ap, p_V );

  struct arg_int *p_N = arg_int0( "N", "noise-points", "<N>", "Number of noise points (default: 0)" );
  if( p_N ) p_N->ival[0] = 0 ;
  arg_parser_add( ap, p_N );

  struct arg_lit *p_r = arg_lit0( "r", NULL, "Non constant number of noise points" );
  arg_parser_add( ap, p_r );

  struct arg_lit *p_F = arg_lit0( "F", NULL, "Do not force the trajectories to stay in the image" );
  arg_parser_add( ap, p_F );

  /* Handle arguments */
  arg_parser_handle( ap, ARGC, ARGV );

  /* Check arguments */
  int K = p_K->ival[0]; C_assert( K > 0 );
  int n = p_n->ival[0]; C_assert( n >= 0 );
  char* out = (char*)p_out->sval[0]; C_assert( out && strlen(out) > 0 );
  int w = p_w->ival[0]; C_assert( w > 0 );
  int h = p_h->ival[0]; C_assert( h > 0 );
  double a = p_a->dval[0]; C_assert( a >= 0 );
  double o = p_o->dval[0]; C_assert( o >= 0 );
  double v = p_v->dval[0]; C_assert( v >= 0 );
  double V = p_V->dval[0]; C_assert( V >= 0 );
  int N = p_N->ival[0]; C_assert( N >= 0 );
  char r = p_r->count > 0 ;
  char F = p_F->count > 0 ;

  Rawdata rd_out = new_rawdata_or_die();

  tpsmg( K, n, rd_out, w, h, a, o, v, V, N, r, F );

  save_rawdata( rd_out, out );

  mw_delete_rawdata( rd_out );

  /* Clean memory */
  arg_parser_free_all( &ap );
}
