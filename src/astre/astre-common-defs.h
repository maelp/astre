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

#ifndef TASTRE_H
#define TASTRE_H

#include <vision/core.h>
#include <vision/math/combinatorics.h>
#include <vision/trajs/pointsdesc.h>
#include <vision/trajs/trajs.h>

#ifdef ASTRE_HAS_HOLES
 #undef ASTRE_HAS_NO_HOLES
#else
 #ifndef ASTRE_HAS_NO_HOLES
  #error "Please define either ASTRE_HAS_HOLES or ASTRE_HAS_NO_HOLES"
 #endif
#endif

/* You can optionnaly define ASTRE_HAS_INTENSITY */

/*                                    Notes
  =============================================================================
 
  -----------------------------------------------------------------------------
 
         Naming conventions
 
  -----------------------------------------------------------------------------
   When using multidimensional arrays, we try to use a "Hungarian-like" notation
   that tells what index correspond to what. Usual suffixes are 'f' for frame,
   'p' for point, etc.
 
   Eg.: to store activated points : activated_fp[frame][point_id] ;
 
*/

/*******************************************************************************

        Constants and global variables

TODO:
  - pourquoi est-ce que INFTY est un float? (et non un double)
  - vérifier que l'on conserve les positions des points en double pour éviter
    les conversions

*******************************************************************************/

/* INFTY value, usually denotes impossible paths or unintialized variables */
static const float INFTY = 1E18 ;

/* This is the precision used when comparing log NFAs */
static const double LOG_NFA_COMP_EPS = 1E-5 ;

static points_desc pd ;
static trajs_file tf ;
static int n_fields ;                           /* quick access to pd->n_fields */
static int *n_points_in_frame ;            /* quick access to pd->n_points_in_frame */
static double **points ;                        /* quick access to pd->points */

static double* IMAGE_AREA ;
static int N ;                                  /* maximal number of points in a frame */
static int K ;                                  /* number of frames */

static double* LOG_IMAGE_AREA ;                 /* log( IMAGE_AREA ) */
static double LOG_N ;                           /* log( N ) */
static double LOG_K ;                           /* log( K ) */



/*******************************************************************************

        Parameters.

        If we have to choose some parameters to lower the computational
        overhead (maximal length of trajectories, maximal length of holes,
        maximal speed of trajectories, etc.), we put them here, to have a
        clear view of all parameters to tune.

*******************************************************************************/

/** Maximal allowed log NFA */
static double MAX_ALLOWED_LOG_NFA = 0 ;

/** Maximal allowed trajectory length (ie. number of points in the trajectory).
 * Set this to 0 to allow any length. */
static int MAX_ALLOWED_TRAJECTORY_LENGTH = 0 ;

#ifdef ASTRE_HAS_HOLES
/** Maximal allowed hole length (set as a command line parameter) */
static int MAX_ALLOWED_HOLE_LENGTH ;
#endif

/*******************************************************************************

        Potentially useful precomputations.

*******************************************************************************/

/*******************************************************************************

        Precompute the combinatorial coefficients of the log NFA.

        array : k => log(k), k = 1..K

*******************************************************************************/
static double* LOG_k ;
static void
precompute_log_k()
{
  LOG_k = combinatorics_log_K_init( K );
}

/*******************************************************************************

        Precompute the combinatorial coefficients of the log NFA.

        array : n*(K+1)+k => log( comb(n,k) ), n,k = 0..K

*******************************************************************************/
static double* LOG_Cnk ;
static void
precompute_log_cnk()
{
  LOG_Cnk = combinatorics_log_Cnk_init( K );
}

/*******************************************************************************

        Precompute the LOG_Kfact array.

        array : k => log(k!), k = 0..K

*******************************************************************************/
static double* LOG_Kfact ;
static void
precompute_log_kfact()
{
  LOG_Kfact = combinatorics_log_Kfact_init( K );
}

/*******************************************************************************

        Precompute the LOG_Nprod array.

        LOG_Nprod[k][l][s] = max_{k = i_1 < ... < i_s = k+l-1}
                               log(N_{i_1} * ... * N_{i_s}) if l >= 2, s >= 2

        this is the maximal possible value of log( N_i1 * ... * N_is ) for a
        trajectory of length l and size s starting at frame k, that we use to
        compute the upper bound on all these values for the NFA.

*******************************************************************************/
static double*** LOG_Nprod ;

static void
precompute_log_nprod()
{
  double* sort_array = (double*)calloc_or_die( K, sizeof(double) );

  LOG_Nprod = (double***)calloc_or_die( K, sizeof(double**) );
  for( int k = 0 ; k < K ; k++ )
  {
    LOG_Nprod[k] = (double**)calloc_or_die( K-k+1, sizeof(double*) );
    for( int l = 0 ; l <= K-k ; l++ )
    {
      LOG_Nprod[k][l] = (double*)calloc_or_die( l+1, sizeof(double) );
      double lNprod = -1.0 ;

      if( l >= 2 )
      {
        if( n_points_in_frame[k] == 0 || n_points_in_frame[k+l-1] == 0 )
          lNprod = -1.0 ;
        else
          lNprod = log10(n_points_in_frame[k]) + log10(n_points_in_frame[k+l-1]) ;

        for( int p = k+1 ; p < k+l-1 ; p++ )
        {
          sort_array[p-k-1] = n_points_in_frame[p] ;
        }
        /* sort in descending order */
        qsort( sort_array, l-2, sizeof(double), &compar_d_descend );
      }

      for( int s = 0 ; s <= l ; s++ )
      {
        if( s < 2 )
        {
          LOG_Nprod[k][l][s] = -1.0 ;
        }
        /* lNprod = -1 : frames have no more points, impossible to create a trajectory */
        else if( lNprod >= -0.5 && s >= 3 )
        {
          if( sort_array[s-3] == 0 )
            lNprod = -1.0 ;
          else
            lNprod += log10( sort_array[s-3] );
        }

        LOG_Nprod[k][l][s] = lNprod ;
      }
    }
  }

  free( sort_array ); sort_array = (double*)NULL ;
}

static void
log_nprod_free()
{
  for( int k = 0 ; k < K ; k++ )
  {
    for( int l = 0 ; l <= K-k ; l++ )
    {
      free( LOG_Nprod[k][l] ); LOG_Nprod[k][l] = (double*)NULL ;
    }
    free( LOG_Nprod[k] ); LOG_Nprod[k] = (double**)NULL ;
  }
  free( LOG_Nprod ); LOG_Nprod = (double***)NULL ;
}

/*******************************************************************************

        Active points.
        activated_fp[f][j] is TRUE iif point j in frame f is active.

*******************************************************************************/

static char **activated_fp ;

static void
activated_fp_init()
{
  activated_fp = (char**)calloc_or_die( K, sizeof(char*) );
  for( int k = 0 ; k < K ; k++ )
  {
    activated_fp[k] =
      (char*)calloc_or_die( n_points_in_frame[k], sizeof(char) );

    for( int p = 0 ; p < n_points_in_frame[k] ; p++ )
      activated_fp[k][p] = TRUE ;
  }
}

static void
activated_fp_free()
{
  for( int k = 0 ; k < K ; k++ )
  {
    free( activated_fp[k] ); activated_fp[k] = (char*)NULL ;
  }
  free( activated_fp ); activated_fp = (char**)NULL ;
}

/*******************************************************************************

        Image areas.

*******************************************************************************/
void precompute_image_areas(double defined_image_area, char auto_crop)
{
  IMAGE_AREA = (double*)calloc_or_die(K, sizeof(double));
  LOG_IMAGE_AREA = (double*)calloc_or_die(K, sizeof(double));
  for( int k = 0 ; k < K ; k++ )
  {
    IMAGE_AREA[k] = -1.0 ;
    LOG_IMAGE_AREA[k] = -1.0 ;

    double area = defined_image_area ;
    if( auto_crop )
    {
      double xmin=-1.0, ymin=-1.0, xmax=-1.0, ymax=-1.0 ;
      char is_init = FALSE ;
      for( int p = 0 ; p < n_points_in_frame[k] ; p++ )
      {
        double pX = points[k][p*n_fields+0] ;
        double pY = points[k][p*n_fields+1] ;
        if( !is_init )
        {
          is_init = TRUE ;
          xmin = xmax = pX ;
          ymin = ymax = pY ;
        }
        else
        {
          xmin = min_d(xmin, pX);
          xmax = max_d(xmax, pX);
          ymin = min_d(ymin, pY);
          ymax = max_d(ymax, pY);
        }
      }
      area = max_d( 1.0, (xmax-xmin)*(ymax-ymin) );
    }
    IMAGE_AREA[k] = area;
    LOG_IMAGE_AREA[k] = log10(area);
  }
}

void free_image_areas()
{
  free(IMAGE_AREA); IMAGE_AREA = (double*)NULL ;
  free(LOG_IMAGE_AREA); LOG_IMAGE_AREA = (double*)NULL ;
}

/*******************************************************************************

        Trajectory store variables.  This maintains a resizable buffer
        of trajectories.

*******************************************************************************/

static traj_store trajectory_store ;

void traj_store_init(int ninit)
{
  trajectory_store = traj_store_create(ninit);
}

/*******************************************************************************

        Save partial results

*******************************************************************************/

char* partial_results_fname = (char*)NULL ;

#ifdef ASTRE_HAS_NO_HOLES
  /*******************************************************************************
  
          Store for the G function computations.
  
          g_fxl[k][idx(x,y)][l0] = G( x^k, y^k-1, l )
  
          The stored value is either the value of the G function if such a path
          exists, or INFTY if none exists.
  
          NMAX = maximal index of a point = N-1
          HMAX = maximal length of a hole starting from image k
          idx(x,y) = x*(NMAX+1) + y
  
          n{k} = # of points in frame k (where frames = 0 .. K-1)
  
          k    = 1 .. K-1                                 Current frame of x
          x    = 0 .. n{k} - 1                            Index of x
          y    = 0 .. n{k-1} - 1                          Index of y
          l    = 2 .. k+1
  
  *******************************************************************************/
  static float*** g_fxl ;
  
  /* Macros to ease the access to the G array */

  #define VDEFINE_MAX_points(max_x,k)  const int max_x = n_points_in_frame[(k)] - 1 ;

  #define VDEFINE_MAX_l(max_l,k)       const int max_l = min_i( MAX_ALLOWED_TRAJECTORY_LENGTH, (k)+1 );
  #define VDEFINE_MIN_l(min_l,k)       const int min_l = 3 ;
  #define VDEFINE_BOUNDS_l(max_l,min_l,size_l0,k) \
                                       VDEFINE_MAX_l(max_l,(k)); \
                                       VDEFINE_MIN_l(min_l,(k)); \
                                       const int size_l0 = max_l - min_l + 1 ;
  
  #define DEFINE_MAX_k                 const int __max_k = K-1 ;
  
  #define DEFINE_MAX_x(k)              VDEFINE_MAX_points(__max_x,(k));
  #define DEFINE_MAX_y(p)              VDEFINE_MAX_points(__max_y,(p));
  #define DEFINE_MAX_z(q)              VDEFINE_MAX_points(__max_z,(q));
  
  #define DEFINE_MAX_l(k)              VDEFINE_MAX_l(__max_l,(k));
  #define DEFINE_MIN_l(k)              VDEFINE_MIN_l(__min_l,(k));
  #define DEFINE_BOUNDS_l(k)           VDEFINE_BOUNDS_l(__max_l,__min_l,__size_l0,(k))
  
  #define DEFINE_MAX_l_prev(p)         VDEFINE_MAX_l(__max_l_prev,(p));
  #define DEFINE_MIN_l_prev(p)         VDEFINE_MIN_l(__min_l_prev,(p));
  #define DEFINE_BOUNDS_l_prev(p) \
                            VDEFINE_BOUNDS_l(__max_l_prev,__min_l_prev,__size_l0_prev,(p))

  /* Macros to ease the traversal of the G array */
  
  #define FORALL_k \
    DEFINE_MAX_k ; \
    for( int k = 1 ; k <= __max_k ; k++ ) \
    {
  
  #define FORALL_x \
      DEFINE_MAX_x(k); \
      char* activatedX = activated_fp[k] ; \
      float** g_xl = g_fxl[k]; \
      \
      for( int x = 0 ; x <= __max_x ; x++ ) \
      { \
        if( !activatedX[x] ) continue ; \
        const int x_idx = x*N ;
  
  #define FORALL_y \
        const int p = k-1; \
        DEFINE_MAX_y(p); \
        DEFINE_BOUNDS_l(k); \
        char* activatedY = activated_fp[p] ; \
        \
        for( int y = 0 ; y <= __max_y ; y++ ) \
        { \
          if( !activatedY[y] ) continue ; \
          int xy_idx = x_idx + y ;

  #define FORALL_l \
          float* g_l = g_xl[xy_idx] ; \
          \
          int l = __min_l ; \
          float* g_l_first = &(g_l[0]); \
          float* g_l_last = &(g_l[__size_l0]); \
          \
          for( float* g_l_cur = g_l_first ; g_l_cur != g_l_last ; g_l_cur++, l++ ) \
          {
  
  #define END_FORALL_k }
  #define END_FORALL_x }
  #define END_FORALL_y }
  #define END_FORALL_l }
#endif // ASTRE_HAS_NO_HOLES

#ifdef ASTRE_HAS_HOLES
  /*******************************************************************************
  
          Store for the G function computations.
  
          g_fxlsj[k][idx(x,y,h)][l0][s0][j0] = G( x^k, h, y^k-h-1, l, s, j )
  
          The stored value is either the value of the G function if such a path
          exists, or INFTY if none exists.
  
          NMAX = maximal index of a point = N-1
          HMAX = maximal length of a hole starting from image k
          idx(x,y,h) = x*(NMAX+1)*(HMAX+1) + h*(NMAX+1) + y
  
          n{k} = # of points in frame k (where frames = 0 .. K-1)
  
          k    = 1 .. K-1                                 Current frame of x
          h    = 0 .. min( MAX_ALLOWED_HOLE_LENGTH, k-1 )
          x    = 0 .. n{k} - 1                            Index of x
          y    = 0 .. n{k-h-1} - 1                        Index of y
          l    = 2+h .. k+1
          s    = 2 .. (l-h)                Size (number of present points) of traj
          j    = eps_j .. min( l-s+1, s )
                                      Number of runs of consecutive present points
                                      where eps_j = 1 if h = 0
                                            eps_j = 2 otherwise
  
          l0 = l - l_min = l - (2+h)
          s0 = s - s_min = s - 2
          j0 = j - j_min = j - eps_j
  
  *******************************************************************************/
  static float***** g_fxlsj ;
  
  /* Macros to ease the access to the G array */
  
  #define VDEFINE_MAX_points(max_x,k)  const int max_x = n_points_in_frame[(k)] - 1 ;

  #define VDEFINE_MAX_h(max_h,k)       const int max_h = min_i( (k)-1, MAX_ALLOWED_HOLE_LENGTH );

  #define VDEFINE_MAX_l(max_l,k,h)     const int max_l = min_i( MAX_ALLOWED_TRAJECTORY_LENGTH, (k)+1 );
  #define VDEFINE_MIN_l(min_l,k,h)     const int min_l = (h)+3 ;
  #define VDEFINE_BOUNDS_l(max_l,min_l,size_l0,k,h) \
                                       VDEFINE_MAX_l(max_l,(k),(h)); \
                                       VDEFINE_MIN_l(min_l,(k),(h)); \
                                       const int size_l0 = max_l - min_l + 1 ;
  
  #define VDEFINE_MAX_s(max_s,k,h,l)   const int max_s = (l)-(h) ;
  #define VDEFINE_MIN_s(min_s,k,h,l)   const int min_s = 3 ;
  #define VDEFINE_BOUNDS_s(max_s,min_s,size_s0,k,h,l) \
                                       VDEFINE_MAX_s(max_s,(k),(h),(l)); \
                                       VDEFINE_MIN_s(min_s,(k),(h),(l)); \
                                       const int size_s0 = max_s - min_s + 1 ;
  
  #define VDEFINE_MAX_j(max_j,k,h,l,s) const int max_j = min_i( l-s+1, s );
  #define VDEFINE_MIN_j(min_j,k,h,l,s) const int min_j = ((h) == 0) ? 1 : 2 ;
  #define VDEFINE_BOUNDS_j(max_j,min_j,size_j0,k,h,l,s) \
                                       VDEFINE_MAX_j(max_j,(k),(h),(l),(s)); \
                                       VDEFINE_MIN_j(min_j,(k),(h),(l),(s)); \
                                       const int size_j0 = max_j - min_j + 1 ;
  
  #define DEFINE_MAX_k                 const int __max_k = K-1 ;
  
  #define DEFINE_MAX_x(k)              VDEFINE_MAX_points(__max_x,(k));
  #define DEFINE_MAX_h(k)              VDEFINE_MAX_h(__max_h,(k));
  #define DEFINE_MAX_y(p)              VDEFINE_MAX_points(__max_y,(p));
  #define DEFINE_MAX_h_prev(p)         VDEFINE_MAX_h(__max_h_prev,(p));
  #define DEFINE_MAX_z(q)              VDEFINE_MAX_points(__max_z,(q));
  
  #define DEFINE_MAX_l(k,h)            VDEFINE_MAX_l(__max_l,(k),(h));
  #define DEFINE_MIN_l(k,h)            VDEFINE_MIN_l(__min_l,(k),(h));
  #define DEFINE_BOUNDS_l(k,h)         VDEFINE_BOUNDS_l(__max_l,__min_l,__size_l0,(k),(h))
  
  #define DEFINE_MAX_s(k,h,l)          VDEFINE_MAX_s(__max_s,(k),(h),(l));
  #define DEFINE_MIN_s(k,h,l)          VDEFINE_MIN_s(__min_s,(k),(h),(l));
  #define DEFINE_BOUNDS_s(k,h,l)       VDEFINE_BOUNDS_s(__max_s,__min_s,__size_s0,(k),(h),(l));
  
  #define DEFINE_MAX_j(k,h,l,s)        VDEFINE_MAX_j(__max_j,(k),(h),(l),(s));
  #define DEFINE_MIN_j(k,h,l,s)        VDEFINE_MIN_j(__min_j,(k),(h),(l),(s));
  #define DEFINE_BOUNDS_j(k,h,l,s)     VDEFINE_BOUNDS_j(__max_j,__min_j,__size_j0,(k),(h),(l),(s));
  
  #define DEFINE_MAX_l_prev(p,h_prev)  VDEFINE_MAX_l(__max_l_prev,(p),(h_prev));
  #define DEFINE_MIN_l_prev(p,h_prev)  VDEFINE_MIN_l(__min_l_prev,(p),(h_prev));
  #define DEFINE_BOUNDS_l_prev(p,h_prev) \
                            VDEFINE_BOUNDS_l(__max_l_prev,__min_l_prev,__size_l0_prev,(p),(h_prev))
  
  #define DEFINE_MAX_s_prev(p,h_prev,l_prev)  VDEFINE_MAX_s(__max_s_prev,(p),(h_prev),(l_prev));
  #define DEFINE_MIN_s_prev(p,h_prev,l_prev)  VDEFINE_MIN_s(__min_s_prev,(p),(h_prev),(l_prev));
  #define DEFINE_BOUNDS_s_prev(p,h_prev,l_prev) \
                            VDEFINE_BOUNDS_s(__max_s_prev,__min_s_prev,__size_s0_prev,(p),(h_prev),(l_prev))
  
  #define DEFINE_MAX_j_prev(p,h_prev,l_prev,s_prev) \
                            VDEFINE_MAX_j(__max_j_prev,(p),(h_prev),(l_prev),(s_prev));
  #define DEFINE_MIN_j_prev(p,h_prev,l_prev,s_prev) \
                            VDEFINE_MIN_j(__min_j_prev,(p),(h_prev),(l_prev),(s_prev));
  #define DEFINE_BOUNDS_j_prev(p,h_prev,l_prev,s_prev) \
                            VDEFINE_BOUNDS_j(__max_j_prev,__min_j_prev,__size_j0_prev,\
                                             (p),(h_prev),(l_prev),(s_prev))
  
  /* Macros to ease the traversal of the G array */
  
  #define FORALL_k \
    DEFINE_MAX_k ; \
    for( int k = 1 ; k <= __max_k ; k++ ) \
    {
  
  #define FORALL_x \
      DEFINE_MAX_x(k); \
      DEFINE_MAX_h(k); \
      char* activatedX = activated_fp[k] ; \
      float**** g_xlsj = g_fxlsj[k]; \
      \
      for( int x = 0 ; x <= __max_x ; x++ ) \
      { \
        if( !activatedX[x] ) continue ; \
        const int x_idx = x*N*(__max_h+1) ;
  
  #define FORALL_h \
        for( int h = 0 ; h <= __max_h ; h++ ) \
        { \
          const int xh_idx = x_idx + h*N ; \
          const int p = k-h-1 ;
  
  #define FORALL_y \
          DEFINE_MAX_y(p); \
          DEFINE_BOUNDS_l(k,h); \
          char* activatedY = activated_fp[p] ; \
          \
          for( int y = 0 ; y <= __max_y ; y++ ) \
          { \
            if( !activatedY[y] ) continue ; \
            const int xhy_idx = xh_idx + y ;
  
  #define FORALL_l \
            float*** g_lsj = g_xlsj[xhy_idx] ; \
            \
            for( int l0 = 0, l = __min_l ; l0 < __size_l0 ; l0++, l++ ) \
            {
  
  #define FORALL_s \
              DEFINE_BOUNDS_s( k, h, l ); \
              float** g_sj = g_lsj[l0] ; \
              \
              for( int s0 = 0, s = __min_s ; s0 < __size_s0 ; s0++, s++ ) \
              {
  
  #define FORALL_j \
                DEFINE_BOUNDS_j( k, h, l, s ); \
                float* g_j = g_sj[s0]; \
                \
                int j = __min_j ; \
                float* g_j_first = &(g_j[0]); \
                float* g_j_after_last = &(g_j[__size_j0]); \
                \
                for( float* g_j_cur = g_j_first ; g_j_cur != g_j_after_last ; g_j_cur++, j++ ) \
                {
  
  #define END_FORALL_k }
  #define END_FORALL_x }
  #define END_FORALL_h }
  #define END_FORALL_y }
  #define END_FORALL_l }
  #define END_FORALL_s }
  #define END_FORALL_j }
#endif // ASTRE_HAS_HOLES

/* Forward declarations */

#endif


double find_minimal_NFA();
#ifdef ASTRE_HAS_NO_HOLES
char extract_trajectory_if_possible( int k, int x, int y, int l, double info_logNFA );
#endif
#ifdef ASTRE_HAS_HOLES
char
extract_trajectory_if_possible( int k,
                                int x, int h, int y,
                                int l, int s, int j,
                                double info_logNFA );
#endif

char extract_and_disable_most_significant_trajectories ();
void compute_most_significant_trajectories();
void do_detect();
void compute_caracteristics_of_trajectory( int starting_frame, int length, int* type, union u_ref_point* points, float* o_delta, int* o_s, int* o_j );
double compute_log_NFA_of_trajectory( int starting_frame, int length, int* type, union u_ref_point* points );
void info_on_traj( char* str );

/* Discrete area
 *
 * Motivation:
 * ===========
 * When we compute NFAs, we often want to compute the areas of the
 * discrete balls containing some pixel (rather than approximating this
 * area by the area of the euclidian ball containing that pixel, which is
 * problematic for instance if the pixel is the one at the origin, resulting in
 * a null area).
 *
 * The discrete balls are the intersections of the Z^2 grid with the
 * euclidian balls.
 *
 * We set a strictly positive real number r that will give a bound for
 * the computed areas. If we are given a radius greater than r, we may
 * approximate the discrete ball area by the euclidian ball area.
 *
 * We output an area computation code.
 *
 * Output:
 * =======
 * For memory efficiency purposes, the discrete_area_data do not contain
 * the representation of the entire [-r,r]x[-r,r] area. We still
 * represent the first quarter (instead of the first octant) because we
 * want a trade-off between memory and efficiency.
 *
 * To compute the ball sizes, we still only consider the first octant for
 * speed reasons. We check all the (x,y) pairs, we compute their
 * associated radii, we sort them, and we compute the areas.
 *
 * Overview:
 * =========
 *
 * We sort all the pixels of distance less than r from the origin by their
 * distance to the origin. We process "by layers", counting the contribution
 * to the area of all pixels of a certain distance from the origin.
 *
 * Code:
 * =====
 *
 * discrete_area_init( max_r );
 *
 * int vx = ..., vy = ... ;
 * double t = discrete_area( vx, vy );
 *
 * discrete_area_free();
 *
 **********************************************************/

static const double discrete_area_EPS = 1E-5 ;
static int discrete_area_max_r = -1 ;
static int discrete_area_max_r_sq = -1 ;
static int discrete_area_width = -1 ;
static double* discrete_area_data = (double*)NULL ;

typedef struct
{
  int x, y ;
  double rad ;
} discrete_area_pix;

/* Comparison function for quicksort */
static int
discrete_area_compare_pixs( const void *a, const void *b )
{
  double t = ((discrete_area_pix*)a)->rad - ((discrete_area_pix*)b)->rad ;
  if( t < -discrete_area_EPS ) return -1 ;
  else if( t > discrete_area_EPS ) return 1 ;
  else return 0 ;
}

static void discrete_area_init(int max_r);
static void discrete_area_free();
static double discrete_area(int x, int y);

