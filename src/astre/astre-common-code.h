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
#include <cbase/util.h>

#define P printf

/*******************************************************************************

        Discrete area

*******************************************************************************/
/*{{{*/
/*******************************************************************************

        Discrete area
        Initialize the structures

        PARAMETER:
          max_r -- maximal radius

*******************************************************************************/
static void
discrete_area_init( int max_r )
{
  if( max_r < 0 )
  {
    C_log_error( "Incorrect radius %d!\n", max_r );
    exit(-1);
  }

  if( discrete_area_data )
  {
    discrete_area_free();
  }

  discrete_area_max_r = max_r ;
  discrete_area_max_r_sq = max_r*max_r ;
  discrete_area_width = max_r+1 ;
  discrete_area_data = (double*)calloc_or_die( (max_r+1)*(max_r+1), sizeof(double) );

  for( int i = 0 ; i < (max_r+1)*(max_r+1) ; i++ )
    discrete_area_data[i] = -1.0 ;

  /* Total number of pixel in the first octant */
  int total_num_pix = (max_r+1)*(max_r+2)/2 ;
  discrete_area_pix* pixels =
    (discrete_area_pix*)calloc_or_die( total_num_pix, sizeof(discrete_area_pix) );

  /* Generate all pixels in the first octant, that have infinity norm less than r */
  {
    discrete_area_pix* cur_pix = pixels ;
    for( int x = 0 ; x <= max_r ; x++ )
    for( int y = 0 ; y <= x ; y++ )
    {
      cur_pix->x = x ;
      cur_pix->y = y ;
      cur_pix->rad = sqrt( (double)(x*x + y*y) );
      cur_pix++ ;
    }
  }

  /* Sort by their radii */
  qsort( pixels, total_num_pix, sizeof(discrete_area_pix), &discrete_area_compare_pixs );

  int cur_area = 0 ;
  /* We are only looking for the areas of balls with euclidian radii less than
   * r, so we determine how many we have to consider */
  int num_pix = 0 ;
  while( num_pix < total_num_pix &&
      pixels[num_pix].rad <= (double)max_r+discrete_area_EPS )
    num_pix++ ;

  /* Several pixels are at an equal distance from the center, so we need to
   * compute all their contributions to the current area. This is why we
   * consider successive "layers" of pixels */
  int fst_pix_layer = 0 ;
  while( fst_pix_layer < num_pix )
  {
    double cur_radius = pixels[fst_pix_layer].rad ;
    int lst_pix_layer = fst_pix_layer ;
    while( lst_pix_layer < num_pix-1 &&
        pixels[lst_pix_layer+1].rad - cur_radius < discrete_area_EPS )
      lst_pix_layer++ ;

    /* Add the contribution of all pixels to the ball area */
    for( int q = fst_pix_layer ; q <= lst_pix_layer ; q++ )
    {
      discrete_area_pix *pp = &( pixels[q] );
      int px = pp->x, py = pp->y ;

      /* Increment the area depending on the pixel's position in the octant (when
       * we replicate the octant 8 times, the central pixel is only replicated
       * once, the border pixels are replicated 4 times, and the inside pixels
       * are replicated 8 times. */
      if( px == 0 && py == 0 ) cur_area += 1 ;
      else if( py == 0 || px == py ) cur_area += 4 ;
      else cur_area += 8 ;
    }

    /* Set the ball area of the corresponding pixels */
    for( int q = fst_pix_layer ; q <= lst_pix_layer ; q++ )
    {
      discrete_area_pix *pp = &( pixels[q] );
      int px = pp->x, py = pp->y ;

      /* Discrete_area_data is the first quadrant, so we fill it by symmetry */
      discrete_area_data[py*discrete_area_width + px] = (double)cur_area ;
      discrete_area_data[px*discrete_area_width + py] = (double)cur_area ;
    }

    fst_pix_layer = lst_pix_layer + 1 ;
  }

  free( pixels ); pixels = (discrete_area_pix*)NULL ;
}

/*******************************************************************************

        Discrete area
        Free the structures

*******************************************************************************/
static void
discrete_area_free()
{
  discrete_area_max_r = -1 ;
  discrete_area_max_r_sq = -1 ;
  discrete_area_width = -1 ;
  free( discrete_area_data ); discrete_area_data = (double*)NULL ;
}

/*******************************************************************************

        Discrete area

        PARAMETERS:
          x, y  --  x and y coordinate of the pixel

        RETURNS:
          The area of the smallest discrete ball centered on (0, 0) containing
          the pixel (if norm(x,y) <= discrete_area_max_r) or the area of the
          smallest euclidian ball centered on (0, 0) containing the pixel
          (otherwise)

*******************************************************************************/
static double
discrete_area( int x, int y )
{
  int d_sq = x*x + y*y ;
  if( d_sq > discrete_area_max_r_sq ) return M_PI*(double)d_sq ;
  else
  {
    if( x < 0 ) x = -x ;
    if( y < 0 ) y = -y ;
    return discrete_area_data[y*discrete_area_width+x] ;
  }
}
/*}}}*/

/*******************************************************************************

        Find the NFA of the most significant trajectory.

*******************************************************************************/

double
find_minimal_NFA()
{
/*{{{*/
  float min_log_NFA = INFTY ;

  FORALL_k

#ifdef ASTRE_HAS_NO_HOLES
    FORALL_x ; FORALL_y ; FORALL_l
    const float delta = *g_l_cur ;
#endif
#ifdef ASTRE_HAS_HOLES
    FORALL_x ; FORALL_h ; FORALL_y
    FORALL_l ; FORALL_s ; FORALL_j
    const float delta = *g_j_cur ;
#endif

      if( delta >= INFTY-1 ) continue ; /* No trajectory */

#ifdef ASTRE_HAS_NO_HOLES
      FIND_MINIMAL_NFA__COMPUTE_NFA(k,x,y,l,delta);
#endif
#ifdef ASTRE_HAS_HOLES
      FIND_MINIMAL_NFA__COMPUTE_NFA(k,x,y,l,s,j,delta);
#endif

      if( lNFA < min_log_NFA )
      {
        min_log_NFA = lNFA ;
      }

#ifdef ASTRE_HAS_NO_HOLES
    END_FORALL_l ; END_FORALL_y ; END_FORALL_x
#endif
#ifdef ASTRE_HAS_HOLES
    END_FORALL_j ; END_FORALL_s ; END_FORALL_l
    END_FORALL_y ; END_FORALL_h ; END_FORALL_x
#endif

  END_FORALL_k

  return min_log_NFA ;
/*}}}*/
}

/*******************************************************************************

        Extract a trajectory if it is unbroken (ie. all the points are
        activated). Add the trajectory to the store if it is valid.

        Also, we don't consider every possible trajectory of the same type
        ending in the same points, but we try to greedily detect one.
        This means that we can conclude that the "greedily found" trajectory
        is broken and return FALSE while there exist an unbroken trajectory.
        If there really exist a trajectory, we will still find it at the next
        computation / extraction iteration of the algorithm.

        PARAMETERS:

          k     = End frame
          x     = Index of the point in the last frame
          h     = Size of the first hole
          y     = Index of the point in the previous frame
          l     = Length (ie. span of the trajectory)
          s     = Size (ie. number of present points)
          j     = Number of runs of consecutive present points
          info_logNFA = log(NFA) of trajectory, to add to the trajectory infos

        RETURNS:
          TRUE    if the trajectory was not broken
          FALSE   otherwise

*******************************************************************************/

#ifdef ASTRE_HAS_NO_HOLES
char
extract_trajectory_if_possible( int k, int x, int y, int l, double logNFA )
#endif
#ifdef ASTRE_HAS_HOLES
char
extract_trajectory_if_possible( int k,
                                int x, int h, int y,
                                int l, int s, int j,
                                double logNFA )
#endif
{
/*{{{*/
  /* We allocate K points to be safe */
  int* types = (int*)calloc_or_die( K, sizeof(int) );
  ref_point* point_refs = (ref_point*)calloc_or_die( K, sizeof(ref_point) );
  /* Index in the arrays */
  int cur_idx = 0 ;
  /* Last frame of the trajectory, we use it to compute the trajectory length */
  int last_frame = k ;
#if ASTRE_HAS_NO_HOLES
  /* So we can use the same code for both ASTRE_HAS_NO_HOLES and ASTRE_HAS_HOLES */
  const int h = 0 ;
#endif
  /* Is the trajectory broken ? */
  char trajectory_is_broken = FALSE ;
  /* Find the value of the criterion on the trajectory */
  float global_criterion = -1 ;
  {
#ifdef ASTRE_HAS_NO_HOLES
    DEFINE_MIN_l(k);

    int idx_xy = x*N + y ;

    global_criterion = g_fxl[k][idx_xy][l-__min_l] ;
#endif

#ifdef ASTRE_HAS_HOLES
    DEFINE_MAX_h(k);
    DEFINE_MIN_l(k,h);
    DEFINE_MIN_s(k,h,l);
    DEFINE_MIN_j(k,h,l,s);

    int idx_xhy = x*N*(__max_h+1) + h*N + y ;

    global_criterion = g_fxlsj[k][idx_xhy][l-__min_l][s-__min_s][j-__min_j] ;
#endif
  }

  while( TRUE )
  {
    const int p = k-h-1 ;

    DEFINE_MAX_x(k);
    DEFINE_MAX_y(p);

#ifdef ASTRE_HAS_NO_HOLES
    DEFINE_BOUNDS_l(k);
#else
    DEFINE_BOUNDS_l(k,h);
    DEFINE_BOUNDS_s(k,h,l);
    DEFINE_BOUNDS_j(k,h,l,s);
#endif

    char* activatedX = activated_fp[k] ;
    char* activatedY = activated_fp[p] ;

    if( !activatedX[x] || !activatedY[y] )
    {
      C_log_error( "[extract_trajectory_if_possible] Points are expected to be active!" );
      exit( -1 );
    }

    /* If (X,Y) are the ending points, add them and stop extracting */
    if( l == h+2 ) /* h == 0 when ASTRE_HAS_NO_HOLES */
    {
      types[cur_idx] = PRTYPE_REF ;
      point_refs[cur_idx].r = x ;
      cur_idx++ ;

#ifdef ASTRE_HAS_HOLES
      for( int cur_h = 0 ; cur_h < h ; cur_h++ )
      {
        types[cur_idx] = PRTYPE_NONE ;
        cur_idx++ ;
      }
#endif

      types[cur_idx] = PRTYPE_REF ;
      point_refs[cur_idx].r = y ;
      cur_idx++ ;

      break ;
    }

    /* The next l value we are looking for */
    int l_prev = l-h-1 ;

#ifdef ASTRE_HAS_HOLES
    int s_prev = s-1 ;
    int j_prev = j - (h == 0 ? 0 : 1) ;
#endif

#ifdef ASTRE_HAS_NO_HOLES
    float** g_xl_prev = g_fxl[p] ;
#else
    float**** g_xlsj_prev = g_fxlsj[p] ;
#endif

    /* Try to find a possible predecessor having minimal criterion */
    double min_criterion = INFTY ;
    int min_z = -1 ;
    int min_h2 = -1 ;

    float px_X = points[k][x*n_fields+0] ;
    float px_Y = points[k][x*n_fields+1] ;
    float py_X = points[p][y*n_fields+0] ;
    float py_Y = points[p][y*n_fields+1] ;
    float f_h1_p1 = (float)h + 1.0 ;

#ifdef ASTRE_HAS_NO_HOLES
    int idx_y = y*N ;
#else
    DEFINE_MAX_h_prev( p );
    int max_h2 = (j_prev == 1) ? 0 : min_i( __max_h_prev, l_prev-s_prev+2-j_prev );
    int idx_y = y*N*(__max_h_prev+1) ;
#endif

#ifdef ASTRE_HAS_NO_HOLES
    const int q = p-1 ;
#endif
#ifdef ASTRE_HAS_HOLES
    for( int h2 = 0 ; h2 <= max_h2 ; h2++ )
    {
      int idx_yh2 = idx_y + h2*N ;
      const int q = p-1-h2 ;
      float f_h2_p1 = (float)h2 + 1.0 ;
#endif

      DEFINE_MAX_z( q );

#ifdef ASTRE_HAS_NO_HOLES
      DEFINE_MIN_l_prev( p );
#else
      DEFINE_MIN_l_prev( p, h2 );
#endif
      int l0_prev = l_prev - __min_l_prev ;

#ifdef ASTRE_HAS_HOLES
      DEFINE_MIN_s_prev( p, h2, l_prev );
      DEFINE_MIN_j_prev( p, h2, l_prev, s_prev );
      int s0_prev = s_prev - __min_s_prev ;
      int j0_prev = j_prev - __min_j_prev ;
#endif

      char* activatedZ = activated_fp[q] ;
      double* pointsZ = points[q] ;

      for( int z = 0 ; z <= __max_z ; z++ )
      {
        if( !activatedZ[z] ) continue ;

#ifdef ASTRE_HAS_NO_HOLES
        int idx_yz = idx_y + z ;
#else
        int idx_yh2z = idx_yh2 + z ;
#endif

        /* There is no prev_criterion if s <= 2
         * (we only compute the criterion when there are at least 3 points to
         * have an acceleration) */
        float prev_criterion = -1.0 ;

#ifdef ASTRE_HAS_NO_HOLES
        char has_prev_criterion = l > 3 ;
#endif
#ifdef ASTRE_HAS_HOLES
        char has_prev_criterion = s > 3 ;
#endif

        /* Dismiss possibility early */
        if( has_prev_criterion )
        {
#ifdef ASTRE_HAS_NO_HOLES
          prev_criterion = g_xl_prev[idx_yz][l0_prev] ;
#else
          prev_criterion = g_xlsj_prev[idx_yh2z][l0_prev][s0_prev][j0_prev] ;
#endif
          if( prev_criterion >= min_criterion ) continue ;
        }

        /* Otherwise, compute the measure update */
        float pz_X = pointsZ[z*n_fields+0] ;
        float pz_Y = pointsZ[z*n_fields+1] ;

        ASTRE_DEFINE_CRITERION ;

        /* Compute the updated criterion if there is a previous criterion */
        float updated_criterion = -1.0 ;

        if( has_prev_criterion )
        {
        /* Update criterion */
          updated_criterion = max_f( criterion, prev_criterion );
        }
        else
        {
          updated_criterion = criterion ;
        }

        /* Update the minimum found criterion */
        if( updated_criterion < min_criterion )
        {
          min_criterion = updated_criterion ;
          min_z = z ;
#ifdef ASTRE_HAS_HOLES
          min_h2 = h2 ;
#endif
        }
      } /* END foreach( POINT z IN FRAME q ) */
#ifdef ASTRE_HAS_HOLES
    } /* END foreach( HOLE LENGTH h2 ) */
#endif

    /* If we have found a point with a valid maximal acceleration, update
     * the trajectory */
    /* We don't really care about making a crude global_criterion+1 approximation
     * here since:
     * - we keep the minimal min_criterion, and not the first one that satisfies
     *   min_criterion <= global_criterion + EPS
     * - we check in the end that the log(NFA) of the extracted trajectory is not
     *   too far from the one we are trying to obtain
     * - sometime, even a moderate EPS like 1E-2 might cause some floating point
     *   comparison problems, for some unknown reaons. */
    const float EPS = 1 ; // 1E-2 ;
    if( min_criterion <= global_criterion + EPS )
    {
      /* Copy point X and hole h1 */
      types[cur_idx] = PRTYPE_REF ;
      point_refs[cur_idx].r = x ;
      cur_idx++ ;

#ifdef ASTRE_HAS_HOLES
      for( int h_cur = 0 ; h_cur < h ; h_cur++ )
      {
        types[cur_idx] = PRTYPE_NONE ;
        cur_idx++ ;
      }
#endif

      /* Update variables */
      x = y ;
      y = min_z ;
      k = k-h-1 ;

#ifdef ASTRE_HAS_HOLES
      h = min_h2 ;
      s = s_prev ;
      j = j_prev ;
#endif

      l = l_prev ;
    }
    else
    {
      /* Couldn't extend the trajectory */
      trajectory_is_broken = TRUE ;
      break ;
    }
  }

  /* If the trajectory is not broken, add it to the trajectory store if the NFA
   * is not too far from the expected NFA (it might vary because of the EPS slack
   * comparison when trying to extract the correct trajectory) */
  if( !trajectory_is_broken )
  {
    /* Since we stored points in the reverse order, change their positions */
    int* n_types = (int*)calloc_or_die( cur_idx, sizeof(int) );
    ref_point* n_point_refs = (ref_point*)calloc_or_die( cur_idx, sizeof(ref_point) );
    for( int i = 0 ; i < cur_idx ; i++ )
    {
      n_types[i] = types[cur_idx-i-1] ;
      n_point_refs[i] = point_refs[cur_idx-i-1] ;
    }

    int length = cur_idx ;
    int starting_frame = last_frame - length + 1 ;

    /* There is one weakness in the extraction process: since we're comparing
     * floating point values, we need to add a comparison "slack" EPS, and for
     * some features, this might result in a completely different NFA if the
     * slack is too high, so we check again that the NFA of the extracted trajectory
     * is not too far from the predicted NFA */
    double extracted_lNFA = compute_log_NFA_of_trajectory(
        starting_frame, length, n_types, n_point_refs
    );
    if( abs_d( logNFA - extracted_lNFA ) > 1E-2 )
    {
      P(" expected log(NFA) = %g, found log(NFA) = %g (diff: %g)\n", logNFA, extracted_lNFA,
          abs_d(logNFA-extracted_lNFA));
      trajectory_is_broken = TRUE ;
      free( n_types ); n_types = (int*)NULL ;
      free( n_point_refs ); n_point_refs = (ref_point*)NULL ;
    }
    else
    {
    /* If the trajectory is not broken and the NFA is not too far, add it to the store */
      /* Disable points */
      for( int k = 0 ; k < length ; k++ )
      {
        if( n_types[k] == PRTYPE_REF )
        {
          if( activated_fp[starting_frame+k][n_point_refs[k].r] == FALSE )
          {
            C_log_error( "[extract_trajectory_if_possible] Point is not active!" );
            exit(-1);
          }
          activated_fp[starting_frame+k][n_point_refs[k].r] = FALSE ;
        }
      }

      /* Add the log(NFA) to trajectory infos */
      char buf[100] ; sprintf(buf,"%g",logNFA);
      char* data = strdup(buf);

      add_traj( trajectory_store, starting_frame, length, n_types, n_point_refs, (void*)data );

      /* WARNING: we should not free the n_types and n_point_refs arrays since
       * they are referenced by the trajectory store */
    }
  }

  /* Free initial arrays */
  free( types ); types = (int*)NULL ;
  free( point_refs ); point_refs = (ref_point*)NULL ;

  return !trajectory_is_broken ;
/*}}}*/
}

/*******************************************************************************

        Extract the most significant maximal trajectories and deactivate
        their points.

        Returns true if there might possibly be other trajectories that can 
        be extracted, returns false otherwise.

*******************************************************************************/

char
extract_and_disable_most_significant_trajectories ()
{
/*{{{*/
  /* While the trajectory we have encountered are not broken (ie. do not
   * contain points that have been deactivated while trying to extract 
   * another trajectory) we can continue to search for the next minimal NFA
   * and try to extract as many (unbroken) trajectories as possible. */
  char valid_state = TRUE ;

  /* Overview :

   0. while( valid_state )
      {
        1. min <-- min( log NFAs )
        2. if( min > MIN_ALLOWED_LOG_NFA )
           {
              return false ; // stop
           }
        3. foreach( x realizing the min )
           {
              bool is_valid = extract_trajectory_if_possible( x );

              valid_state &= is_valid ;
           }
      }
   4. return true ; // continue

  */

  /* Because of floating-point precision, we may vary dynamically the precision
   * of comparisons if the minimal log(NFA) cannot be found */
  double prec = LOG_NFA_COMP_EPS ;
  char found_the_minimal_lNFA = FALSE ;

  while( valid_state )
  {
    double min_log_NFA = find_minimal_NFA() ;

    if( min_log_NFA > MAX_ALLOWED_LOG_NFA )
    {
      P( " Min log NFA = %g > MAX_LOG_NFA = %g\n", min_log_NFA, MAX_ALLOWED_LOG_NFA );
      P( " All the meaningful trajectories have been extracted!\n" );
      /* Since the state is still valid, we cannot extract trajectories
       * having a log NFA lower than min_log_NFA, so we won't find any
       * new significant trajectories, we can stop the extraction */
      return FALSE ;
    }
    else
    {
      P( " > Min log NFA = %g...\n", min_log_NFA );
    }

    FORALL_k

#ifdef ASTRE_HAS_NO_HOLES
      FORALL_x ; FORALL_y ; FORALL_l
      float delta = *g_l_cur ;
#endif
#ifdef ASTRE_HAS_HOLES
      FORALL_x ; FORALL_h ; FORALL_y
      FORALL_l ; FORALL_s ; FORALL_j
      float delta = *g_j_cur ;
#endif

        if( delta >= INFTY-1 ) continue ; /* No trajectory */

#ifdef ASTRE_HAS_NO_HOLES
        double lNFA = log_NFA( k, delta, l );
#else
        double lNFA = log_NFA( k, delta, l, s, j );
#endif

        if( lNFA <= min_log_NFA + prec )
        {
          /* Reinit precision */
          found_the_minimal_lNFA = TRUE ; prec = LOG_NFA_COMP_EPS ;

#ifdef ASTRE_HAS_NO_HOLES
          char is_valid = extract_trajectory_if_possible( k, x, y, l, lNFA );
#else
          char is_valid = extract_trajectory_if_possible( k, x, h, y, l, s, j, lNFA );
#endif
          /* If trajectory was broken, we switch to invalid state, we can
           * still extract other trajectories having same NFA, but we cannot
           * continue with higher NFAs, since we could find lower NFAs when
           * doing another computation pass. */
          valid_state &= is_valid ;

          if( !is_valid )
          {
            P(" Two trajectories shared a point, a recomputation of the weights is required!\n ");
          }
          else
          {
            P(" Trajectory extracted!\n ");
            /* Since current x and y are now deactivated, we need to find other x, y */
            goto NextXLoop ;
          }
        }

#ifdef ASTRE_HAS_NO_HOLES
      END_FORALL_l ; END_FORALL_y ;
#endif
#ifdef ASTRE_HAS_HOLES
      END_FORALL_j ; END_FORALL_s ; END_FORALL_l
      END_FORALL_y ; END_FORALL_h ;
#endif

NextXLoop: ;      
      END_FORALL_x
    END_FORALL_k

    if( !found_the_minimal_lNFA )
    {
      P(" [!] WARNING: couldn't find the minimal lNFA %g!\n", min_log_NFA );
      prec = prec*10 ;
      P("              setting precision to %g\n", prec );
      if( prec >= 0.01 )
      {
        mini_mwerror( FATAL, 1, "Precision much too low... might be an error\n" );
      }
    }

    found_the_minimal_lNFA = FALSE ;

    P("\n");
  } /* END while( valid_state ) */

  /* The minimal NFA of trajectories that we have checked was greater than
   * the limit, therefore there is possibly other trajectories to extract
   * if we compute the new trajectories NFA for the points that are left */
  return TRUE ;
/*}}}*/
}

/*******************************************************************************

        Compute the most significant trajectories.

        Use a dynamic programming approach for efficiency.

        Brief overview: (TODO)

*******************************************************************************/

void
compute_most_significant_trajectories()
#ifdef ASTRE_HAS_NO_HOLES
{
/*{{{*/
  P( "  -- k = 000 / 000" );
  FORALL_k

    P( "\b\b\b\b\b\b\b\b\b%03d / %03d", k, K-1 ); fflush(stdout) ;
    double* pointsX = points[k] ;

    FORALL_x

      const float px_X = pointsX[x*n_fields+0] ;
      const float px_Y = pointsX[x*n_fields+1] ;

      double* pointsY = points[k-1] ;
      float** g_xl_prev = g_fxl[k-1];

      FORALL_y

        const float py_X = pointsY[y*n_fields+0];
        const float py_Y = pointsY[y*n_fields+1];
        const int idx_y = y*N ;

        /* Reinit the values for (x,y) */
        FORALL_l ;
          *g_l_cur = INFTY ;
        END_FORALL_l

        if( k <= 1 ) continue ;

        DEFINE_BOUNDS_l_prev( p );

        const int q = k-2 ;
        DEFINE_MAX_z( q );
        char* activatedZ = activated_fp[q] ;
        double* pointsZ = points[q] ;

        for( int z = 0 ; z <= __max_z ; z++ )
        {
          if( !activatedZ[z] ) continue ;

          const int idx_yz = idx_y + z ;
          float* g_l_prev = g_xl_prev[idx_yz] ;

          const float pz_X = pointsZ[z*n_fields+0] ;
          const float pz_Y = pointsZ[z*n_fields+1] ;

          ASTRE_DEFINE_CRITERION ;

          /* The iteration here looks a bit cumbersome, because it was written
           * in a way similar to that for the case with holes, where the
           * iteration is more complex. This actually simply loops on all
           * length len from 3 to (k+1), and the check whether the
           * corresponding best trajectory of length len-1 ending on (z,y) and
           * extended by (y,x) is better than the other extensions of length
           * len ending on (y,x). */

          /* Points to G(y,z,k-1,l=3) */
          float* g_l_prev_first = &(g_l_prev[0]);
          /* Points after last G(z,y,k-1,l) */
          float* g_l_prev_last = &(g_l_prev[__size_l0_prev]);

          /* Points to G(x,y,k,l=3) */
          float* g_l_cur = &(g_l[0]);

          /* Len == 3 */
          {
            if( *g_l_cur > criterion ) *g_l_cur = criterion ;
            g_l_cur++ ;
          }

          /* Len > 3 */
          for( float* g_l_prev_cur = g_l_prev_first ;
                      g_l_prev_cur != g_l_prev_last ;
                      g_l_prev_cur++, g_l_cur++ )
          {
            float delta_prev = *g_l_prev_cur ;
            float updated_criterion = max_f( criterion, delta_prev );

            if( *g_l_cur > updated_criterion ) *g_l_cur = updated_criterion ;

          } /* END foreach( LENGTH l ) */
        } /* END foreach( POINT z IN FRAME q ) */

      END_FORALL_y
    END_FORALL_x
  END_FORALL_k

  P("\n");
/*}}}*/
}
#endif
#ifdef ASTRE_HAS_HOLES
{
/*{{{*/
  P( "  -- k = 000 / 000" );
  FORALL_k

    P( "\b\b\b\b\b\b\b\b\b%03d / %03d", k, K-1 ); fflush(stdout) ;
    double* pointsX = points[k] ;

    FORALL_x

      const float px_X = pointsX[x*n_fields+0] ;
      const float px_Y = pointsX[x*n_fields+1] ;

      FORALL_h

        double* pointsY = points[p] ;

        /* constants initialization */
        const float f_h1_p1 = (float)h+1.0 ;
        
        /* if there is a hole (h > 0), the next j we are looking for is cur_j - eps_j */
        const int eps_j = h == 0 ? 0 : 1 ;

        /* the next l we are looking for is cur_l - delta_l */
        const int delta_l = h+1 ;

        float**** g_xlsj_prev = g_fxlsj[p];
        DEFINE_MAX_h_prev( p );

        FORALL_y

          const float py_X = pointsY[y*n_fields+0] ;
          const float py_Y = pointsY[y*n_fields+1] ;
          const int idx_y = y*N*(__max_h_prev+1);

          /* Reinit the values for (x,h,y) */
          FORALL_l ;
            FORALL_s ;
              FORALL_j ;
                *g_j_cur = INFTY ;
              END_FORALL_j ;
            END_FORALL_s ;
          END_FORALL_l

          for( int h2 = 0 ; h2 <= __max_h_prev ; h2++ )
          {
            const int idx_yh2 = idx_y + h2*N ;
            const float f_h2_p1 = (float)h2+1.0 ;

            DEFINE_BOUNDS_l_prev( p, h2 );

            const int q = p-1-h2 ;
            DEFINE_MAX_z( q );
            char* activatedZ = activated_fp[q] ;
            double* pointsZ = points[q] ;

            for( int z = 0 ; z <= __max_z ; z++ )
            {
              if( !activatedZ[z] ) continue ;

              const int idx_yh2z = idx_yh2 + z ;
              float*** g_lsj_prev = g_xlsj_prev[idx_yh2z] ;

              const float pz_X = pointsZ[z*n_fields+0] ;
              const float pz_Y = pointsZ[z*n_fields+1] ;

              ASTRE_DEFINE_CRITERION ;

              /* Criterion initialization */
              {
                int l = k-q+1 ;
                int s = 3 ;
                int j = 1+(h==0?0:1)+(h2==0?0:1) ;

                DEFINE_MIN_l(k,h);
                DEFINE_MIN_s(k,h,l);
                DEFINE_MIN_j(k,h,l,s);

#ifdef ALL_CHECKS
                C_assert( l >= __min_l );
                C_assert( s >= __min_s );
                C_assert( j >= __min_j );
#endif

                float *init = &(g_lsj[l-__min_l][s-__min_s][j-__min_j]);
                if( *init > criterion ) *init = criterion ;
              }

              for( int l0_prev = 0, l_prev = __min_l_prev ; l0_prev < __size_l0_prev ; l0_prev++, l_prev++ )
              {
                float** g_sj_prev = g_lsj_prev[l0_prev];

                DEFINE_BOUNDS_s_prev( p, h2, l_prev );

                const int l = l_prev + delta_l ;
                float** g_sj = g_lsj[l-__min_l] ;

                DEFINE_MIN_s( k, h1, l );

                for( int s0_prev = 0, s_prev = __min_s_prev ; s0_prev < __size_s0_prev ; s0_prev++, s_prev++ )
                {
                  float* g_j_prev = g_sj_prev[s0_prev];

                  DEFINE_BOUNDS_j_prev( p, h2, l_prev, s_prev );

                  float* g_j_prev_first = &(g_j_prev[0]);
                  float* g_j_prev_last = &(g_j_prev[__size_j0_prev]);

                  const int s = s_prev + 1 ;
                  float* g_j = g_sj[s-__min_s] ;
                  DEFINE_MIN_j( k, h, l, s );

                  int j_fst = __min_j_prev + eps_j ; /* eps_j = 1 if there is a hole (h > 0) */
                  float* g_j_cur = &(g_j[j_fst-__min_j]);

                  for( float* g_j_prev_cur = g_j_prev_first ;
                              g_j_prev_cur != g_j_prev_last ;
                              g_j_prev_cur++, g_j_cur++ )
                  {
                    float delta_prev = *g_j_prev_cur ;

                    float updated_criterion = max_f(criterion, delta_prev);

                    if( *g_j_cur > updated_criterion )
                    {
                        *g_j_cur = updated_criterion ;
                    }

                  } /* END foreach( RUNS j ) */
                } /* END foreach( SIZE s ) */
              } /* END foreach( LENGTH l ) */
            } /* END foreach( POINT z IN FRAME q ) */
          } /* END foreach( HOLE LENGTH h2 ) */

        END_FORALL_y
      END_FORALL_h
    END_FORALL_x
  END_FORALL_k
/*}}}*/
}
#endif

void
my_points_desc_save_with_new_trajs( Rawdata raw_out, points_desc pd, trajs_file tf )
{
/*{{{*/
  points_desc pn = points_desc_copy( pd, 1 );

  int n_field = pn->n_fields-1 ;
  pn->tags[n_field] = C_string_dup("t");

  /* Tag trajectories */
  points_desc_set_traj_tags( pn, tf, n_field );

  char **captions = (char**)realloc_or_die(pn->header_captions, (pn->n_headers+tf->num_of_trajs)*sizeof(char*));
  pn->header_captions = captions;
  char **contents =  (char**)realloc_or_die(pn->header_contents, (pn->n_headers+tf->num_of_trajs)*sizeof(char*));
  pn->header_contents = contents;

  for( int i = 0 ; i < tf->num_of_trajs ; i++ )
  {
      char buf[256]; sprintf(buf, "traj:%d:lNFA", i);
      pn->header_captions[pn->n_headers + i] = C_string_dup(buf);
      pn->header_contents[pn->n_headers + i] = C_string_dup(tf->trajs[i].data);
  }
  pn->n_headers += tf->num_of_trajs;

  /* Save points_desc */
  points_desc_save( raw_out, pn );

  /* Free memory */
  points_desc_free_all( &pn );
/*}}}*/
}

/*******************************************************************************

        Detect and extract most significant trajectories.

        This function compute and extract most significant trajectories
        while significant trajectories can be found.

        Function outline:
          1. Initialize the round
          2. Solve for the most significant trajectories
          3. Extract correct most significant trajectories (at least one)
          4. Disable extracted points
          5. Check if new significant trajectories can be found, of if we
            should stop.
         (6) Save the current detection to the 'cur_detect' file to make
           it easier to follow the computation

*******************************************************************************/

void
do_detect()
{
/*{{{*/
  while( TRUE )
  {
    P( " > computing..." ); fflush( stdout );
    compute_most_significant_trajectories() ;
    P( "done!\n" );

    P( " > extracting...\n" );
    char cont = extract_and_disable_most_significant_trajectories() ;
    if( !cont ) break ;

    if( partial_results_fname )
    {
      P( " > saving to temporary file %s...\n", partial_results_fname );
      Rawdata rd = new_rawdata_or_die();
      tf->num_of_trajs = trajectory_store->num_trajs ;
      tf->trajs = trajectory_store->trajs ;
      my_points_desc_save_with_new_trajs( rd, pd, tf );
      save_rawdata( rd, partial_results_fname );
      mw_delete_rawdata( rd ); rd = (Rawdata)NULL ;
    }
  }
/*}}}*/
}

/*******************************************************************************

        Compute the log NFA of a trajectory.

        Using the extended version of acceleration, where the acceleration
        between X -h1-> Y -h2-> Z is (X-Y)/(h1+1) + (Z-Y)/(h2+1)

*******************************************************************************/

/* Helper function */
inline static float __astre_get_point_coordinate
(
  int coord, int k, int type, union u_ref_point* pt
)
{
/*{{{*/
  if( type == PRTYPE_NONE )
  {
    mini_mwerror( FATAL, 1, "[get_point_coordinate] No coordinates for NONE points!" );
  }
  else if( type == PRTYPE_ADD )
  {
    mini_mwerror( FATAL, 1, "[get_point_coordinate] Unimplemented!" );
  }
  else
  {
    int idx = pt->r ;
    return points[k][idx*n_fields+coord] ;
  }
/*}}}*/
}
inline static int __astre_get_point_ref
(
  int k, int type, union u_ref_point* pt
)
{
/*{{{*/
  if( type == PRTYPE_NONE || type == PRTYPE_ADD )
  {
    mini_mwerror( FATAL, 1, "[get_point_ref] No ref!" );
  }
  else
  {
    int idx = pt->r ;
    return idx ;
  }
/*}}}*/
}

/* Replace all interpolated points by holes */
void compute_caracteristics_of_trajectory( int starting_frame, int length, int* type, union u_ref_point* points, float* o_delta, int* o_s, int* o_j )
{
/*{{{*/
  if( type[0] != PRTYPE_REF || type[length-1] != PRTYPE_REF || length < 2 )
  {
    mini_mwerror( FATAL, 1, "[compute_caracteristics_of_trajectory] Trajectory doesn't start and end on points, or has a length < 2!" );
  }

  float max_accel_area = 0.0 ;

  /* Initializes x,h,y */
  int k = starting_frame + length - 1 ;
  float px_X=-1.0f, px_Y=-1.0f, py_X=-1.0f, py_Y=-1.0f, pz_X=-1.0f, pz_Y=-1.0f ;
  int x, y, z ;

  int p ;
  int x_typ = type[k-starting_frame];
  ref_point* x_pt = &(points[k-starting_frame]);
  x = __astre_get_point_ref( k, x_typ, x_pt );
  px_X = __astre_get_point_coordinate( 0, k, x_typ, x_pt );
  px_Y = __astre_get_point_coordinate( 1, k, x_typ, x_pt );
  for( p = k-1 ; p >= starting_frame ; p-- )
  {
    if( type[p-starting_frame] == PRTYPE_REF )
    {

      int y_typ = type[p-starting_frame];
      ref_point* y_pt = &(points[p-starting_frame]);
      y = __astre_get_point_ref( p, y_typ, y_pt );
      py_X = __astre_get_point_coordinate( 0, p, y_typ, y_pt );
      py_Y = __astre_get_point_coordinate( 1, p, y_typ, y_pt );
      break ;
    }
  }
  int size = 2, runs = 1 ;

  while( p >= starting_frame )
  {
    int h1 = k-p-1 ;

    int q ;
    for( q = p-1 ; q >= starting_frame ; q-- )
    {
      if( type[q-starting_frame] == PRTYPE_REF )
      {
        int z_typ = type[q-starting_frame];
        ref_point* z_pt = &(points[q-starting_frame]);
        z = __astre_get_point_ref( q, z_typ, z_pt );
        pz_X = __astre_get_point_coordinate( 0, q, z_typ, z_pt );
        pz_Y = __astre_get_point_coordinate( 1, q, z_typ, z_pt );
        break ;
      }
    }

    if( q >= starting_frame )
    {
      size += 1 ;

      int h2 = p-q-1 ;

#ifdef ASTRE_HAS_NO_HOLES
      if( h1 > 0 || h2 > 0 )
      {
        C_log_error( "[compute_caracteristics_of_trajectory] Trajectory has holes!\n" );
        exit(-1);
      }
#endif

      float f_h1_p1 = (float)h1 + 1.0 ;
      float f_h2_p1 = (float)h2 + 1.0 ;

      /* Has access to (x,k,px), (y,p,py), (z,q,pz), h1, h2 */
      ASTRE_DEFINE_CRITERION ;

      if( (criterion) > max_accel_area ) max_accel_area = (criterion) ;
    }

    if( h1 > 0 ) runs += 1 ;

    /* Next points */
    px_X = py_X ; px_Y = py_Y ;
    py_X = pz_X ; py_Y = pz_Y ;
    k = p ;
    p = q ;
  }

  *o_delta = max_accel_area ;
  *o_s = size ;
  *o_j = runs ;
/*}}}*/
}

/* Replace all interpolated points by holes */
double compute_log_NFA_of_trajectory( int starting_frame, int length, int* type, union u_ref_point* points )
{
  float delta = -1.0 ;
  int s = -1, j = -1 ;
  compute_caracteristics_of_trajectory( starting_frame, length, type, points,
      &delta, &s, &j );
  int kl = starting_frame + length - 1 ;
#ifdef ASTRE_HAS_NO_HOLES
  double lNFA = log_NFA( kl, delta, length );
#else
  double lNFA = log_NFA( kl, delta, length, s, j );
#endif

  return lNFA ;
}

void info_on_traj( char* str )
{
  traj* tt = read_trajectory_descriptor( str );

  float delta ;
  int s, j ;
  compute_caracteristics_of_trajectory( tt->starting_frame, tt->length, tt->type, tt->points,
      &delta, &s, &j );
  int kl = tt->starting_frame + tt->length - 1 ;
#ifdef ASTRE_HAS_NO_HOLES
  double lNFA = log_NFA( kl, delta, tt->length );
#else
  double lNFA = log_NFA( kl, delta, tt->length, s, j );
#endif

  P(" traj: lNFA(%g)  start(%d) delta(%g) len(%d) size(%d) runs(%d)\n",
      lNFA, tt->starting_frame, delta, tt->length, s, j );

  free( tt->type );
  free( tt->points );
  free( tt );
}

/*******************************************************************************

        Restart computation from trajectories

        Take a description of trajectories and add them to the store,
        recompute their NFA and deactivate their points, prior to solving the
        trajectory detection problem.

*******************************************************************************/
void
astre__restart_trajectories( points_desc restart )
{
  if( restart->uid != pd->uid )
  {
    C_log_error( "Restart file UID does not match Pointsdesc file UID!\n" );
    exit(-1);
  }

  if( restart->n_fields < 3 )
  {
    C_log_error( "Restart file has no trajectory field!\n" );
    exit(-1);
  }

  trajs_file rf = points_desc_extract_trajs( restart, -1, FALSE );

  for( int k = 0 ; k < rf->num_of_trajs ; k++ )
  {
    traj* tt = &(rf->trajs[k]);
    double lNFA = compute_log_NFA_of_trajectory( tt->starting_frame, tt->length, tt->type, tt->points );
    char buf[256]; sprintf(buf, "%g", lNFA);
    tt->data = strdup(buf);
    add_traj( trajectory_store, tt->starting_frame, tt->length, tt->type, tt->points, tt->data );
    /* Deactivate points */
    for( int p = 0 ; p < tt->length ; p++ )
    {
      if( tt->type[p] == PRTYPE_REF )
      {
        if( !activated_fp[p+tt->starting_frame][tt->points[p].r] )
        {
          C_log_error(
              "Error while loading restart trajectories: "
              "point already deactivated! Trajectories share a point?"
          );
          exit(-1);
        }
        activated_fp[p+tt->starting_frame][tt->points[p].r] = FALSE ;
      }
#ifdef ASTRE_HAS_NO_HOLES
      else
      {
        C_log_warning( "Restarting from a trajectory containing holes!\n" );
      }
#endif
    }

    /* We do not copy the array but move them to the trajectory list, so we should
     * not free them afterwards */
    tt->points = (ref_point*)NULL ;
    tt->type = (int*)NULL ;
    tt->data = (void*)NULL ;
  }

  trajs_file_free_all( &rf );
}

/*******************************************************************************

        Main ASTRE function

        i_pd   : Input Pointsdesc
        o_pd   : Output Pointsdesc, with an additional column for found trajectories
        i_e    : Maximal allowed value of log(NFA)
        i_h    : Maximal allowed length of a hole (-1: any length)
        r_pd   : Partial Pointsdesc to resume from, or NULL
        partial_fname : File where we save partial computations, or NULL
        just_tag_trajectories : tag trajectories with their NFA and exit
        auto_crop : crop each image to its bounding-box
        parameters : optionnal parameters defined by each algorithm

*******************************************************************************/
void
astre
(
    Rawdata i_pd,
    Rawdata o_pd,
    float i_e,
    int i_h,
    Rawdata r_pd,
    char* partial_fname,
    char just_tag_trajectories,
    char auto_crop,
    astre_parameters parameters
)
{
  pd = points_desc_load ( i_pd ) ;
  tf = trajs_file_new() ;

  if( pd->n_frames < 3 )
  {
    C_log_error("Not enough frames available for a trajectory search!\n");
    exit(-1);
  }

  MAX_ALLOWED_LOG_NFA = i_e ;

  if( MAX_ALLOWED_TRAJECTORY_LENGTH == 0 )
    MAX_ALLOWED_TRAJECTORY_LENGTH = pd->n_frames ;

#ifdef ASTRE_HAS_HOLES
  MAX_ALLOWED_HOLE_LENGTH = i_h ;
  if( MAX_ALLOWED_HOLE_LENGTH < 0 )
    MAX_ALLOWED_HOLE_LENGTH = pd->n_frames-3 ;
#endif

  partial_results_fname = partial_fname ;

  P( " ------------------------------------------------\n");
  P( "  MAXIMAL log(NFA) = %g\n", MAX_ALLOWED_LOG_NFA );
  P( "  MAXIMAL TRAJECTORY LENGTH = %d\n", MAX_ALLOWED_TRAJECTORY_LENGTH );
#ifdef ASTRE_HAS_HOLES
  P( "  MAXIMAL HOLE LENGTH = %d\n", MAX_ALLOWED_HOLE_LENGTH );
#endif
  P( " ------------------------------------------------\n");

#ifdef ALL_CHECKS
  P( "\n\n" );
  P( " ------------------------------------------------\n");
  P( "  WARNING: ALL_CHECKS is set!\n" );
  P( " ------------------------------------------------\n");
#endif

  /*                     Initialize the algorithm variables */
  /* ------------------------------------------------------ */
  P( " > Initialization...\n" );

  n_points_in_frame = pd->n_points_in_frame ;
  points = pd->points ;
  n_fields = pd->n_fields ;

  K = pd->n_frames ;
  P( " > Number of frames K = %d\n", K );

  N = 0 ;
  for( int k = 0 ; k < K ; k++ ) N = max_ui( N, n_points_in_frame[k] );
  P( " > Maximal number of points N = %d\n", N );

  int nr = pd->height ;
  int nc = pd->width ;
  precompute_image_areas(nr*nc, auto_crop);

  /* Precomputed values */
  LOG_K = log10(K);
  LOG_N = log10(N);
  precompute_log_k();
  precompute_log_cnk();
  precompute_log_kfact();
  precompute_log_nprod();
  discrete_area_init( 50 );

  activated_fp_init();
  traj_store_init(200); /* Allocate a trajectory store of 200 trajectories */

  ASTRE__INITIALIZATION ;

  if( just_tag_trajectories )
  {
    astre__restart_trajectories( pd );
    goto astre__SaveTrajectories ;
  }

#ifdef ASTRE_HAS_NO_HOLES
  DEFINE_MAX_k ;
  g_fxl = (float***)calloc_or_die( __max_k+1, sizeof(float**) );
  g_fxl[0] = (float**)NULL ;
  for( int k = 1 ; k <= __max_k ; k++ )
  {
    DEFINE_MAX_x(k);
    const int p = k-1 ;
    DEFINE_MAX_y(p);
    DEFINE_BOUNDS_l(k);

    g_fxl[k] =
      (float**)calloc_or_die( N*N, sizeof(float*) );

    for( int x = 0 ; x <= __max_x ; x++ )
    {
      int x_idx = x*N ;

      for( int y = 0 ; y <= __max_y ; y++ )
      {
        int xy_idx = x_idx + y ;

        g_fxl[k][xy_idx] =
          (float*)calloc_or_die( __size_l0, sizeof(float) );
      }
    }
  }
#endif
#ifdef ASTRE_HAS_HOLES
  DEFINE_MAX_k ;
  g_fxlsj = (float*****)calloc_or_die( __max_k+1, sizeof(float****) );
  g_fxlsj[0] = (float****)NULL ;
  for( int k = 1 ; k <= __max_k ; k++ )
  {
    DEFINE_MAX_x(k);
    DEFINE_MAX_h(k);

    g_fxlsj[k] =
      (float****)calloc_or_die( N*N*(__max_h+1), sizeof(float***) );

    for( int x = 0 ; x <= __max_x ; x++ )
    {
      int x_idx = x*N*(__max_h+1) ;

      for( int h = 0 ; h <= __max_h ; h++ )
      {
        int xh_idx = x_idx + h*N ;

        const int p = k-h-1 ;
        DEFINE_MAX_y(p);
        DEFINE_BOUNDS_l(k,h);

        for( int y = 0 ; y <= __max_y ; y++ )
        {
          int xhy_idx = xh_idx + y ;

          g_fxlsj[k][xhy_idx] =
            (float***)calloc_or_die( __size_l0, sizeof(float**) );

          for( int l0 = 0, l = __min_l ; l0 < __size_l0 ; l0++, l++ )
          {
            DEFINE_BOUNDS_s( k, h, l );

            g_fxlsj[k][xhy_idx][l0] =
              (float**)calloc_or_die( __size_s0, sizeof(float*) );

            for( int s0 = 0, s = __min_s ; s0 < __size_s0 ; s0++, s++ )
            {
              DEFINE_BOUNDS_j( k, h, l, s );

              g_fxlsj[k][xhy_idx][l0][s0] =
                (float*)calloc_or_die( __size_j0, sizeof(float) );
            }
          }
        }
      }
    }
  }
#endif

  /* Restart */
  if( r_pd )
  {
    points_desc restart = points_desc_load( r_pd );
    astre__restart_trajectories( restart );
    points_desc_free_all( &restart );
  }

  /*                  Run the dynamic programming algorithm */
  /* ------------------------------------------------------ */
#ifdef ASTRE__SHOW_INFORMATIONS
    /* If we only want to display some informations
     * about certain trajectories */
  ASTRE__SHOW_INFORMATIONS ;
#else
  do_detect();
#endif

  /*                                            Free memory */
  /* ------------------------------------------------------ */
  /* TODO: free G array */

  /*                                  Save the trajectories */
  /* ------------------------------------------------------ */
astre__SaveTrajectories:
  tf->num_of_trajs = trajectory_store->num_trajs ;
  tf->trajs = trajectory_store->trajs ;
  my_points_desc_save_with_new_trajs( o_pd, pd, tf );
  trajectory_store->allocated_trajs = 0 ;
  trajectory_store->num_trajs = 0 ;
  trajectory_store->trajs = (traj*)NULL ;

  for( int i = 0 ; i < tf->num_of_trajs ; i++ )
  {
      free( tf->trajs[i].data );
  }

  /*                                            Free memory */
  /* ------------------------------------------------------ */
  ASTRE__DEINITIALIZATION ;
  free_image_areas();
  free( trajectory_store );
  discrete_area_free();
  activated_fp_free();
  free( LOG_k ); LOG_k = (double*)NULL ;
  free( LOG_Cnk ); LOG_Cnk = (double*)NULL ;
  free( LOG_Kfact ); LOG_Kfact = (double*)NULL ;
  log_nprod_free();
  points_desc_free_all( &pd );
  trajs_file_free_all( &tf );
}

/*******************************************************************************

        Command-line parsing

*******************************************************************************/
int main( int ARGC, char** ARGV )
{
  arg_parser ap = arg_parser_new();

  arg_parser_set_info( ap, main__help_msg );

  struct arg_str *p_in = arg_str1( NULL, NULL, "in", "Input points file" );
  arg_parser_add( ap, p_in );

  struct arg_str *p_out = arg_str1( NULL, NULL, "out", "Output points file" );
  arg_parser_add( ap, p_out );

  struct arg_dbl *p_e = arg_dbl0( "e", "epsilon", "<e>",
      "Maximal allowed log(NFA) (default: 0)" );
  if( p_e ) p_e->dval[0] = 0.0 ;
  arg_parser_add( ap, p_e );

  astre_parameters parameters ;

#ifdef ASTRE_HAS_HOLES
  struct arg_int *p_h = arg_int0( "h", "max-hole-length", "<h>", "Maximal allowed hole length (default: -1, any length)" );
  if( p_h ) p_h->ival[0] = -1 ;
  arg_parser_add( ap, p_h );
#endif

  struct arg_str *p_r = arg_str0( "r", "restart", "<r>",
      "Restart trajectory detection from partial detections "
      "(trajectories are assumed to be in the last column)" );
  arg_parser_add( ap, p_r );

  struct arg_str *p_s = arg_str0( "s", "save-partial", "<s>", "Save partial detections" );
  arg_parser_add( ap, p_s );


  struct arg_lit *p_N = arg_lit0( NULL, "tag-NFA",
      "Tag the trajectories in file <in> with their NFA and quit" );
  arg_parser_add( ap, p_N );

  struct arg_lit *p_c = arg_lit0( NULL, "auto-crop", "Auto-crop images to their bounding-box" );
  arg_parser_add( ap, p_c );

  MAIN__DEFINE_ARGUMENTS ;

  /* Handle arguments */
  arg_parser_handle( ap, ARGC, ARGV );

  /* Check arguments */
  char* in = (char*)p_in->sval[0]; C_assert( in && strlen(in) > 0 );
  char* out = (char*)p_out->sval[0]; C_assert( out && strlen(out) > 0 );
  double e = p_e->dval[0];
#ifdef ASTRE_HAS_HOLES
  int h = p_h->ival[0];
#else
  int h = 0 ;
#endif

  Rawdata rd_in = load_rawdata( in );
  Rawdata rd_out = new_rawdata_or_die();

  Rawdata rd_restart = (Rawdata)NULL ;
  if( p_r->count > 0 )
    rd_restart = load_rawdata( (char*)p_r->sval[0] );

  char just_tag_trajectories = p_N->count > 0 ;
  char crop = p_c->count > 0 ;

  char* save_partial = (char*)NULL ;
  if( p_s->count > 0 )
    save_partial = (char*)p_s->sval[0] ;
  C_assert( !save_partial || strlen(save_partial) > 0 );

  MAIN__VERIFY_ARGUMENTS ;

  astre( rd_in, rd_out,
           e, h,
           rd_restart, save_partial,
           just_tag_trajectories, crop,
           parameters
  );

  save_rawdata( rd_out, out );

  MAIN__AFTER_PROCESSING ;

  mw_delete_rawdata( rd_in );
  mw_delete_rawdata( rd_out );
  if( rd_restart ) mw_delete_rawdata( rd_restart );

  /* Clean memory */
  arg_parser_free_all( &ap );
}
