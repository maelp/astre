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

#define ALL_CHECKS                              /* undef to remove asserts checkings */
#undef ALL_CHECKS

#include "astre-common-defs.h"

/*******************************************************************************

        Global variables declarations

*******************************************************************************/

// int my_astre_variable1 ;

/*******************************************************************************

        NFA and criterion computation function

*******************************************************************************/

#ifdef ASTRE_HAS_NO_HOLES
static inline float
criterion__define(
  float px_X, float px_Y,
  float py_X, float py_Y,
  float pz_X, float pz_Y,
  int q
)
{
  float v_accel_X = abs_f(px_X + pz_X - 2.0*py_X);
  float v_accel_Y = abs_f(px_Y + pz_Y - 2.0*py_Y);
  int last_accel_X = (int)(v_accel_X + 0.5f);
  int last_accel_Y = (int)(v_accel_Y + 0.5f);
  float criterion = discrete_area( last_accel_X, last_accel_Y );
  return criterion / IMAGE_AREA[q] ;
}
#define ASTRE_DEFINE_CRITERION \
  float criterion = criterion__define( \
      px_X, px_Y, py_X, py_Y, pz_X, pz_Y, q \
  );
#endif
#ifdef ASTRE_HAS_HOLES
static inline float
criterion__define(
  float px_X, float px_Y,
  float py_X, float py_Y,
  float pz_X, float pz_Y,
  float f_h1_p1, float f_h2_p1,
  int q
)
{
  float v_accel_X = abs_f((px_X-py_X)/f_h1_p1 + (pz_X-py_X)/f_h2_p1) ;
  float v_accel_Y = abs_f((px_Y-py_Y)/f_h1_p1 + (pz_Y-py_Y)/f_h2_p1) ;
  int last_accel_X = (int)(v_accel_X + 0.5f);
  int last_accel_Y = (int)(v_accel_Y + 0.5f);
  float criterion = discrete_area( last_accel_X, last_accel_Y );
  return criterion / IMAGE_AREA[q] ;
}
#define ASTRE_DEFINE_CRITERION \
  float criterion = criterion__define( \
      px_X, px_Y, py_X, py_Y, pz_X, pz_Y, \
      f_h1_p1, f_h2_p1, q \
  );
#endif

/*******************************************************************************

        Log NFA computation.

        PARAMETERS:
          kl  = ending frame
          a   = (pi.delta^2)

          l   = length of the trajectory (trajectory span)

          s   = size of the trajectory (number of present points)

          j   = number of runs of consecutive present points
              = (number_of_holes + 1)

        RETURNS: log(NFA)

*******************************************************************************/

#ifdef ASTRE_HAS_NO_HOLES
#define log_NFA( kl, a, l ) log_NFA_l( kl, a, l )
static inline double
log_NFA_l( int kl, float a, int l )
{
/*{{{*/
  int k0 = kl - l + 1 ;

  double dl = (double)l ;

  double lnprod = LOG_Nprod[k0][l][l] ;
#ifdef ALL_CHECKS
  if( lnprod < 0 ) /* undefined log_nprod, might not happen */
    mini_mwerror( FATAL, 1, "[log_NFA] internal error\n" );
#endif

  double l_NFA =
    LOG_k[K] + LOG_k[K-l+1] + lnprod +
      (dl - 2.0)*log10((double)a) ;

  return l_NFA ;
/*}}}*/
}
#endif

#ifdef ASTRE_HAS_HOLES
#define log_NFA( kl, a, l, s, j ) log_NFA_ls( kl, a, l, s, j )
static inline double
log_NFA_ls( int kl, float a, int l, int s, int j )
{
/*{{{*/
#ifdef ALL_CHECKS
  if( s < 2 ) mini_mwerror( FATAL, 1, "[log_NFA] s < 2\n" );
  if( s < j ) mini_mwerror( FATAL, 1, "[log_NFA] s < j\n" );
  if( l-s+1 < j ) mini_mwerror( FATAL, 1, "[log_NFA] l-s+1 < j\n" );
  if( j < 1 ) mini_mwerror( FATAL, 1, "[log_NFA] j < 1\n" );
#endif

  /* First image of the trajectory */
  int k0 = kl-l+1 ;

  double ds = (double)s ;

  if( j > 1 )
  {
    double dp = (double)(j-1) ;
    /* the maximum probability is attained when all the holes are of equal length */
    double h = (double)(l - s)/dp ;
    double dhh = ((h+1.0)*(h+1.0));

    double lnprod = LOG_Nprod[k0][l][s] ;
#ifdef ALL_CHECKS
    if( lnprod < 0 ) /* undefined log_prod, might not happen */
      mini_mwerror( FATAL, 1, "[log_NFA] internal error\n" );
#endif

    double l_NFA =
      LOG_K + LOG_k[l] + LOG_k[K-l+1] + LOG_Cnk[l*(K+1)+s] +
      lnprod + (ds - 2.0)*log10((double)a) + dp*log10(dhh) ;

    return l_NFA ;
  }
  else
  {
    double lnprod = LOG_Nprod[k0][l][l] ;
#ifdef ALL_CHECKS
    if( lnprod < 0 ) /* undefined log_prod, might not happen */
      mini_mwerror( FATAL, 1, "[log_NFA] internal error\n" );
#endif

    double l_NFA =
      LOG_K + LOG_k[l] + LOG_k[K-l+1] + /* LOG_Cnk[l*(K+1)+s] = 1 since l = s (j = 1)*/
      lnprod + (ds - 2.0)*log10((double)a) ;

    return l_NFA ;
  }
/*}}}*/
}
#endif

/*******************************************************************************

        Find the NFA of the most significant trajectory.

        find_minimal_nfa

*******************************************************************************/
#ifdef ASTRE_HAS_NO_HOLES
#define FIND_MINIMAL_NFA__COMPUTE_NFA(k,x,y,l,delta) \
  const double lNFA = log_NFA( k, delta, l )
#endif

#ifdef ASTRE_HAS_HOLES
#define FIND_MINIMAL_NFA__COMPUTE_NFA(k,x,y,l,s,j,delta) \
  const double lNFA = log_NFA( k, delta, l, s, j );
#endif

/*******************************************************************************

        Astre function

*******************************************************************************/

#define ASTRE__INITIALIZATION astre__initialization();
void astre__initialization()
{
  // ...
  // my_astre_variable1 = 10 ;
}

#define ASTRE__DEINITIALIZATION astre__deinitialization();
void astre__deinitialization()
{
  // ...
}

#define ASTRE__SHOW_INFORMATIONS astre__show_informations();
void astre__show_informations()
{
  info_on_traj(
    "S0 P14 P15 P0 P13 P17 P12 P8 P2 P15 P13 P2 P5 P5 P7 P13 P13 P6 P2 P9 P1 ;"
  );

  info_on_traj(
    "S5 P12 P8 P2 P15 P13 P2 P5 P5 P7 P13 P13 P6 P2 P9 P1 ;"
  );
}
#undef ASTRE__SHOW_INFORMATIONS

/*******************************************************************************

        Main function

*******************************************************************************/
char *main__help_msg =
#ifdef ASTRE_HAS_NO_HOLES
    "ASTRE (no holes)\n"
#else
    "ASTRE (with holes)\n"
#endif
"Detect trajectories in a noise point clouds sequence.\n" ;

typedef struct st_astre_parameters {
  // int param1 ;
} astre_parameters ;

#define MAIN__DEFINE_ARGUMENTS \
  // Define command-line arguments

#define MAIN__VERIFY_ARGUMENTS \
  // Verify the values of your command-line arguments
  // Store them in the parameter struct

#define MAIN__AFTER_PROCESSING \
  // Save files if needed
  // Clean-up memory

/*******************************************************************************

        Generic ASTRE code

*******************************************************************************/
#include "astre-common-code.h"

