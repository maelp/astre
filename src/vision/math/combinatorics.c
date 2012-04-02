#include <vision/core.h>
#include <vision/math/combinatorics.h>

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

/*******************************************************************************

        Computes log k, k = 0 .. K

        RETURNS:
          store[0] = -1.0
          store[k] = log k (k > 0)

*******************************************************************************/
double*
combinatorics_log_K_init( int K )
{
  double* store = (double*)calloc_or_die( K+1, sizeof(double) );
  combinatorics_log_K_fill_store( store, K );
  return store ;
}

/* store: array of K+1 doubles */
void
combinatorics_log_K_fill_store( double *store, int K )
{
  for( int k = 0 ; k < K+1 ; k++ )
  {
    store[k] = (k == 0)? -1 : log10( (double)k );
  }
}

/*******************************************************************************

        Computes log k!, k = 0 .. K

        RETURNS:
          store[k] = log k!

*******************************************************************************/
double*
combinatorics_log_Kfact_init( int K )
{
  double* store = (double*)calloc_or_die( K+1, sizeof(double) );
  combinatorics_log_Kfact_fill_store( store, K );
  return store ;
}

/* store: array of K+1 doubles */
void
combinatorics_log_Kfact_fill_store( double *store, int K )
{
  for( int k = 0 ; k < K+1 ; k++ )
  {
    store[k] = (k <= 1)? 0.0 : store[k-1] + log10( (double)k );
  }
}

/*******************************************************************************

        Computes log C( N, k ), k = 0 .. N

        RETURNS:
          store[k] = log C( N, k )

*******************************************************************************/
double*
combinatorics_log_CNk_init( int N )
{
  double* store = (double*)calloc_or_die( N+1, sizeof(double) );
  combinatorics_log_CNk_fill_store( store, N );
  return store ;
}

/* store: array of N+1 doubles */
void
combinatorics_log_CNk_fill_store( double *store, int N )
{
/*{{{*/
  for( int k = 0 ; k <= N ; k++ )
    store[k] = -1.0 ;

  double* LOG_kfact = combinatorics_log_Kfact_init( N );

  int max_k = N/2 ;

  /* Compute log C(N,k) */
  for( int k = 0 ; k <= max_k ; k++ )
  {
    store[ k ] = LOG_kfact[N] - LOG_kfact[N-k] - LOG_kfact[k] ;
  }

  /* Mirror all computations, since comb( N, k ) = comb( N, N-k ) */
  for( int k = 0 ; k < (N+1)/2 ; k++ )
  {
    store[ N-k ] = store[ k ];
  }

  free( LOG_kfact ); LOG_kfact = (double*)NULL ;
/*}}}*/
}

/*******************************************************************************

        Compute log C(n,k), n = 0 .. N
                            k = 0 .. n

        RETURNS:
          store[n*(N+1)+k] = log C(n,k)

*******************************************************************************/
double*
combinatorics_log_Cnk_init( int N )
{
  double* store = (double*)calloc_or_die( (N+1)*(N+1), sizeof(double) );
  combinatorics_log_Cnk_fill_store( store, N );
  return store ;
}

/* store: array of (N+1)*(N+1) doubles */
void
combinatorics_log_Cnk_fill_store( double* store, int N )
{
/*{{{*/
  for( int n = 0 ; n <= N ; n++ )
  for( int k = 0 ; k <= N ; k++ )
    store[n*(N+1)+k] = -1.0 ;

  double* LOG_kfact = combinatorics_log_Kfact_init( N );

  /* Compute log C(n,k) */
  for( int n = 0 ; n <= N ; n++ )
  {
    int max_k = n/2 ;

    /* Compute log C(n,k) */
    for( int k = 0 ; k <= max_k ; k++ )
    {
      store[ n*(N+1) + k ] = LOG_kfact[n] - LOG_kfact[n-k] - LOG_kfact[k] ;
    }

    /* Mirror all computations, since comb( n, k ) = comb( n, n-k ) */
    for( int k = 0 ; k < (n+1)/2 ; k++ )
    {
      store[ n*(N+1) + n-k ] = store[ n*(N+1) + k ];
    }
  }

  free( LOG_kfact ); LOG_kfact = (double*)NULL ;
/*}}}*/
}

/*******************************************************************************

        Compute log C(n,k), n = 0 .. N
                            k = 0 .. n

        RETURNS:
          store[n][k] = log C(n,k)

*******************************************************************************/
double**
combinatorics_log_Cnk_2_init( int N )
{
  double** store = (double**)calloc_or_die( (N+1), sizeof(double*) );
  combinatorics_log_Cnk_2_fill_store( store, N );
  return store ;
}

/* store: array of (N+1) pointers to doubles */
void
combinatorics_log_Cnk_2_fill_store( double** store, int N )
{
/*{{{*/
  for( int n = 0 ; n <= N ; n++ )
  {
    store[n] = (double*)calloc_or_die( (n+1), sizeof(double) );
  }

  for( int n = 0 ; n <= N ; n++ )
  for( int k = 0 ; k <= n ; k++ )
    store[n][k] = -1.0 ;

  double* LOG_kfact = combinatorics_log_Kfact_init( N );

  /* Compute log C(n,k) */
  for( int n = 0 ; n <= N ; n++ )
  {
    int max_k = n/2 ;

    /* Compute log C(n,k) */
    for( int k = 0 ; k <= max_k ; k++ )
    {
      store[n][k] = LOG_kfact[n] - LOG_kfact[n-k] - LOG_kfact[k] ;
    }

    /* Mirror all computations, since comb( n, k ) = comb( n, n-k ) */
    for( int k = 0 ; k < (n+1)/2 ; k++ )
    {
      store[n][n-k] = store[n][k];
    }
  }

  free( LOG_kfact ); LOG_kfact = (double*)NULL ;
/*}}}*/
}

/* Free the store */
void
combinatorics_log_Cnk_2_free( double** store, int N )
{
  for( int n = 0 ; n <= N ; n++ )
  {
    free( store[n] ); store[n] = (double*)NULL ;
  }
  free( store );
}
