#ifndef _VISION_MATH_COMBINATORICS_H
#define _VISION_MATH_COMBINATORICS_H

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
double* combinatorics_log_K_init( int K );
void combinatorics_log_K_fill_store( double* store, int K );

/*******************************************************************************

        Computes log k!, k = 0 .. K

        RETURNS:
          store[k] = log k!

*******************************************************************************/
double* combinatorics_log_Kfact_init( int K );
void combinatorics_log_Kfact_fill_store( double* store, int K );

/*******************************************************************************

        Computes log C( N, k ), k = 0 .. N

        RETURNS:
          store[k] = log C( N, k )

*******************************************************************************/
double* combinatorics_log_CNk_init( int N );
void combinatorics_log_CNk_fill_store( double* store, int N );

/*******************************************************************************

        Compute log C(n,k), n = 0 .. N
                            k = 0 .. n

        RETURNS:
          store[n*(N+1)+k] = log C(n,k)

*******************************************************************************/
double* combinatorics_log_Cnk_init( int N );
void combinatorics_log_Cnk_fill_store( double* store, int N );

/*******************************************************************************

        Compute log C(n,k), n = 0 .. N
                            k = 0 .. n

        RETURNS:
          store[n][k] = log C(n,k)

*******************************************************************************/
double** combinatorics_log_Cnk_2_init( int N );
void combinatorics_log_Cnk_2_fill_store( double** store, int N );
void combinatorics_log_Cnk_2_free( double** store, int N );

#endif
