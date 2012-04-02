# encoding: utf-8

import numpy as np

"""
        Computes log k, k = 0 .. K

        RETURNS:
          store[0] = -1.0
          store[k] = log k (k > 0)
"""
def precompute_log_K(K):
  store = np.empty(K+1)
  for k in range(K+1):
    store[k] = -1.0 if k == 0 else np.log10(k)
  return store

"""
        Computes log k!, k = 0 .. K

        RETURNS:
          store[k] = log k!
"""
def precompute_log_Kfact(K):
  store = np.empty(K+1)
  for k in range(K+1):
    store[k] = 0.0 if k == 0 else store[k-1] + np.log10(k)
  return store

"""
        Computes ln k!, k = 0 .. K

        RETURNS:
          store[k] = ln k!
"""
def precompute_ln_Kfact(K):
  store = np.empty(K+1)
  for k in range(K+1):
    store[k] = 0.0 if k == 0 else store[k-1] + np.log(k)
  return store

"""
        Computes log C( N, k ), k = 0 .. N

        RETURNS:
          store[k] = log C( N, k )
"""
def precompute_log_CNk(N):
  store = np.empty(N+1)
  LOG_kfact = precompute_log_Kfact(N)
  max_k = N/2
  # Compute log C(N,k)
  for k in range(max_k+1):
    store[k] = LOG_kfact[N] - LOG_kfact[N-k] - LOG_kfact[k]

  # Mirror all computations, since comb( N, k ) = comb( N, N-k )
  for k in range((N+1)/2):
    store[N-k] = store[k]

  del LOG_kfact

  return store

"""
        Compute log C(n,k), n = 0 .. N
                            k = 0 .. n

        RETURNS:
          store[n, k] = log C(n,k)
"""
def precompute_log_Cnk(N):
  store = np.empty((N+1,N+1))
  LOG_kfact = precompute_log_Kfact(N)

  # Compute log C(n,k)
  for n in range(N+1):
    max_k = n/2

    for k in range(max_k+1):
      store[n,k] = LOG_kfact[n] - LOG_kfact[n-k] - LOG_kfact[k]

    # Mirror all computations, since comb( n, k ) = comb( n, n-k )
    for k in range((n+1)/2):
      store[n, n-k] = store[n, k]

  del LOG_kfact

  return store
