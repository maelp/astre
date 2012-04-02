#!/usr/bin/env python
# encoding: utf-8

#   ASTRE a-contrario single trajectory extraction
#   Copyright (C) 2011 Mael Primet (mael.primet AT gmail.com)
# 
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
# 
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
# 
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
from numpy import Inf, log10, minimum, maximum
import pymage.math.combinatorics as combinatorics
from pymage.trajs.pointsdesc import PointsDesc
from pymage.utils.discrete_area import DiscreteArea

# Naive implementation of the ASTRE algorithms
#
# This should be regarded as a reference implementation, and you should rather
# use the efficient astre-noholes and astre-holes C implementations
#
# Input: a PointsDesc file containing at least two images
# Output: a PointsDesc file where the assignments between the two first images
#         are stored as a label in the last column (-1: the point doesn't
#         belong to any pairing)

discrete_area = DiscreteArea(50)
def a_d(x, y):
    ix = int(abs(x)+0.5)
    iy = int(abs(y)+0.5)
    return discrete_area.area(ix, iy)

class FoundTrajectoryException(Exception):
    def __init__(self, traj, lNFA):
        self.traj = traj
        self.lNFA = lNFA

class NoholesSolver(object):
#{{{
    def __init__(self, image_size, frames):
        self.frames = frames
        self.K = K = len(frames)
        self.log_k = combinatorics.precompute_log_K(K)

        self.image_size = image_size
        self.image_area = image_area = points.width*points.height
        self.log_image_area = log10(image_area)

        N = max([len(frames[k]) for k in xrange(K)])
        self.g = np.ones((K, N, N, K), dtype=np.float64)*Inf
        self.activated = np.ones((K, N), dtype=bool)

        self.log_nprod = np.zeros((K, K), dtype=np.float64)
        self._precompute_log_nprod()

    def _precompute_log_nprod(self):
        """
        log_nprod[k0][l0] = N_{k0-l0} * ... * N_{k0}
        where N_i = len(frames[i])
        """
        frames = self.frames
        log_nprod = self.log_nprod
        K = self.K
        for k0 in xrange(K):
            for l0 in xrange(k0+1):
                if l0 == 0:
                    log_nprod[k0, l0] = log10(len(frames[k0]))
                else:
                    log_nprod[k0, l0] = log10(len(frames[k0])) + log_nprod[k0-1, l0-1]

    def _compute_g(self):
        frames = self.frames
        K = self.K
        g, activated = self.g, self.activated

        trajectory_found = False

        for k0 in xrange(1,K):
            fx = frames[k0]
            fy = frames[k0-1]
            if k0 >= 2:
                fz = frames[k0-2]
            for ix, x in enumerate(fx):
                if not(activated[k0, ix]): continue
                x0, x1 = x[0], x[1]

                for iy, y in enumerate(fy):
                    if not(activated[k0-1, iy]): continue
                    g[k0, ix, iy, 1] = 1.0 # l0 = 1 means l = 2
                    y0, y1 = y[0], y[1]
                    m0, m1 = x0 - 2.0*y0, x1 - 2.0*y1

                    for l0 in xrange(2, k0+1): # l = 3 to k
                        gmin = Inf

                        for iz, z in enumerate(fz):
                            if not(activated[k0-2, iz]): continue
                            z0, z1 = z[0], z[1]

                            a = maximum(a_d(m0 + z0, m1 + z1), g[k0-1, iy, iz, l0-1])
                            gmin = minimum(a, gmin)
                            trajectory_found = True

                        g[k0, ix, iy, l0] = gmin

        return trajectory_found

    def _minimal_lNFA(self):
        frames = self.frames
        K = self.K
        g, activated = self.g, self.activated

        m = Inf
        for k0 in xrange(2,K):
            fx = frames[k0]
            fy = frames[k0-1]
            for ix, x in enumerate(fx):
                if not(activated[k0, ix]): continue
                for iy, y in enumerate(fy):
                    if not(activated[k0-1, iy]): continue
                    for l0 in xrange(2, k0+1):
                        l = l0+1
                        m = minimum(m, self.log_NFA(k0, l, g[k0, ix, iy, l0]))
        return m

    def log_NFA(self, last_frame, l, a):
        """
        PARAMETERS
            last_frame -- last frame of the trajectory
            l          -- length of the trajectory
            a          -- maximal discrete area of the acceleration
        """
        K = self.K
        log_k = self.log_k
        log_nprod = self.log_nprod
        log_image_area = self.log_image_area
        l0 = l-1
        return log_k[K] + log_k[K-l+1] + log_nprod[last_frame, l0] + (l-2)*(log10(a) - log_image_area)

    def _extract_trajectory(self, m, l0, k0, ix, iy):
        frames = self.frames
        K = self.K
        g, activated = self.g, self.activated

        l = l0+1

        assert k0 >= l0
        assert l >= 3
        assert abs(self.log_NFA(k0, l, g[k0,ix,iy,l0])-m) < 1e-4

        a_max = g[k0, ix, iy, l0]

        traj = []

        while l0 >= 2:
            traj.append((k0, ix))

            x = frames[k0][ix]
            y = frames[k0-1][iy]
            m0 = x[0] - 2*y[0]
            m1 = x[1] - 2*y[1]

            iz_min = None
            a_min = Inf
            for iz, z in enumerate(frames[k0-2]):
                if not(activated[k0-2, iz]): continue
                z0, z1 = z[0], z[1]

                a = maximum(a_d(m0 + z0, m1 + z1), g[k0-1, iy, iz, l0-1])
                if a < a_min and a < a_max + 1e-4:
                    iz_min = iz
                    a_min = a

            k0 -= 1
            l0 -= 1
            ix = iy
            iy = iz_min
            a_max = a_min

        traj.append((k0, ix))
        traj.append((k0-1, iy))

        for k0, ix in traj:
            assert activated[k0, ix] == True
            activated[k0, ix] = False

        traj.reverse()
        return traj

    def solve(self, eps=0.0):
        frames = self.frames
        K = self.K
        g, activated = self.g, self.activated

        trajs = []
        while True:
            trajectory_found = self._compute_g()
            if not(trajectory_found): break

            m = self._minimal_lNFA()
            print "min(log_NFA) =", m
            if m > eps: break

            try:
                for k0 in xrange(K-1,1,-1):
                    fx = frames[k0]
                    fy = frames[k0-1]
                    for ix, x in enumerate(fx):
                        if not(activated[k0, ix]): continue
                        for iy, y in enumerate(fy):
                            if not(activated[k0-1, iy]): continue
                            for l0 in xrange(k0,1,-1):
                                l = l0 + 1
                                cur_m = self.log_NFA(k0, l, g[k0, ix, iy, l0])
                                if abs(m-cur_m) < 1e-4:
                                    t = self._extract_trajectory(m, l0, k0, ix, iy)
                                    if t is None:
                                        raise Exception
                                    else:
                                        raise FoundTrajectoryException(t, cur_m)
            except FoundTrajectoryException as t:
                trajs.append((t.traj, t.lNFA))

        return trajs
#}}}

class HolesSolver(object):
#{{{
    def __init__(self, image_size, frames):
        self.frames = frames
        self.K = K = len(frames)
        self.log_k = combinatorics.precompute_log_K(K)
        self.log_Cnk = combinatorics.precompute_log_Cnk(K)

        self.image_size = image_size
        self.image_area = image_area = points.width*points.height
        self.log_image_area = log10(image_area)

        N = max([len(frames[k]) for k in xrange(K)])
        self.g = np.ones((K, N, K, N, K, K, K), dtype=np.float64)*Inf
        self.activated = np.ones((K, N), dtype=bool)

        self.log_nprod = np.zeros((K, K, K), dtype=np.float64)
        self._precompute_log_nprod()

    def _precompute_log_nprod(self):
        """
        log_nprod[k0][l0][s0] = N_{k0-l0} * max_prod * N_{k0}
        where max_prod = max_{k0-l0 < j_1 < ... < j_{s0-1} < k0} prod_k N_{j_k}
        and N_i = len(frames[i])
        """
        frames = self.frames
        log_nprod = self.log_nprod
        K = self.K
        Ns = [log10(len(f)) if len(f) > 0 else -Inf for f in frames]
        for k0 in xrange(K):
            for l0 in xrange(k0+1):
                ms = list(reversed(sorted(Ns[k0-l0+1:k0])))
                for s0 in xrange(l0+1):
                    if s0 == 0:
                        log_nprod[k0, l0, s0] = -Inf
                    else:
                        log_max_prod = np.sum(ms[:s0-2+1])
                        log_nprod[k0, l0, s0] = Ns[k0] + Ns[k0-l0] + log_max_prod

    def _compute_g(self):
      frames = self.frames
      K = self.K
      g, activated = self.g, self.activated

      trajectory_found = False

      # for k # k0 = k - 1
      #   for x
      #     for h1
      #       for y
      #         for l, s, p # l0 = l-1, s0 = s-1, p0 = p-1
      #           # compute g(k, x, h1, y, l, s, p)
      #           for h2
      #             for z

      for k0 in xrange(1,K):
        fx = frames[k0]
        for ix, x in enumerate(fx):
          if not(activated[k0, ix]): continue
          x0, x1 = x[0], x[1]

          for h1 in xrange(k0):
            k1 = k0-h1-1
            fy = frames[k1]

            for iy, y in enumerate(fy):
              if not(activated[k1, iy]): continue
              y0, y1 = y[0], y[1]
              m0, m1 = (x0-y0)/float(h1+1), (x1-y1)/float(h1+1)

              if h1 == 0:
                  g[k0, ix, h1, iy, 1, 1, 0] = 1.0 # l=2, s=2, p=1
              else:
                  g[k0, ix, h1, iy, 1, 1, 1] = 1.0 # l=2, s=2, p=2

              for l0 in xrange(2,k0+1):
                # We must look for the trajectories in the previous frames having
                # length l0_prev
                l0_prev = l0-h1-1

                for s0 in xrange(2,l0+1):
                  # We must look for the trajectories in the previous frames having
                  # size s0_prev
                  s0_prev = s0-1

                  for p0 in xrange(s0+1):
                    # If we have a hole between x and y, we must look for
                    # the trajectories having at most prev_p0 runs
                    p0_prev = p0 - (1 if h1 > 0 else 0)
                    if p0_prev < 0: continue

                    gmin = Inf

                    for h2 in xrange(min(k1, l0_prev)):

                      k2 = k1-h2-1
                      fz = frames[k2]

                      for iz, z in enumerate(fz):
                        if not(activated[k2, iz]): continue
                        z0, z1 = z[0], z[1]
                        c0, c1 = m0 + (z0-y0)/float(h2+1), m1 + (z1-y1)/float(h2+1)

                        a = maximum(a_d(c0, c1), \
                                g[k1, iy, h2, iz, l0_prev, s0_prev, p0_prev])
                        gmin = minimum(a, gmin)

                        trajectory_found = True

                    g[k0, ix, h1, iy, l0, s0, p0] = gmin

      return trajectory_found

    def _minimal_lNFA(self):
      frames = self.frames
      K = self.K
      g, activated = self.g, self.activated

      m = Inf
      for k0 in xrange(2,K):
        fx = frames[k0]
        for ix, x in enumerate(fx):
          if not(activated[k0, ix]): continue

          for h1 in xrange(k0):
            k1 = k0-h1-1
            fy = frames[k1]

            for iy, y in enumerate(fy):
              if not(activated[k1, iy]): continue

              for l0 in xrange(2,k0+1):
                for s0 in xrange(2,l0+1):
                  for p0 in xrange(s0+1):
                    a = g[k0, ix, h1, iy, l0, s0, p0]
                    if np.isinf(a): continue
                    m = minimum(m, self.log_NFA(k0, l0+1, s0+1, p0+1, a))

      return m

    def log_NFA(self, last_frame, l, s, p, a):
        """
        PARAMETERS
            last_frame -- last frame of the trajectory
            l          -- length of the trajectory
            s          -- size of the trajectory
            p          -- number of runs of the trajectory
            a          -- maximal discrete area of the acceleration
        """
        K = self.K
        log_k = self.log_k
        log_Cnk = self.log_Cnk
        log_nprod = self.log_nprod
        log_image_area = self.log_image_area
        l0 = l-1
        s0 = s-1
        p0 = p-1
        if p > 1:
            hole_factor = (2*p-2)*log10((l-s)/float(p-1) + 1.0)
        else:
            hole_factor = 0.0

        return log_k[K] + log_k[K-l+1] + log_k[l] + log_Cnk[l, s] + \
                log_nprod[last_frame, l0, s0] + \
                (s-2)*(log10(a) - log_image_area) + hole_factor

    def _extract_trajectory(self, m, l0, s0, p0, k0, ix, h1, iy):
        frames = self.frames
        K = self.K
        g, activated = self.g, self.activated

        l = l0+1
        s = s0+1
        p = p0+1

        assert k0 >= l0
        assert l >= s
        assert s >= 3
        assert abs(self.log_NFA(k0, l, s, p, g[k0,ix,h1,iy,l0,s0,p0])-m) < 1e-4

        a_max = g[k0,ix,h1,iy,l0,s0,p0]

        traj = []

        while s0 >= 2:
            traj.append((k0, ix))

            x = frames[k0][ix]
            k1 = k0-h1-1
            y = frames[k1][iy]
            m0 = (x[0]-y[0])/float(h1+1)
            m1 = (x[1]-y[1])/float(h1+1)

            iz_min = None
            h2_min = None
            a_min = Inf

            l0_prev = l0-h1-1
            s0_prev = s0-1
            p0_prev = p0 - (1 if h1 > 0 else 0)
            assert p0_prev >= 0

            for h2 in xrange(min(k1, l0_prev)):
                k2 = k1-h2-1
                fz = frames[k2]

                for iz, z in enumerate(fz):
                  if not(activated[k2, iz]): continue
                  c0 = m0 + (z[0]-y[0])/float(h2+1)
                  c1 = m1 + (z[1]-y[1])/float(h2+1)

                  a = maximum(a_d(c0, c1), \
                          g[k1, iy, h2, iz, l0_prev, s0_prev, p0_prev])
                  if a < a_min and a < a_max + 1e-4:
                      iz_min = iz
                      h2_min = h2
                      a_min = a

            k0 = k1
            l0 = l0_prev
            s0 = s0_prev
            p0 = p0_prev
            ix = iy
            iy = iz_min
            h1 = h2_min
            a_max = a_min

        traj.append((k0, ix))
        traj.append((k0-h1-1, iy))

        for k0, ix in traj:
            assert activated[k0, ix] == True
            activated[k0, ix] = False

        traj.reverse()
        return traj

    def solve(self, eps=0.0):
        frames = self.frames
        K = self.K
        g, activated = self.g, self.activated

        trajs = []
        while True:
            trajectory_found = self._compute_g()
            if not(trajectory_found): break

            m = self._minimal_lNFA()
            print "min(log_NFA) =", m
            if m > eps: break

            try:
              for k0 in xrange(K-1,1,-1):
                fx = frames[k0]
                for ix, x in enumerate(fx):
                  if not(activated[k0, ix]): continue
                  for h1 in xrange(k0):
                    k1 = k0-h1-1
                    fy = frames[k1]
                    for iy, y in enumerate(fy):
                      if not(activated[k1, iy]): continue
                      for l0 in xrange(2,k0+1):
                        for s0 in xrange(2,l0+1):
                          for p0 in xrange(s0+1):
                            a = g[k0, ix, h1, iy, l0, s0, p0]
                            if np.isinf(a): continue
                            cur_m = self.log_NFA(k0, l0+1, s0+1, p0+1, a)
                            if abs(m-cur_m) < 1e-4:
                              t = self._extract_trajectory(m, l0, s0, p0, k0, ix, h1, iy)
                              if t is None:
                                raise Exception
                              else:
                                raise FoundTrajectoryException(t, cur_m)

            except FoundTrajectoryException as t:
                trajs.append((t.traj, t.lNFA))

        return trajs
#}}}

if __name__ == "__main__":
    from pymage.vendor import argparse

    parser = argparse.ArgumentParser(description='ASTRE tracking algorithm (naive implementation)')
    parser.add_argument('filename_in', metavar='in', help='the input PointsDesc file')
    parser.add_argument('filename_out', metavar='out', help='the output PointsDesc file')
    parser.add_argument('-e', '--eps', dest='eps', default=0.0, type=float,
      help='the maximal value of log(NFA) when searching for meaningful trajectories (default: 0.0)')
    parser.add_argument('--solver', default='noholes',
      choices=['noholes', 'holes'])
    args = parser.parse_args()

    points = PointsDesc.from_string(open(args.filename_in).read())
    if len(points.frames) < 3:
      raise ValueError('Input points file must have at least 3 images!')

    def load_frames(points):
        fs = []
        for f in points.frames:
            fs.append(np.array([(p[0], p[1]) for p in f], dtype=np.float64))
        return fs

    frames = load_frames(points)
    image_size = (points.width, points.height)
    eps = float(args.eps)

    if args.solver == 'noholes':
        solver = NoholesSolver(image_size, frames)
    elif args.solver == 'holes':
        solver = HolesSolver(image_size, frames)
    else:
      raise ValueError('Unknown solver {0}'.format(args.solver))

    trajs = solver.solve(eps)
    trajs_file = points.copy(1)
    trajs_file.tags[-1] = 't'
    for k, f in enumerate(trajs_file.frames):
        for p in f:
            p[-1] = -1.0
    for it, tdesc in enumerate(trajs):
        t, lNFA = tdesc
        trajs_file.headers['traj:{0}:lNFA'.format(it)] = '%g' % lNFA
        for k, i in t:
            trajs_file.frames[k][i][-1] = it
    trajs_file.save(args.filename_out)

