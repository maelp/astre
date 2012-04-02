#ifndef _VISION_MATH_BASE_H
#define _VISION_MATH_BASE_H

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

#define M_TWO_PI (2.0f*M_PI)

static inline float dist(float x1, float y1, float x2, float y2) { return sqrtf((float)((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))) ; }
static inline double dist_d(double x1, double y1, double x2, double y2) { return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)) ; }
static inline float dist2(float x1, float y1, float x2, float y2) { return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) ; }
static inline double dist2_d(double x1, double y1, double x2, double y2) { return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) ; }
static inline int dist2_i(int x1, int y1, int x2, int y2) { return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) ; }
/* returns the equivalent angle in [-pi, pi[ */
static inline float normalize_angle_f(float theta) {
  while (theta >= M_PI) theta -= M_TWO_PI ;
  while (theta < -M_PI) theta += M_TWO_PI ;
  return theta ;
}
static inline double normalize_angle_d(double theta) {
  while (theta >= M_PI) theta -= M_TWO_PI ;
  while (theta < -M_PI) theta += M_TWO_PI ;
  return theta ;
}
static inline float angle_abs_diff_f(float t1, float t2)
{
  float theta = t1-t2 ;
  while (theta >= M_PI) theta -= M_TWO_PI ;
  while (theta < -M_PI) theta += M_TWO_PI ;
  return abs_f(theta) ;
}
static inline double angle_abs_diff_d(double t1, double t2)
{
  double theta = t1-t2 ;
  while (theta >= M_PI) theta -= M_TWO_PI ;
  while (theta < -M_PI) theta += M_TWO_PI ;
  return abs_d(theta) ;
}
static inline double angle_line_abs_diff_d(double t1, double t2)
{
  return min_d(angle_abs_diff_d(t1,t2),angle_abs_diff_d(t1,t2+M_PI)) ;
}

#endif
