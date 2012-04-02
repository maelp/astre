#ifndef _VISION_TRAJS_POINTSDESC_H
#define _VISION_TRAJS_POINTSDESC_H

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

/*
 * Notes:
 *
 *   This is a DescFile (see vision/formats/descfile.h)
 *
 *   Headers must include:
 *     type = PointsFile v.1.0
 *     uid = <integer uid>
 *     width = <integer width>
 *     height = <integer height>
 *
 *   The data must be:
 *
 *     frame:<integer frame> x:<x coord> y:<y coord> ...
 *
 ******************************************************************/

#include <vision/core.h>
#include <vision/formats/descfile.h>
#include <vision/trajs/trajs.h>

typedef struct st_points_desc *points_desc ;
struct st_points_desc
{
  int width ;
  int height ;
  int uid ;

  int orig_first_frame ;

  int n_headers ;
  char** header_captions ;
  char** header_contents ;

  int n_frames ;
  int* n_points_in_frame ;
  int n_fields ;
  char** tags ;
  /* points[f][p*n_fields + k] gives the value of field k for the point i in
   * image f (first two fields are x, y) */
  double** points ;
};

points_desc points_desc_new();
void points_desc_free_all( points_desc* ppd );

/* Set a random uid */
void points_desc_set_uid( points_desc pd );

/* Load and optionally add an additional column */
points_desc points_desc_load_ext( Rawdata raw_in, int n_additional_fields );

/* Load a pointsdesc file from a Rawdata structure */
points_desc points_desc_load( Rawdata raw_in );

void points_desc_save( Rawdata raw_out, points_desc pd );

points_desc points_desc_copy( points_desc pd, int n_new_fields );

/* Set a field value for all points */
void points_desc_set_field( points_desc pd, int n_field, double v );

/* Tag the points in pd with corresponding index of trajectory in field n_field */
void points_desc_set_traj_tags( points_desc pd, trajs_file tf, int n_field );

/* Convenience function that add a trajectory field and save trajectories */
void points_desc_save_with_new_trajs( Rawdata raw_out, points_desc pd, trajs_file tf );

/* Extract tags in field <n_field> and construct trajectories, trajectories
 * indices in trajs_file do not necessarily correspond to point indices,
 * however if relabel_trajs is set, the point indices will be relabeled so that
 * it is the case */
trajs_file points_desc_extract_trajs( points_desc pd, int n_field, char relabel_trajs );

/* Copy basic informations from the source points_desc to the destination
 * points_desc */
void points_desc_copy_infos( points_desc from, points_desc to );

#endif
