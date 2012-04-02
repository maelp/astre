#ifndef _VISION_FORMATS_DESCFILE_H
#define _VISION_FORMATS_DESCFILE_H

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
 *  You can add comment lines by starting with a "#"
 *
 *  Format is:
 *
 *  header1 = content1
 *  header2 = content2
 *
 *  DATA
 *
 *  id1:value id2:value id3:value
 *  id1:value id2:value id3:value
 *
 *  the number of fields must be the same in each line
 *  the "id:" part is optional
 *  the labels must (if present) be the same in each column
 *
 ******************************************************************/

#include <vision/core.h>

typedef struct st_desc_file *desc_file ;
struct st_desc_file
{
  int n_headers ;
  char** header_captions ; /* end with a NULL entry */
  char** header_contents ;

  int n_lines ;
  int n_fields ;
  double** lines ;
  char** tags ;
};

/******************************************************************************

        desc_file_new

        Create an empty desc_file structure

******************************************************************************/
desc_file desc_file_new();

/******************************************************************************

        desc_file_free_all

        Free a desc_file structure and all its inner fields recursively

******************************************************************************/
void desc_file_free_all( desc_file* pdf );

/******************************************************************************

        desc_file_load

        Create a desc_file structure from [raw_in]

******************************************************************************/
desc_file desc_file_load( Rawdata raw_in );

/******************************************************************************

        desc_file_save

        Save the desc_file structure in [raw_out]

******************************************************************************/
void desc_file_save( Rawdata raw_out, desc_file df );

/******************************************************************************

        desc_file_print

        Print the desc_file structure

******************************************************************************/
void desc_file_print( desc_file df );

/******************************************************************************

        desc_file_add_header

        Adds an header, does not copy the string caption and content but
        simply reference them

******************************************************************************/
void desc_file_add_header( desc_file df, char* caption, char* content );

/******************************************************************************

        desc_file_find_header

        Returns the index of the first header matching [str], or -1 if there
        is no matching header

******************************************************************************/
int desc_file_find_header( desc_file df, char* str );

/******************************************************************************

        desc_file_read_header_value

        Return true if the value could be read

******************************************************************************/
char desc_file_read_header_value( desc_file df, char* caption, char* fmt, void* res );

#endif
