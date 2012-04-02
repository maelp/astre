#ifndef _VISION_STRING_H
#define _VISION_STRING_H

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

/* return whether c is a space, tab, etc. */
char is_whitespace( char c );
/* modify the string in place, return a pointer to the first non-whitespace
 * character, and put a null character after the last non-whitespace character. */
char* trim_whitespace( char* str );

/* locate a character */
int str_index_of_char( char* str, char c );
/* remove the chars from trim appearing at the end of str */
void str_rchomp( char* str, const char* trim );

#endif
