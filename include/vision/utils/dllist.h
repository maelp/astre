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

#error "Do not include this file directly, but copy it and replace the _TYPE_ informations"

/* SIMPLE DOUBLY-LINKED LIST STRUCTURE */
/* ------------------------------------------------- */
/* replace _TYPE_ with the real type (ex: my_struct*)
 * replace _TYPEN_ with the "typename" for function/structure name (ex:
 * my_struct_ptr) */
typedef struct st__TYPEN_list {
  _TYPE_ data ;

  struct st__TYPEN_list *next ;
  struct st__TYPEN_list *prev ;
} *_TYPEN_list ;
static void _TYPEN_list_cons(_TYPE_ hd, _TYPEN_list *next)
{
  _TYPEN_list t = (_TYPEN_list) malloc(sizeof(struct st__TYPEN_list)) ;
  t->data = hd ;
  t->prev = NULL ;
  t->next = *next ;
  if (*next) (*next)->prev = t ;
  *next = t ;
}
static uint _TYPEN_list_length(_TYPEN_list l)
{
  uint count = 0 ;
  while (l) { count++ ; l=l->next ; }
  return count ;
}
static inline void _TYPEN_list_free(_TYPEN_list *pl)
{
  _TYPEN_list l = *pl ;
  if (l && l->prev) mini_mwerror(FATAL,1,"[_TYPEN_list_free]: not the first list item\n") ;
  while (l) {
    _TYPEN_list next = l->next ;
    free(l) ;
    if (next) next->prev = (_TYPEN_list)NULL ;
    l = next ;
  }
  *pl = (_TYPEN_list)NULL ;
}
static inline _TYPE_ _TYPEN_list_hd(_TYPEN_list l)
{
  if (!l) mini_mwerror(FATAL,1,"[_TYPEN_list_hd] list is empty !\n") ;
  return l->data ;
}
static inline _TYPEN_list _TYPEN_list_tl(_TYPEN_list l)
{
  if (!l) mini_mwerror(FATAL,1,"[_TYPEN_list_tl] list is empty !\n") ;
  return l->next ;
}
/* Returns NULL if index out of bounds */
static _TYPE_ _TYPEN_list_nth(uint n, _TYPEN_list l)
{
  while (l && n > 0) {
    l = l->next ;
    n-- ;
  }
  return l->data ;
}
static _TYPEN_list _TYPEN_list_find(_TYPE_ p, _TYPEN_list l)
{
  while (l) {
    if (l->data == p) return l ;
    l = l->next ;
  }
  return NULL ;
}
static int _TYPEN_list_mem(_TYPE_ p, _TYPEN_list l)
{
  while (l) {
    if (l->data == p) return TRUE ;
    l = l->next ;
  }
  return FALSE ;
}
static inline _TYPEN_list _TYPEN_list_copy(_TYPEN_list l)
{
  if (!l) return NULL ;
  _TYPEN_list c = (_TYPEN_list)NULL ;
  _TYPEN_list_cons(l->data,&c) ;
  _TYPEN_list lst = c ;
  l = l->next ;
  while (l) {
    _TYPEN_list nxt = (_TYPEN_list)NULL ;
    _TYPEN_list_cons(l->data,&nxt) ;
    lst->next = nxt ;
    nxt->prev = lst ;
    lst = nxt ;
    l = l->next ;
  }
  return c ;
}
/* do not free the node */
static void _TYPEN_list_remove_node(_TYPEN_list node, _TYPEN_list *pl)
{
  _TYPEN_list l = *pl ;
  if (l && l->prev) mini_mwerror(FATAL,1,"[_TYPEN_list_remove_node]: not the first item of the list\n") ;
  if (node == l) {
    l = node->next ;
    if (l) l->prev = (_TYPEN_list)NULL ;
    node->next = (_TYPEN_list)NULL ;
    *pl = l ;
  }
  else {
    if (node->prev == NULL) mini_mwerror(FATAL,1,"[_TYPEN_list_remove_node] node is not the first in the list, but has no "
                                            "previous node!\n") ;
    node->prev->next = node->next ;
    if (node->next) node->next->prev = node->prev ;
    node->prev = (_TYPEN_list)NULL ;
    node->next = (_TYPEN_list)NULL ;
    *pl = l ;
  }
}
/* free the node */
static void _TYPEN_list_remove(_TYPE_ p, _TYPEN_list *pl)
{
  _TYPEN_list l = *pl ;
  if (l && l->prev) mini_mwerror(FATAL,1,"[_TYPEN_list_remove]: not the first item of the list\n") ;
  _TYPEN_list first = l ;
  while (l) {
    if (l->data == p) {
      if (l->prev) l->prev->next = l->next ;
      else first = l->next ;
      if (l->next) l->next->prev = l->prev ;
      l->prev = NULL ;
      l->next = NULL ;
      _TYPEN_list_free(&l) ;
      *pl = first ;
      return ;
    }
    l = l->next ;
  }
  *pl = first ;
  return ;
}
