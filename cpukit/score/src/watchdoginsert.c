/**
 * @file
 *
 * @brief Watchdog Insert
 * @ingroup ScoreWatchdog
 */

/*
 * Copyright (c) 2016 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/score/watchdogimpl.h>

<<<<<<< HEAD
static void _Watchdog_Insert_fixup(
  Watchdog_Header   *header,
  Watchdog_Control  *the_watchdog,
  Watchdog_Interval  delta,
  Watchdog_Control  *next_watchdog,
  Watchdog_Interval  delta_next
)
{
  const Chain_Node *iterator_tail;
  Chain_Node       *iterator_node;

  next_watchdog->delta_interval = delta_next - delta;

  iterator_node = _Chain_First( &header->Iterators );
  iterator_tail = _Chain_Immutable_tail( &header->Iterators );

  while ( iterator_node != iterator_tail ) {
    Watchdog_Iterator *iterator;

    iterator = (Watchdog_Iterator *) iterator_node;

    if ( iterator->current == &next_watchdog->Node ) {
      iterator->current = &the_watchdog->Node;
    }

    iterator_node = _Chain_Next( iterator_node );
  }
}

void _Watchdog_Insert_locked(
=======
void _Watchdog_Insert(
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
  Watchdog_Header  *header,
  Watchdog_Control *the_watchdog,
  uint64_t          expire
)
{
  RBTree_Node **link;
  RBTree_Node  *parent;
  RBTree_Node  *old_first;
  RBTree_Node  *new_first;

  _Assert( _Watchdog_Get_state( the_watchdog ) == WATCHDOG_INACTIVE );

  link = _RBTree_Root_reference( &header->Watchdogs );
  parent = NULL;
  old_first = header->first;
  new_first = &the_watchdog->Node.RBTree;

<<<<<<< HEAD
      if ( delta < delta_next ) {
        _Watchdog_Insert_fixup(
          header,
          the_watchdog,
          delta,
          next_watchdog,
          delta_next
        );
        break;
      }
=======
  the_watchdog->expire = expire;
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

  while ( *link != NULL ) {
    Watchdog_Control *parent_watchdog;

    parent = *link;
    parent_watchdog = (Watchdog_Control *) parent;

    if ( expire < parent_watchdog->expire ) {
      link = _RBTree_Left_reference( parent );
    } else {
      link = _RBTree_Right_reference( parent );
      new_first = old_first;
    }
  }

  header->first = new_first;
  _RBTree_Initialize_node( &the_watchdog->Node.RBTree );
  _RBTree_Add_child( &the_watchdog->Node.RBTree, parent, link );
  _RBTree_Insert_color( &header->Watchdogs, &the_watchdog->Node.RBTree );
}
