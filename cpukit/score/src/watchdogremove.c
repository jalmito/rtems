/**
 * @file
 *
 * @brief Remove Watchdog
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
  while ( iterator_node != iterator_tail ) {
    Watchdog_Iterator *iterator;

    iterator = (Watchdog_Iterator *) iterator_node;

    if ( iterator->current == next ) {
      iterator->delta_interval += delta;
    }

    if ( iterator->current == &the_watchdog->Node ) {
      Chain_Node *previous = _Chain_Previous( &the_watchdog->Node );

      iterator->current = previous;

      if ( previous != _Chain_Head( &header->Watchdogs ) ) {
        Watchdog_Control *previous_watchdog;

        previous_watchdog = (Watchdog_Control *) previous;
        iterator->delta_interval += previous_watchdog->delta_interval;
      }
    }

    iterator_node = _Chain_Next( iterator_node );
  }
}

Watchdog_States _Watchdog_Remove(
=======
void _Watchdog_Remove(
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
  Watchdog_Header  *header,
  Watchdog_Control *the_watchdog
)
{
  if ( _Watchdog_Is_scheduled( the_watchdog ) ) {
    if ( header->first == &the_watchdog->Node.RBTree ) {
      _Watchdog_Next_first( header, the_watchdog );
    }

    _RBTree_Extract( &header->Watchdogs, &the_watchdog->Node.RBTree );
    _Watchdog_Set_state( the_watchdog, WATCHDOG_INACTIVE );
  }
}
