/**
 * @file tms570-pins.h
 *
 * @ingroup tms570
 *
 * @brief Select pin mapping according to selected chip.
 *        Defaults to TMS570LS3137ZWT for now.
 */
#define TMS570_LC43X
#if defined (TMS570_LC43X)
#include <bsp/tms570lc4357-pins.h>
#else
#include <bsp/tms570ls3137zwt-pins.h>
#endif
