/* Storage the generated object dictionary points into.
 *
 * gen_od.py emits `extern _Objects Obj;` in utypes.h and points every mapped
 * entry of slave_objectlist.c at a member of it. Exactly one translation unit
 * has to define it, and this is that unit: keeping it out of the generated
 * directory means the application owns its own process data while the
 * dictionary describing it stays machine-written.
 */

#include "utypes.h"

_Objects Obj;
