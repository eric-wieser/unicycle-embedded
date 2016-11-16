#pragma once

// https://github.com/chipKIT32/chipKIT-core/pull/298
/* Define bits in the change notice control register
*/
#define _BN_CNCON_ON        15
#define _BN_CNCON_SIDL      13

#define CNCON_ON            (1 << _BN_CNCON_ON)
#define CNCON_OFF           (0)
#define CNCON_IDLE_STOP     (1 << _BN_CNCON_SIDL)
#define CNCON_IDLE_RUN      (0)

#define _CN_BASE_ADDRESS    reinterpret_cast<uintptr_t>(&CNCON)
