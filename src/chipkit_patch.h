#pragma once

// https://github.com/chipKIT32/chipKIT-core/pull/298
// Belongs in :/cores/pic32/p32_defs.h
#define _BN_CNCON_ON        15
#define _BN_CNCON_SIDL      13

#define CNCON_ON            (1 << _BN_CNCON_ON)
#define CNCON_OFF           (0)
#define CNCON_IDLE_STOP     (1 << _BN_CNCON_SIDL)
#define CNCON_IDLE_RUN      (0)

#define _CN_BASE_ADDRESS    reinterpret_cast<uintptr_t>(&CNCON)

// https://github.com/chipKIT32/chipKIT-core/pull/299/files
// Belongs in :/variants/Max32/Board_Data.c
#define _SER0_TX_PIN    1
#define _SER0_RX_PIN    0
#define _SER0_RTS_PIN   18
#define _SER0_CTS_PIN   19
