#ifndef _PRIV_TYPES_H_
#define _PRIV_TYPES_H_

enum screen {
  SCREEN_STATUS = 0,
  SCREEN_SET_MODE,
  SCREEN_SET_FREQUENCY,
  SCREEN_SET_OUTPUT_POWER,
  SCREEN_GPS_STATUS,
  SCREEN_CALIBRATION,
  SCREEN_TRANSMITTER,
  SCREEN_VERSION,
  SCREEN_COUNT
};

enum mode {
  MODE_JT9 = 0,
  MODE_JT65,
/*  
  MODE_JT4,
*/
  MODE_WSPR,
/*
  MODE_FSQ_2,
  MODE_FSQ_3,
  MODE_FSQ_4_5,
  MODE_FSQ_6,
*/
  MODE_FT8,
  MODE_COUNT
};

enum band {
  BAND_160M = 0,
  BAND_80M,
  BAND_40M,
  BAND_30M,
  BAND_20M,
  BAND_17M,
  BAND_15M,
  BAND_12M,
  BAND_10M,
  BAND_6M,
  BAND_2M,
  BAND_COUNT
};

struct mode_param {
  const char mode_name[5];
  uint16_t symbol_count;
  uint16_t tone_spacing;
  uint16_t tone_delay;
  uint8_t start_time;
  const uint32_t *freqs;
};

const uint32_t jt9_freqs[BAND_COUNT] = {
    1839000UL,
    3572000UL,
    7078000UL,
   10140000UL,
   14078000UL,
   18104000UL,
   21078000UL,
   24919000UL,
   28078000UL,
   50312000UL,
  144120000UL
};

const uint32_t jt65_freqs[BAND_COUNT] = {
    1838000UL,
    3570000UL,
    7076000UL,
   10138000UL,
   14076000UL,
   18102000UL,
   21076000UL,
   24917000UL,
   28076000UL,
   50310000UL,
  144120000UL
};

const uint32_t wspr_freqs[BAND_COUNT] = {
    1838100UL,
    3570100UL,
    7040100UL,
   10140200UL,
   14097100UL,
   18106100UL,
   21096100UL,
   24926100UL,
   28126100UL,
   50294500UL,
  144490500UL
};

const uint32_t ft8_freqs[BAND_COUNT] = {
    1840000UL,
    3573000UL,
    7074000UL,
   10136000UL,
   14074000UL,
   18100000UL,
   21074000UL,
   24915000UL,
   28074000UL,
   50313000UL,
  144174000UL
};

#define OUTPUT_POWER_MIN_DBM  (0U)
#define OUTPUT_POWER_MAX_DBM (30U)

typedef void (*cal_refresh_cb)(void);

#endif /* _PRIV_TYPES_H_ */
