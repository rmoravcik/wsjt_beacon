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
    1839700UL,
    3572700UL,
    7078700UL,
   10140700UL,
   14078700UL,
   18104700UL,
   21078700UL,
   24919700UL,
   28078700UL,
   50312700UL,
  144120700UL
};

const uint32_t jt65_freqs[BAND_COUNT] = {
    1838300UL,
    3570300UL,
    7076300UL,
   10138300UL,
   14076300UL,
   18102300UL,
   21076300UL,
   24917300UL,
   28076300UL,
   50310300UL,
  144120300UL
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
    1841000UL,
    3574000UL,
    7075000UL,
   10137000UL,
   14075000UL,
   18101000UL,
   21075000UL,
   24916000UL,
   28075000UL,
   50314000UL,
  144175000UL
};

#define OUTPUT_POWER_MIN_DBM  (0U)
#define OUTPUT_POWER_MAX_DBM (30U)

typedef void (*cal_refresh_cb)(void);

#endif /* _PRIV_TYPES_H_ */
