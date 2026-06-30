/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten
Lamers. Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2024 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "tinygps.hpp"
#include "tinygps.h"
#include "tinyubx.h"

#include "stm32g4xx_hal.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _RMCterm "RMC"
#define _GGAterm "GGA"

TinyGPSPlus::TinyGPSPlus()
    : parity(0), isChecksumTerm(false), curSentenceType(GPS_SENTENCE_OTHER),
      curTermNumber(0), curTermOffset(0), sentenceHasFix(false), customElts(0),
      customCandidates(0), encodedCharCount(0), sentencesWithFixCount(0),
      failedChecksumCount(0), passedChecksumCount(0) {
  term[0] = '\0';
}

//
// public methods
//

bool TinyGPSPlus::encode(char c) {
  ++encodedCharCount;

  switch (c) {
  case ',': // term terminators
    parity ^= (uint8_t)c;
  case '\r':
  case '\n':
  case '*': {
    bool isValidSentence = false;
    if (curTermOffset < sizeof(term)) {
      term[curTermOffset] = 0;
      isValidSentence = endOfTermHandler();
    }
    ++curTermNumber;
    curTermOffset = 0;
    isChecksumTerm = c == '*';
    return isValidSentence;
  } break;

  case '$': // sentence begin
    curTermNumber = curTermOffset = 0;
    parity = 0;
    curSentenceType = GPS_SENTENCE_OTHER;
    isChecksumTerm = false;
    sentenceHasFix = false;
    return false;

  default: // ordinary characters
    if (curTermOffset < sizeof(term) - 1)
      term[curTermOffset++] = c;
    if (!isChecksumTerm)
      parity ^= c;
    return false;
  }

  return false;
}

//
// internal utilities
//

int TinyGPSPlus::fromHex(char a) {
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

// static
// Parse a (potentially negative) number with up to 2 decimal digits -xxxx.yy
int32_t TinyGPSPlus::parseDecimal(const char *term) {
  bool negative = *term == '-';
  if (negative)
    ++term;
  int32_t ret = 100 * (int32_t)atol(term);
  while (isdigit(*term))
    ++term;
  if (*term == '.' && isdigit(term[1])) {
    ret += 10 * (term[1] - '0');
    if (isdigit(term[2]))
      ret += term[2] - '0';
  }
  return negative ? -ret : ret;
}

// static
// Parse degrees in that funny NMEA format DDMM.MMMM
void TinyGPSPlus::parseDegrees(const char *term, RawDegrees &deg) {
  uint32_t leftOfDecimal = (uint32_t)atol(term);
  uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
  uint32_t multiplier = 10000000UL;
  uint32_t tenMillionthsOfMinutes = minutes * multiplier;

  deg.deg = (int16_t)(leftOfDecimal / 100);

  while (isdigit(*term))
    ++term;

  if (*term == '.')
    while (isdigit(*++term)) {
      multiplier /= 10;
      tenMillionthsOfMinutes += (*term - '0') * multiplier;
    }

  deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
  deg.negative = false;
}

#define COMBINE(sentence_type, term_number)                                    \
  (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPSPlus::endOfTermHandler() {
  // If it's the checksum term, and the checksum checks out, commit
  if (isChecksumTerm) {
    uint8_t checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
    if (checksum == parity) {
      passedChecksumCount++;
      if (sentenceHasFix)
        ++sentencesWithFixCount;

      switch (curSentenceType) {
      case GPS_SENTENCE_RMC:
        date.commit();
        time.commit();
        if (sentenceHasFix) {
          location.commit();
          speed.commit();
          course.commit();
        }
        break;
      case GPS_SENTENCE_GGA:
        time.commit();
        if (sentenceHasFix) {
          location.commit();
          altitude.commit();
        }
        satellites.commit();
        hdop.commit();
        break;
      }

      // Commit all custom listeners of this sentence type
      for (TinyGPSCustom *p = customCandidates;
           p != NULL &&
           strcmp(p->sentenceName, customCandidates->sentenceName) == 0;
           p = p->next)
        p->commit();
      return true;
    }

    else {
      ++failedChecksumCount;
    }

    return false;
  }

  // the first term determines the sentence type
  if (curTermNumber == 0) {
    if (term[0] == 'G' && strchr("PNABL", term[1]) != NULL &&
        !strcmp(term + 2, _RMCterm))
      curSentenceType = GPS_SENTENCE_RMC;
    else if (term[0] == 'G' && strchr("PNABL", term[1]) != NULL &&
             !strcmp(term + 2, _GGAterm))
      curSentenceType = GPS_SENTENCE_GGA;
    else
      curSentenceType = GPS_SENTENCE_OTHER;

    // Any custom candidates of this sentence type?
    for (customCandidates = customElts;
         customCandidates != NULL &&
         strcmp(customCandidates->sentenceName, term) < 0;
         customCandidates = customCandidates->next)
      ;
    if (customCandidates != NULL &&
        strcmp(customCandidates->sentenceName, term) > 0)
      customCandidates = NULL;

    return false;
  }

  if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
    switch (COMBINE(curSentenceType, curTermNumber)) {
    case COMBINE(GPS_SENTENCE_RMC, 1): // Time in both sentences
    case COMBINE(GPS_SENTENCE_GGA, 1):
      time.setTime(term);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 2): // RMC validity
      sentenceHasFix = term[0] == 'A';
      break;
    case COMBINE(GPS_SENTENCE_RMC, 3): // Latitude
    case COMBINE(GPS_SENTENCE_GGA, 2):
      location.setLatitude(term);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 4): // N/S
    case COMBINE(GPS_SENTENCE_GGA, 3):
      location.rawNewLatData.negative = term[0] == 'S';
      break;
    case COMBINE(GPS_SENTENCE_RMC, 5): // Longitude
    case COMBINE(GPS_SENTENCE_GGA, 4):
      location.setLongitude(term);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 6): // E/W
    case COMBINE(GPS_SENTENCE_GGA, 5):
      location.rawNewLngData.negative = term[0] == 'W';
      break;
    case COMBINE(GPS_SENTENCE_RMC, 7): // Speed (RMC)
      speed.set(term);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 8): // Course (RMC)
      course.set(term);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 9): // Date (RMC)
      date.setDate(term);
      break;
    case COMBINE(GPS_SENTENCE_GGA, 6): // Fix data (GGA)
      sentenceHasFix = term[0] > '0';
      location.newFixQuality = (TinyGPSLocation::Quality)term[0];
      break;
    case COMBINE(GPS_SENTENCE_GGA, 7): // Satellites used (GGA)
      satellites.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GGA, 8): // HDOP
      hdop.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GGA, 9): // Altitude (GGA)
      altitude.set(term);
      break;
    case COMBINE(GPS_SENTENCE_RMC, 12):
      location.newFixMode = (TinyGPSLocation::Mode)term[0];
      break;
    }

  // Set custom values as needed
  for (TinyGPSCustom *p = customCandidates;
       p != NULL &&
       strcmp(p->sentenceName, customCandidates->sentenceName) == 0 &&
       p->termNumber <= curTermNumber;
       p = p->next)
    if (p->termNumber == curTermNumber)
      p->set(term);

  return false;
}

/* static */
double TinyGPSPlus::distanceBetween(double lat1, double long1, double lat2,
                                    double long2) {
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6371009 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * _GPS_EARTH_MEAN_RADIUS;
}

double TinyGPSPlus::courseTo(double lat1, double long1, double lat2,
                             double long2) {
  // returns course in degrees (North=0, West=270) from position 1 to position
  // 2, both specified as signed decimal-degrees latitude and longitude. Because
  // Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char *TinyGPSPlus::cardinal(double course) {
  static const char *directions[] = {"N",  "NNE", "NE", "ENE", "E",  "ESE",
                                     "SE", "SSE", "S",  "SSW", "SW", "WSW",
                                     "W",  "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

void TinyGPSLocation::commit() {
  rawLatData = rawNewLatData;
  rawLngData = rawNewLngData;
  fixQuality = newFixQuality;
  fixMode = newFixMode;
  lastCommitTime = HAL_GetTick();
  valid = updated = true;
}

void TinyGPSLocation::setLatitude(const char *term) {
  TinyGPSPlus::parseDegrees(term, rawNewLatData);
}

void TinyGPSLocation::setLongitude(const char *term) {
  TinyGPSPlus::parseDegrees(term, rawNewLngData);
}

double TinyGPSLocation::lat() {
  updated = false;
  double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
  return rawLatData.negative ? -ret : ret;
}

double TinyGPSLocation::lng() {
  updated = false;
  double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
  return rawLngData.negative ? -ret : ret;
}

void TinyGPSDate::commit() {
  date = newDate;
  lastCommitTime = HAL_GetTick();
  valid = updated = true;
}

void TinyGPSTime::commit() {
  time = newTime;
  lastCommitTime = HAL_GetTick();
  valid = updated = true;
}

void TinyGPSTime::setTime(const char *term) {
  newTime = (uint32_t)TinyGPSPlus::parseDecimal(term);
}

void TinyGPSDate::setDate(const char *term) { newDate = atol(term); }

uint16_t TinyGPSDate::year() {
  updated = false;
  uint16_t year = date % 100;
  return year + 2000;
}

uint8_t TinyGPSDate::month() {
  updated = false;
  return (date / 100) % 100;
}

uint8_t TinyGPSDate::day() {
  updated = false;
  return date / 10000;
}

uint8_t TinyGPSTime::hour() {
  updated = false;
  return time / 1000000;
}

uint8_t TinyGPSTime::minute() {
  updated = false;
  return (time / 10000) % 100;
}

uint8_t TinyGPSTime::second() {
  updated = false;
  return (time / 100) % 100;
}

uint8_t TinyGPSTime::centisecond() {
  updated = false;
  return time % 100;
}

void TinyGPSDecimal::commit() {
  val = newval;
  lastCommitTime = HAL_GetTick();
  valid = updated = true;
}

void TinyGPSDecimal::set(const char *term) {
  newval = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSInteger::commit() {
  val = newval;
  lastCommitTime = HAL_GetTick();
  valid = updated = true;
}

void TinyGPSInteger::set(const char *term) { newval = atol(term); }

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, const char *_sentenceName,
                             int _termNumber) {
  begin(gps, _sentenceName, _termNumber);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps, const char *_sentenceName,
                          int _termNumber) {
  lastCommitTime = 0;
  updated = valid = false;
  sentenceName = _sentenceName;
  termNumber = _termNumber;
  memset(stagingBuffer, '\0', sizeof(stagingBuffer));
  memset(buffer, '\0', sizeof(buffer));

  gps.insertCustom(this, _sentenceName, _termNumber);
}

void TinyGPSCustom::commit() {
  strcpy(this->buffer, this->stagingBuffer);
  lastCommitTime = HAL_GetTick();
  valid = updated = true;
}

void TinyGPSCustom::set(const char *term) {
  strncpy(this->stagingBuffer, term, sizeof(this->stagingBuffer) - 1);
}

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, const char *sentenceName,
                               int termNumber) {
  TinyGPSCustom **ppelt;

  for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next) {
    int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
    if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
      break;
  }

  pElt->next = *ppelt;
  *ppelt = pElt;
}

// EKF: static position model (F=I), direct lat/lon measurement (H=I).
// hAcc_m: 1-sigma horizontal accuracy in metres used to build R.
static void apply_ekf(gps_t *g, float hAcc_m) {
    static const float Q_VAL    = 1e-10f;  // ponytail: tune up if filter too slow to track motion
    static const float DEG_PER_M = 1.0f / 111111.0f;

    if (g->ekf.x[0] == 0.0f) {
        g->ekf.x[0] = (float)g->snapshot.lat;
        g->ekf.x[1] = (float)g->snapshot.lon;
        return;
    }

    float fx[2] = { g->ekf.x[0], g->ekf.x[1] };
    float F[4]  = { 1,0, 0,1 };
    float Q[4]  = { Q_VAL,0, 0,Q_VAL };
    ekf_predict(&g->ekf, fx, F, Q);

    float z[2]  = { (float)g->snapshot.lat, (float)g->snapshot.lon };
    float hx[2] = { g->ekf.x[0], g->ekf.x[1] };
    float H[4]  = { 1,0, 0,1 };
    float sigma = hAcc_m * DEG_PER_M;
    float r     = sigma * sigma;
    float R[4]  = { r,0, 0,r };

    if (ekf_update(&g->ekf, z, hx, H, R)) {
        g->snapshot.lat = g->ekf.x[0];
        g->snapshot.lon = g->ekf.x[1];
    }
}

extern "C" {

void gps_init(gps_t *g, int type, UART_HandleTypeDef *huart, bool use_ekf) {
    g->huart       = huart;
    g->type        = type;
    g->frame_ready = false;
    g->use_ekf     = use_ekf;
    memset(&g->ubx, 0, sizeof(g->ubx));

    huart->Init.BaudRate = (type == GPS_UBX) ? 115200 : 9600;
    HAL_UART_Init(huart);

    if (type == GPS_NMEA)
        g->nmea = new TinyGPSPlus();

    if (use_ekf) {
        // Large initial P so first measurement dominates
        float pdiag[2] = { 1.0, 1.0 };
        ekf_initialize(&g->ekf, pdiag);
    }
}

bool gps_process(gps_t *g, uint8_t byte) {
    if (g->type == GPS_UBX) {
        ubx_nav_pvt_t pvt;
        if (!ubx_process(&g->ubx, &pvt, byte)) return false;
        
        // Check for succesful fix
        if (pvt.fixType < 2 || !(pvt.flags & 0x01)) return false;
        g->snapshot.lat     = pvt.lat     * 1e-7;
        g->snapshot.lon     = pvt.lon     * 1e-7;
        g->snapshot.alt     = pvt.hMSL    * 1e-3;
        g->snapshot.gSpeed  = pvt.gSpeed  * 1e-3;
        g->snapshot.headMot = pvt.headMot * 1e-5;
        g->snapshot.numSV   = pvt.numSV;
        g->snapshot.fixType = pvt.fixType;
        if (g->use_ekf) apply_ekf(g, pvt.hAcc * 1e-3f);
    } else {
        TinyGPSPlus *nmea = (TinyGPSPlus *)g->nmea;

        // Check for succesful fix
        if (!nmea->encode((char)byte) || !nmea->location.isValid()) return false;
        g->snapshot.lat     = nmea->location.lat();
        g->snapshot.lon     = nmea->location.lng();
        g->snapshot.alt     = nmea->altitude.meters();
        g->snapshot.gSpeed  = nmea->speed.mps();
        g->snapshot.headMot = nmea->course.deg();
        g->snapshot.numSV   = (int)nmea->satellites.value();
        g->snapshot.fixType = (nmea->location.FixQuality() != TinyGPSLocation::Invalid) ? 3 : 0;
        // 5 m/HDOP baseline; upgrade to per-sentence accuracy if receiver provides it
        if (g->use_ekf) {
            float hAcc_m = nmea->hdop.isValid() ? (nmea->hdop.value() / 100.0f) * 5.0f : 10.0f;
            apply_ekf(g, hAcc_m);
        }
    }
    g->frame_ready = true;
    return true;
}

bool gps_read_snapshot(gps_t *g, gps_data_t *out) {
    if (!g->frame_ready) return false;
    __disable_irq();
    g->frame_ready = false;
    *out = g->snapshot;
    __enable_irq();
    return true;
}

bool gps_read_combined(gps_t *a, gps_t *b, gps_data_t *out) {
    gps_data_t da, db;
    bool ok_a = gps_read_snapshot(a, &da);
    bool ok_b = gps_read_snapshot(b, &db);

    if (!ok_a && !ok_b) return false;
    if (!ok_a) { *out = db; return true; }
    if (!ok_b) { *out = da; return true; }

    float wa = (float)da.numSV, wb = (float)db.numSV, wt = wa + wb;
    if (wt == 0.0f) { *out = da; return true; }

    out->lat     = (da.lat     * wa + db.lat     * wb) / wt;
    out->lon     = (da.lon     * wa + db.lon     * wb) / wt;
    out->alt     = (da.alt     * wa + db.alt     * wb) / wt;
    out->gSpeed  = (da.gSpeed  * wa + db.gSpeed  * wb) / wt;
    out->headMot = (da.headMot * wa + db.headMot * wb) / wt;
    out->numSV   = da.numSV + db.numSV;
    out->fixType = (da.fixType < db.fixType) ? da.fixType : db.fixType;
    return true;
}

}