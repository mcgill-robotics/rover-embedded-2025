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

#include "stm32g4xx_hal.h"
#include <cmath>
#include <cstdlib>
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

  // Insert this item into the GPS tree
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

extern "C" {
struct Time {
   uint16_t year;
   uint8_t month;
   uint8_t day;
   uint8_t hour;
   uint8_t minute;
   uint8_t second;
   uint8_t centisecond;
};

struct combined_gps_data {
  double lat;
  double lng;
  double course;
  double kmph;
  double alt;
  int satellites;
  int valid_gps_count;
  double bearing;
};

struct single_gps_data {
  double lat;
  double lng;
  double course;
  double kmph;
  double alt;
  int satellites;
  struct Time time;
  double hdop;
  char fix_mode;
  char fix_quality;
};

// limits for prefilter
static int max_hdop = 10;
static int max_kmph = 50;

// 0 = averaging
// 1 = averaging + prefilter
// 3 = averaging + prefilter + advanced filter
static int filter_mode = 0;

// 0 = gps1
// 1 = gps2
// -1 = no select
static int gps_selector = -1;

static TinyGPSPlus *gps[2] = {0};


const struct Time gps_time(int g) {
  return (Time){
      .year =         gps[g]->date.year(),
      .month =        gps[g]->date.month(),
      .day =          gps[g]->date.day(),
      .hour =         gps[g]->time.hour(),
      .minute =       gps[g]->time.minute(),
      .second =       gps[g]->time.second(),
      .centisecond =  gps[g]->time.centisecond(),
  };
}

// Returns a TinyGPSPlus class as void pointer
void gps_init() { 
  gps[0] = new TinyGPSPlus();
  gps[1] = new TinyGPSPlus();
}

// Encode one character at a time
bool gps_encode(int g, char c) {
  return gps[g]->encode(c);
}


bool gps_parse(int g, struct single_gps_data *data) {
  if (g < 0 || g > 1 || data == NULL)
    return false;
  bool valid = true;

  if (filter_mode > 0) {
    if (!gps[g]->location.isValid())
      valid = false;
    if (!gps[g]->speed.isValid())
      valid = false;
    if (!gps[g]->altitude.isValid())
      valid = false;
    if (!gps[g]->hdop.isValid())
      valid = false;
    if (!gps[g]->satellites.isValid())
      valid = false;
    if (!gps[g]->course.isValid())
      valid = false;
  }

  *data = (struct single_gps_data) {
    .lat = gps[g]->location.lat(), 
    .lng = gps[g]->location.lng(), 
    .course = gps[g]->course.deg(), 
    .kmph = gps[g]->speed.kmph(), 
    .alt = gps[g]->altitude.meters(), 
    .satellites = gps[g]->satellites.value(),
    .time = gps_time(g),
    .hdop = gps[g]->hdop.hdop(),
    .fix_mode = gps[g]->location.FixMode(),
    .fix_quality = gps[g]->location.FixQuality(),
  };
  return valid;
}

static double bearing(double alng, double alat, double blng, double blat) {
  double dlng = blng - alng;
  double dlat = blat - alat;
  double bearing = atan2(dlng, dlat) * RAD_TO_DEG;
  if (bearing < 0)
    bearing += 360;
  return bearing;
}

static double gps_bearing(struct single_gps_data *g1, struct single_gps_data *g2) {
  return bearing(g1->lng, g1->lat, g2->lng, g2->lat);
}

static bool gps_prefilter(struct single_gps_data data) {
  // 1 = ideal
  // 2-5 = excellent
  // 5-10 = moderate
  // 10-20 = not accurate
  // 20+ = don't trust at all
  if (data.hdop > max_hdop)
    return false;

  if (data.fix_mode == 'N')
    return false;

  if (data.fix_quality == '0')
    return false;

  // If exceeding max rover speed, drop data as being inaccurate
  if (data.kmph > max_kmph)
    return false;

  return true;
}

static void gps_combine(struct single_gps_data &g1, struct single_gps_data &g2,
                 struct combined_gps_data *output) {
    // Just averaging for now
    // Filtering should be set up in here
    output->lat = (g1.lat + g2.lat) / 2.0;
    output->lng = (g1.lng + g2.lng) / 2.0;
    output->alt = (g1.alt + g2.alt) / 2.0;
    output->course = (g1.course + g2.course) / 2.0;
    output->kmph = (g1.kmph + g2.kmph) / 2.0;
    output->satellites = (g1.satellites + g2.satellites) / 2.0;
    output->valid_gps_count = 2;
    output->bearing = gps_bearing(&g1, &g2);
}

// 0 = gps1, 1 = gps2
bool gps_process(char *buf, int g, char c) {
  static struct single_gps_data gps_data[2] = {0};
  static bool gps_valid[2] = {true, true};

  // No new data
  if (!gps_encode(g, c) || (gps_selector >= 0 && gps_selector != g))
    return false;
  
  // Parse and prefilter
  gps_valid[g] = gps_parse(g, &gps_data[g]);
  if (gps_valid[g] && filter_mode >= 1) {
    gps_valid[g] = gps_prefilter(gps_data[g]);
  }

  // New data is invalid, no reason to continue
  if (!gps_valid[g])
    return false;

  struct combined_gps_data ret_data = {0};

  // If there is no gps selector and both data valid
  if (gps_selector < 0 && gps_valid[0] && gps_valid[1]) {
      gps_combine(gps_data[0], gps_data[1], &ret_data);
      // Otherwise if only gps g is valid or selector points to g
      // We don't consider the other option since in this case the new data comes from g
  } else if (gps_selector == g || (gps_valid[g] && !gps_valid[!g])) {
    ret_data = {
        .lat = gps_data[g].lat,
        .lng = gps_data[g].lng,
        .course = gps_data[g].course,
        .kmph = gps_data[g].kmph,
        .alt = gps_data[g].alt,
        .satellites = gps_data[g].satellites,
        .valid_gps_count = 1,
        .bearing = -1, // Should prob try to calculate
    };
  }
  
  // Sending only basic gps data - similar to what was sent from old pantilt
  sprintf(buf, "%d,%f,%f\n", ret_data.satellites, ret_data.lng, ret_data.lat);

  // Send complete gps data
  /*
  sprintf(buf, "%d, %f, %f, %f, %f,%f, %d, %f\n", 
    ret_data.satellites, ret_data.lng, ret_data.lat, ret_data.course, ret_data.kmph, 
    ret_data.alt, ret_data.valid_gps_count, ret_data.bearing);
    */
  return true;
}

static void gps_set_max_hdop(int hdop) { max_hdop = hdop; }

static void gps_set_max_kmph(int kmph) { max_kmph = kmph; }

static void gps_set_filter_mode(int mode) {
  if (mode < 0)
    filter_mode = 0;
  else if (mode > 2)
    filter_mode = 2;
  else
    filter_mode = mode;
}

static void gps_set_selector(int g) {
  if (g == 0)
    gps_selector = g;
  else if (g == 1)
    gps_selector = g;
  else
   gps_selector = -1;

}

bool gps_command(const char *buf, int bufz) {
  const char *kmph_cmd = "kmph ";
  const char *hdop_cmd = "hdop ";
  const char *filter_cmd = "filter ";
  const char *gps_cmd = "gps ";

  if (strncmp(buf, kmph_cmd, strlen(kmph_cmd)) == 0) {
    max_kmph = strtol(buf + strlen(kmph_cmd), NULL, 10);
    return true;
  } else if (strncmp(buf, hdop_cmd, strlen(hdop_cmd)) == 0) {
    max_hdop = strtol(buf + strlen(hdop_cmd), NULL, 10);
    return true;
  } else if (strncmp(buf, filter_cmd, strlen(filter_cmd)) == 0) {
    gps_set_filter_mode(strtol(buf+strlen(filter_cmd), NULL, 10));
    return true;
  } else if (strncmp(buf, gps_cmd, strlen(gps_cmd)) == 0) {
    gps_set_selector(strtol(buf+strlen(gps_cmd),NULL,10));
    return true;
  }

  return false;
}

}