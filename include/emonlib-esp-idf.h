/* Emonlib-ESP-IDF
 * Copyright (C) 2022 Uhlig & Korovsky Tecnologia Ltda
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __EMONLIB_ESP_IDF_H__
#define __EMONLIB_ESP_IDF_H__

#include "esp_err.h"

typedef struct {
    double Vrms;
    double Irms;
    double realPower;
    double apparentPower;
    double powerFactor;
} emonlib_esp_idf_data_t;

#define READVCC_CALIBRATION_CONST 1126400L
#define ADC_BITS    12

#define ADC_COUNTS  (1<<ADC_BITS)

esp_err_t emonlib_init(double _VCAL, double _PHASECAL, double _ICAL);
esp_err_t emonlib_calc_vi(unsigned int crossings, unsigned int timeout, emonlib_esp_idf_data_t *data);

#endif