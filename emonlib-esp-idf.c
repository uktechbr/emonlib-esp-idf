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

#include <math.h>
#include "esp_log.h"
#include "emonlib-esp-idf.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"


static const char *TAG = "emonlib";

//Calibration coefficients
double VCAL, ICAL, PHASECAL;

int sampleV, sampleI;

double lastFilteredV,filteredV;          
double filteredI;
double offsetV;                          
double offsetI;                          

double phaseShiftedV;                             

double sqV,sumV,sqI,sumI,instP,sumP;              

int startV;                                      

bool lastVCross, checkVCross;                  

esp_adc_cal_characteristics_t *adc_chars;


unsigned long  millisec()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

esp_err_t emonlib_init(double _VCAL, double _PHASECAL, double _ICAL) {
  ESP_LOGD(TAG, "Calibration values: VCal = %f; PhaseCal = %f; ICal = %f", _VCAL, _PHASECAL, _ICAL);
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;

  adc1_config_channel_atten(CONFIG_EMONLIB_VOLTAGE_SENSOR_ADC1_CHANNEL, ADC_ATTEN_DB_11);//Configura a atenuacao
  adc1_config_channel_atten(CONFIG_EMONLIB_CURRENT_SENSOR_ADC1_CHANNEL, ADC_ATTEN_DB_11);//Configura a atenuacao
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, adc_chars);
  return ESP_OK;
}


esp_err_t emonlib_calc_vi(unsigned int crossings, unsigned int timeout, emonlib_esp_idf_data_t *data) {

  int SupplyVoltage = CONFIG_EMONLIB_SUPPLY_VOLTAGE;
  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millisec();    //millisec()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(1)                                   //the while loop...
  {
    startV = esp_adc_cal_raw_to_voltage(adc1_get_raw(CONFIG_EMONLIB_VOLTAGE_SENSOR_ADC1_CHANNEL), adc_chars);                    //using the voltage waveform
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
    if ((millisec() - start)>timeout) break;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millisec();

  while ((crossCount < crossings) && ((millisec()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = esp_adc_cal_raw_to_voltage(adc1_get_raw(CONFIG_EMONLIB_VOLTAGE_SENSOR_ADC1_CHANNEL), adc_chars);                 //Read in raw voltage signal
    sampleI = esp_adc_cal_raw_to_voltage(adc1_get_raw(CONFIG_EMONLIB_CURRENT_SENSOR_ADC1_CHANNEL), adc_chars);               //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/ADC_COUNTS);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/ADC_COUNTS);
    filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) {
      checkVCross = true;
    } else {
      checkVCross = false;
    }

    if (numberOfSamples == 1) {
      lastVCross = checkVCross;
    }

    if (lastVCross != checkVCross) {
      crossCount++;
    }
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  data->Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  data->Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  //Calculation power values
  data->realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  data->apparentPower = data->Vrms * data->Irms;
  data->powerFactor = data->realPower / data->apparentPower;

  ESP_LOGD(TAG, "VRMS = %f; IRMS = %f; Real Power = %f; Apparent Power = %f, Power Factor = %f", data->Vrms, data->Irms, data->realPower, data->apparentPower, data->powerFactor);

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;

  return ESP_OK;
}
