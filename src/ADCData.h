/*
 * ADCData.h
 *
 *  Created on: Jan 4, 2021
 *      Author: Eric Chung
 */

#ifndef ADCDATA_H_
#define ADCDATA_H_

#include <stdint.h>
#include <math.h>
#include <DAVE.h>

struct Telemetry1 {
    float vIn    = 0.0f;
    float iIn    = 0.0f;
    float iOut   = 0.0f;
    float iTrack = 0.0f;
    float vRef   = 0.0f;
    float tempC  = 0.0f;
};

#ifdef __cplusplus
extern "C"
{
#endif

	class ADCData
	{
		private:
			float avgXFactor  = 0.0f;
			float avgVin      = 0.0f;
			uint8_t tempCount = 0;
			float lastTempC   = 25.0f;

			inline float calculateAverage(float newVal, float currentAvg, int count) {
				if (currentAvg == 0.0f) return newVal;
				return (currentAvg * (static_cast<float>(count) - 1.0f) + newVal) / static_cast<float>(count);
			}
		public:
			//temporary value where we store raw value from the ADC callback
			unsigned long TempIin;
			unsigned long TempTemperature;
			unsigned long TempVin;
			unsigned long TempVref;
			unsigned long TempIout;

			//value after conversion into real world value
			float xFactor;
			float AvgxFactor;
			float Vin;
			float AvgVin;
			float Iin;
			float Iout;
			float Iout_Diff;
			float Iout_Prev;

			float Temperature;
			int TemperatureCounter;
			float TempBValue; //b value of thermistor, get from datasheet

			bool ADC_Done;

			void ADC_GetResult(void);
			void ADC_CalculateResult(void);
			float ADC_CalculateAverage(float,float,int);

			inline Telemetry1 readAndProcess1(bool dip3State) {
			uint16_t rawVref   = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_VREF_handle);
			uint16_t rawIin    = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_IIN_handle);
			uint16_t rawIout   = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_ADC_IOUT_handle);
			uint16_t rawVin    = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_VIN_handle);
			uint16_t rawTemp   = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_TEMP_handle);

			Telemetry1 data;

			xFactor = 2.5 / rawVref;

			if (AvgxFactor == 0.0)
			{
				AvgxFactor = xFactor;
			}

			AvgxFactor = ADC_CalculateAverage(xFactor, AvgxFactor, 5);

			Vin = rawVin * AvgxFactor;
			Vin = Vin / 0.0033;

			if (Vin == 0.00)
			{
				Vin = 1.0;
			}

			if (AvgVin == 0.0)
			{
				AvgVin = Vin;
			}
			data.vIn = ADC_CalculateAverage(Vin, data.vIn, 5);

			if (dip3State == true)
			{
				//50mv per 1a
				data.iIn = rawIin * AvgxFactor - 0.5;
				data.iIn /= 0.04;
			}
			
			else
			{
				//40mv per 1a
				data.iIn = rawIin * AvgxFactor - 0.5;
				data.iIn /= 0.04;
			}

			data.iOut = rawIout * AvgxFactor - 0.5;
			data.iOut /= 0.04;

			if (TemperatureCounter < 10)
			{
				TemperatureCounter++;
			}
			else
			{
				float x = log(rawTemp / (4095.0 - rawTemp));
				float tempData = 1.0 / (25.0 + 273.15) + 1.0 / TempBValue * x;
				data.tempC = 1.0 / tempData - 273.15;

				TemperatureCounter = 0;
			}

			ADC_Done = true;

			/*if (WPT_Controller1.rowToUpdate == 3 && WPT_Controller1.DischargingFlag == false)
			{
				Iout_Diff = Iout - Iout_Prev;
				Iout_Prev = Iout;
			}
			if (WPT_Controller1.StateChange == true)
			{
				if(Iout > WPT_Controller1.Iout_Limit)
				{
					Iout_Diff = 0;
					WPT_Controller1.StateChange = false;
				}
			}*/
			return data;
		}

			ADCData();
			virtual ~ADCData();
	};

#ifdef __cplusplus
}
#endif

#endif /* ADCDATA_H_ */
