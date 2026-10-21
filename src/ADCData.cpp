/*
 * ADCData.cpp
 *
 *  Created on: Jan 4, 2021
 *      Author: Eric Chung
 */

#include "ADCData.h"
#include <DAVE.h>
#include "math.h"

#define FILTER_COUNT	5


ADCData::ADCData()
{
	// TODO Auto-generated constructor stub
	ADC_Done = 0;

	TempIin = 0;
	TempIout = 0;
	TempTemperature = 0;
	TempVin = 0;
	TempVref = 0;

	xFactor = 0;
	AvgxFactor = 0;
	Vin = 0;
	AvgVin = 0;
	Iin = 0;
	Iout = 0;

	TemperatureCounter = 0;
	TempBValue = 3977.0; //value get from thermistor datasheet
}

ADCData::~ADCData()
{
	// TODO Auto-generated destructor stub
}

void ADCData::ADC_GetResult(void)
{
	/*Read out conversion results*/
	TempIin = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_IIN_handle);
	TempTemperature = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_TEMP_handle);
	TempVin = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_VIN_handle);
	TempVref = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_ADC_VREF_handle);
	TempIout = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_ADC_IOUT_handle);
}

float ADCData::ADC_CalculateAverage(float new_value, float current_avg_value, int filter)
{
	return (current_avg_value * ((float) filter - 1) + new_value) / filter;
}

void ADCData::ADC_CalculateResult(void)
{
	xFactor = 2.5 / TempVref;

	if (AvgxFactor == 0.0)
	{
		AvgxFactor = xFactor;
	}

	AvgxFactor = ADC_CalculateAverage(xFactor, AvgxFactor, FILTER_COUNT);

	Vin = TempVin * AvgxFactor;
	Vin = Vin / 0.0033;

	if (Vin == 0.00)
	{
		Vin = 1.0;
	}

	if (AvgVin == 0.0)
	{
		AvgVin = Vin;
	}
	AvgVin = ADC_CalculateAverage(Vin, AvgVin, FILTER_COUNT);

	if (true)
	{
		//50mv per 1a
		Iin = TempIin * AvgxFactor - 0.5;
		Iin /= 0.05;
	}
	
	else
	{
		//40mv per 1a
		Iin = TempIin * AvgxFactor - 0.5;
		Iin /= 0.04;
	}

	Iout = TempIout * AvgxFactor - 0.5;
	Iout /= 0.04;

	if (TemperatureCounter < 10)
	{
		TemperatureCounter++;
	}
	else
	{
		float x = log(TempTemperature / (4095.0 - TempTemperature));
		float tempData = 1.0 / (25.0 + 273.15) + 1.0 / TempBValue * x;
		Temperature = 1.0 / tempData - 273.15;

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
}


