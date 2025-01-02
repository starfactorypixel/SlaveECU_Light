#pragma once
#include <AnalogMux.h>
#include <EasyPinA.h>
#include <CUtils.h>

extern ADC_HandleTypeDef hadc1;

namespace Analog
{
	uint16_t OnMuxRequest(uint8_t address);
	void OnMuxResponse(uint8_t address, uint16_t value);
	
	EasyPinA adc_pin(&hadc1, GPIOB, GPIO_PIN_1, ADC_CHANNEL_9, ADC_SAMPLETIME_8CYCLES_5);
	volt_calc_t VoltCalcParams = {((1 << 12) - 1), 3324, 82000, 10000, 17};
	
	AnalogMux<0> mux( OnMuxRequest, OnMuxResponse
	/*,
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_4}, 
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_5}, 
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_6}, 
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_7}
	*/
	);
	
	
	uint16_t OnMuxRequest(uint8_t address)
	{
		return adc_pin.Get();
	}
	
	void OnMuxResponse(uint8_t address, uint16_t value)
	{
		if(address == 0)
		{
			uint16_t vin = VoltageCalculate(value, VoltCalcParams);
			uint8_t *vin_bytes = (uint8_t *)&vin;
			
			CANLib::obj_block_health.SetValue(0, vin_bytes[0]);
			CANLib::obj_block_health.SetValue(1, vin_bytes[1]);
		}
		
		return;
	}
	
	inline void Setup()
	{
		adc_pin.Init();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		mux.Processing(current_time);
		
		static uint32_t tick1000 = 0;
		if(current_time - tick1000 > 1000)
		{
			tick1000 = current_time;
			
			// Раз в минуту запускаем калибровку ADC
			static uint8_t adc_calibration = 0;
			if(++adc_calibration >= 60)
			{
				adc_calibration = 0;

				adc_pin.Calibration();
			}
		}
		
		// При выходе обновляем время
		current_time = HAL_GetTick();
		
		return;
	}
};
