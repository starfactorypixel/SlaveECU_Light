#pragma once
#include <inttypes.h>
#include <DrakePinD.hpp>

namespace WS2812
{

	// Управление питами контроля DC-DC на 5 вольт (для ws2812 или hub75 панелей)
	DrakePinD Dc1En({GPIOD, GPIO_PIN_0}, DrakePin::Output, DrakePin::Low);
	DrakePinD Dc2En({GPIOD, GPIO_PIN_14}, DrakePin::Output, DrakePin::Low);
	
	
	inline void Setup()
	{
		Dc1En.Init();
		Dc2En.Init();
		
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		
		
		current_time = HAL_GetTick();
		return;
	}
}
