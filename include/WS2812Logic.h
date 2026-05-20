#pragma once
#include <inttypes.h>
#include <DrakePinD.hpp>

#define DISPLAY_WIDTH		128		// переименовать в FRAME_ OR NOT?
#define DISPLAY_HEIGHT		16
#include <FrameBuffer.h>
#include <FrameManager.h>
#include <effects/FrameEffectRearLights.h>

namespace WS2812
{
	
	FrameBuffer buffer;
	FrameManager manager(buffer);
	FrameEffectRearLights effect;

	uint8_t *frame_buffer_ptr;
	uint16_t frame_buffer_len;

	
	// Управление питами контроля DC-DC на 5 вольт (для ws2812 или hub75 панелей)
	DrakePinD Dc1En({GPIOD, GPIO_PIN_0}, DrakePin::Output, DrakePin::Low);
	DrakePinD Dc2En({GPIOD, GPIO_PIN_14}, DrakePin::Output, DrakePin::Low);

	// Выходы на ws2812
	// GPIOD, GPIO_PIN_12
	// GPIOD, GPIO_PIN_13
	
	
	inline void Setup()
	{
		Dc1En.Init();
		Dc2En.Init();



		buffer.SetBrightness(64);
		buffer.SetColorCorrection(255, 211, 167);
		frame_buffer_ptr = buffer.frame_buffer.raw;
		frame_buffer_len = sizeof(buffer.frame_buffer.raw);

		manager.SelectEffect(effect);


		effect.AddLayer(0, "layer0.pxl");	// 0 - Фон / Заливка;
		effect.AddLayer(1, "layer1.pxl");	// 1 - Анимация;
		effect.AddLayer(2, "layer2.pxl");	// 2 - Габариты;
		effect.AddLayer(3, "layer3.pxl");	// 3 - Задних ход;
		effect.AddLayer(4, "layer4.pxl");	// 4 - Стопы;
		effect.AddLayer(5, "layer5.pxl");	// 5 - Повтороты лево;
		effect.AddLayer(6, "layer6.pxl");	// 6 - Повтороты право;
		effect.AddLayer(7, "layer7.pxl");	// 7 - Аварийка;

		effect.ShowLayer(0);
		effect.ShowLayer(1);

		effect.HideLayer(5);
		
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		manager.Tick(current_time);

		static uint32_t lasttime = 0;
		if(buffer.DrawIsReady() == true)
		{
			buffer.DrawBegin();

			//Logger.Print("+PXL=128,16,6144,");
			//Logger.Print(frame_buffer_ptr, frame_buffer_len, LOG_OUT_TYPE_BYTES);
			//Logger.Print("\n");

			buffer.DrawEnding();

			//DEBUG_LOG_TOPIC("DMADraw", "time: %d\n", (HAL_GetTick() - lasttime));
			lasttime = HAL_GetTick();
		}
		
		
		current_time = HAL_GetTick();
		return;
	}
}
