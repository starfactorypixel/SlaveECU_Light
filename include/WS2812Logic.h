#pragma once
#include <inttypes.h>
#include <DrakePinD.hpp>
#include <WS2812HW.h>

#define DISPLAY_WIDTH		128		// переименовать в FRAME_ OR NOT?
#define DISPLAY_HEIGHT		16
#include <FrameBuffer.h>
#include <FrameManager.h>
#include <effects/FrameEffectRearLights.h>

namespace WS2812
{


	uint8_t *frame_buffer_ptr;
	uint16_t frame_buffer_len;
	volatile uint16_t frame_buffer_idx = 0;




	
	FrameBuffer buffer;
	FrameManager manager(buffer);
	FrameEffectRearLights effect;



	
	// Управление питами контроля DC-DC на 5 вольт (для ws2812 или hub75 панелей)
	DrakePinD Dc1En({GPIOD, GPIO_PIN_0}, DrakePin::Output, DrakePin::Low);
	DrakePinD Dc2En({GPIOD, GPIO_PIN_14}, DrakePin::Output, DrakePin::Low);

	// Выходы на ws2812
	// GPIOD, GPIO_PIN_12
	// GPIOD, GPIO_PIN_13








	// Маппер без маппинга
	static inline uint16_t mapper_0(uint16_t input)
	{
		return input;
	}
	
	// Маппер последовательной панели, Линейное подлючение, Один ряд
	// +----+----+----+----+----+----+
	// | 01 | 02 | 03 | 04 | 05 | XX |
	// +----+----+----+----+----+----+	
	static inline uint16_t mapper_1(uint16_t input)
	{
		static uint16_t width = DISPLAY_WIDTH;
		static uint16_t height = DISPLAY_HEIGHT;
		static uint8_t color_map[] = {1, 0, 2, 0};

		uint16_t pixelIndex = input / 3;
		uint16_t row = pixelIndex % height;		// Номер пикселя в зиг-заге
		uint16_t col = pixelIndex / height;		// Номер столбца зиг-зага

		uint16_t rowTransformed = (col & 1) ? (height - row - 1) : row;
		uint16_t index = rowTransformed * width + col;

		return (index * 3) + color_map[(input - pixelIndex * 3)];
	}

	// Маппер последовательной панели, Построчного подключения, Несколько рядов
	// +----+----+----+----+
	// | 01 | 02 | 03 | 04 |
	// +----+----+----+----+
	// | 05 | 06 | 07 | 08 |
	// +----+----+----+----+
	// | 09 | 10 | 11 | 12 |
	// +----+----+----+----+
	static inline uint16_t mapper_2(uint16_t input)
	{
		static const uint16_t width = DISPLAY_WIDTH;
		static const uint16_t height = DISPLAY_HEIGHT;
		static const uint16_t block_height = 16;
		static const uint8_t color_map[] = {1, 0, 2, 0};
		
		// Номер пикселя
		uint16_t pixelIndex = input / 3;

		// Идём по «столбцам»
		uint16_t row = pixelIndex % block_height;				// 0..15
		uint16_t col = (pixelIndex / block_height) % width;		// 0..63
		uint16_t block = pixelIndex / (width * block_height);	// 0..2

		// Зигзаг по строкам в столбце
		uint16_t rowTransformed = (col & 1) 
			? (block_height - 1 - row) 
			: row;
		
		// Номер строки на экране (0..47)
		uint16_t transformed_row = (block * block_height) + rowTransformed;

		// Линейный индекс пикселя на физическом экране
		uint16_t index = transformed_row * width + col;

		// Смещение по компоненте
		return (index * 3) + color_map[(input - (pixelIndex * 3))];
	}

	
	
	
	


	typedef uint16_t (*idx_mapper_ptr)(uint16_t idx);
	idx_mapper_ptr mapper_func = mapper_1;
	



//__attribute__((section(".RAM_D2"))) 
volatile uint16_t dma_buffer[ (8 * 3 * 6) ];		// 8 бит * 3 цвета * 6 пикселя.
static const uint16_t dma_buffer_len = sizeof(dma_buffer) / sizeof(dma_buffer[0]);











void CreateDMABuffer(uint8_t mode)
{
	using namespace WS2812WH;

	if(frame_buffer_idx >= frame_buffer_len) return;

	//Leds::obj.SetOn(Leds::LED_WHITE);
	
	static uint16_t buff_copy_logic[3][2] = 
	{
		{0, dma_buffer_len}, 
		{0, (dma_buffer_len / 2)}, 
		{(dma_buffer_len / 2), dma_buffer_len}
	};	
	uint16_t start = buff_copy_logic[mode][0];
	uint16_t end = buff_copy_logic[mode][1];
	
	//uint8_t *frame_ptr = &frame_buffer_ptr[frame_buffer_idx];
	uint8_t byte, mask;
	uint16_t index;
	
	for(uint16_t i = start; i < end; i += 8)
	{
		//byte = *frame_ptr++;
		index = mapper_func(frame_buffer_idx++);
		byte = frame_buffer_ptr[index];
		mask = 0x80;
		
		for(uint8_t b = 0; b < 8; ++b)
		{
			dma_buffer[i + b] = (byte & mask) ? WS2812WH::PWM_HI : WS2812WH::PWM_LO;
			mask >>= 1;
		}
	}
	//frame_buffer_idx += (end - start) / 8;
	
	//Leds::obj.SetOff(Leds::LED_WHITE);
}

void CreateDMABuffer2(uint8_t mode)
{
    using namespace WS2812WH;

	if(frame_buffer_idx >= frame_buffer_len) return;

	//DEBUG_LOG_TOPIC("IF", "%d >= %d\n", frame_buffer_idx, frame_buffer_len);

    uint16_t start = 0;
    uint16_t end = dma_buffer_len;

    if(mode == 1)
    {
        end = dma_buffer_len / 2;
    }
    else if(mode == 2)
    {
        start = dma_buffer_len / 2;
    }

    uint16_t *buf = (uint16_t*)dma_buffer;

    for(uint16_t i = start; (i + 7) < end; i += 8)
    {
		if(frame_buffer_idx >= frame_buffer_len) return;	// Обазательно

        uint32_t fb_idx = frame_buffer_idx++;

        uint16_t index = mapper_func(fb_idx);

        uint8_t byte = frame_buffer_ptr[index];

		if(((i + 7) >= 144))
		{
			//Leds::obj.SetOn(Leds::LED_RED);
			DEBUG_LOG_TOPIC("", "%d, %d, %d\n", i, (i + 7), mode);
		}

        buf[i + 0] = (byte & 0x80) ? PWM_HI : PWM_LO;
        buf[i + 1] = (byte & 0x40) ? PWM_HI : PWM_LO;
        buf[i + 2] = (byte & 0x20) ? PWM_HI : PWM_LO;
        buf[i + 3] = (byte & 0x10) ? PWM_HI : PWM_LO;
        buf[i + 4] = (byte & 0x08) ? PWM_HI : PWM_LO;
        buf[i + 5] = (byte & 0x04) ? PWM_HI : PWM_LO;
        buf[i + 6] = (byte & 0x02) ? PWM_HI : PWM_LO;
        buf[i + 7] = (byte & 0x01) ? PWM_HI : PWM_LO;
    }
}

	void Stop();

	static void My_FullCpltCallback(DMA_HandleTypeDef *hdma)
	{
		if (hdma != &hdma_tim4_ch1) return;

		//Leds::obj.SetOn(Leds::LED_RED);

		if(frame_buffer_idx < frame_buffer_len)
		{
			CreateDMABuffer(2);
		}
		else if(frame_buffer_idx < frame_buffer_len + (dma_buffer_len / 8))
		{
			//DEBUG_LOG_TOPIC("q1", "%d\n", frame_buffer_idx);
			memset((uint8_t *) &dma_buffer[dma_buffer_len / 2], 0x00, (dma_buffer_len / 2) * sizeof(uint16_t));
			frame_buffer_idx += (dma_buffer_len / 2 / 8);
			//DEBUG_LOG_TOPIC("q2", "%d\n", frame_buffer_idx);
		}
		else
		{
			//DEBUG_LOG_TOPIC("", "STOP %d\n", frame_buffer_idx);
			Stop();
		}

		//Leds::obj.SetOff(Leds::LED_RED);
	}

	static void My_HalfCpltCallback(DMA_HandleTypeDef *hdma)
	{
		if (hdma != &hdma_tim4_ch1) return;

		//Leds::obj.SetOn(Leds::LED_WHITE);


		if(frame_buffer_idx < frame_buffer_len)
		{
			CreateDMABuffer(1);
		}
		else if(frame_buffer_idx < frame_buffer_len + (dma_buffer_len / 8))
		{
			//DEBUG_LOG_TOPIC("q3", "%d\n", frame_buffer_idx);
			memset((uint8_t *) &dma_buffer[0], 0x00, (dma_buffer_len / 2) * sizeof(uint16_t));
			frame_buffer_idx += (dma_buffer_len / 2 / 8);
			//DEBUG_LOG_TOPIC("q4", "%d\n", frame_buffer_idx);
		}
		else
		{
			//DEBUG_LOG_TOPIC("", "STOP %d\n", frame_buffer_idx);
			Stop();
		}

		//Leds::obj.SetOff(Leds::LED_WHITE);
	}


	void Start()
	{
		buffer.DrawBegin();
		
		frame_buffer_idx = 0;
		CreateDMABuffer(0);
		
		//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);												//// нужно?
		HAL_StatusTypeDef stat = HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)dma_buffer, (uint32_t)&TIM4->CCR1, dma_buffer_len);
		
		if(stat == HAL_OK)
		{
			__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);
		}
	}

	void Stop()
	{
		//HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);

		__HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_CC1);
		HAL_DMA_Abort_IT(&hdma_tim4_ch1);

		frame_buffer_idx = 0;
		
		buffer.DrawEnding();
	}


	
	
	inline void Setup()
	{
		Dc1En.Init();
		Dc2En.Init();

		WS2812WH::Setup();
		HAL_DMA_RegisterCallback(&hdma_tim4_ch1, HAL_DMA_XFER_CPLT_CB_ID, My_FullCpltCallback);
		HAL_DMA_RegisterCallback(&hdma_tim4_ch1, HAL_DMA_XFER_HALFCPLT_CB_ID, My_HalfCpltCallback);
		HAL_DMA_RegisterCallback(&hdma_tim4_ch1, HAL_DMA_XFER_ERROR_CB_ID, TIM_DMAError);
		HAL_DMA_RegisterCallback(&hdma_tim4_ch2, HAL_DMA_XFER_CPLT_CB_ID, My_FullCpltCallback);
		HAL_DMA_RegisterCallback(&hdma_tim4_ch2, HAL_DMA_XFER_HALFCPLT_CB_ID, My_HalfCpltCallback);
		HAL_DMA_RegisterCallback(&hdma_tim4_ch2, HAL_DMA_XFER_ERROR_CB_ID, TIM_DMAError);

		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);												//// нужно?
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);												//// нужно?



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
			Start();

			//Logger.Print("+PXL=128,16,6144,");
			//Logger.Print(frame_buffer_ptr, frame_buffer_len, LOG_OUT_TYPE_BYTES);
			//Logger.Print("\n");

			//buffer.DrawEnding();

			//DEBUG_LOG_TOPIC("DMADraw", "time: %d\n", (HAL_GetTick() - lasttime));
			lasttime = HAL_GetTick();
		}
		
		
		current_time = HAL_GetTick();
		return;
	}
}
