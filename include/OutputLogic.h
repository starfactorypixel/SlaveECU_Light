#pragma once
#include  <PowerOut.h>

extern ADC_HandleTypeDef hadc1;

namespace Outputs
{
	/* Настройки */
	static constexpr uint8_t CFG_PortCount = 7;			// Кол-во портов управления.
	static constexpr uint32_t CFG_RefVoltage = 3300000;	// Опорное напряжение, микровольты.
	static constexpr uint8_t CFG_INA180_Gain = 50;		// Усиление микросхемы INA180.
	static constexpr uint8_t CFG_ShuntResistance = 5;	// Сопротивление шунта, миллиомы.

	static constexpr uint16_t CFG_TurnTimeOn = 550;
	static constexpr uint16_t CFG_TurnTimeOf = 400;
	/* */
	
	PowerOut<CFG_PortCount> outObj(&hadc1, CFG_RefVoltage, CFG_INA180_Gain, CFG_ShuntResistance);


	struct obj_t
	{
		CANObjectInterface *obj;
	};
	
	
	void OnShortCircuit(uint8_t num, uint16_t current)
	{

	}


	
	
	inline void Setup()
	{
		outObj.AddPort( {GPIOD, GPIO_PIN_0}, {GPIOA, GPIO_PIN_1, ADC_CHANNEL_17}, 5000 );	// Выход 1, Габариты
		outObj.AddPort( {GPIOD, GPIO_PIN_1}, {GPIOA, GPIO_PIN_2, ADC_CHANNEL_14}, 5000 );	// Выход 2, Ближний свет или Стоп сигнал
		outObj.AddPort( {GPIOD, GPIO_PIN_3}, {GPIOA, GPIO_PIN_3, ADC_CHANNEL_15}, 5000 );	// Выход 3, Дальний свет или Задний ход
		outObj.AddPort( {GPIOD, GPIO_PIN_4}, {GPIOA, GPIO_PIN_4, ADC_CHANNEL_18}, 5000 );	// Выход 4, Левый поворотник
		outObj.AddPort( {GPIOD, GPIO_PIN_5}, {GPIOA, GPIO_PIN_5, ADC_CHANNEL_19}, 5000 );	// Выход 5, Правый поворотник
		outObj.AddPort( {GPIOD, GPIO_PIN_6}, {GPIOA, GPIO_PIN_6, ADC_CHANNEL_3}, 5000 );	// Выход 6, Доп. свет
		outObj.AddPort( {GPIOD, GPIO_PIN_7}, {GPIOA, GPIO_PIN_7, ADC_CHANNEL_7}, 20000 );	// Выход HiPower-1
		
		outObj.Init();

		//outObj.On(4);
		//outObj.On(6);
		//outObj.Off(1);
		outObj.RegShortCircuitEvent(OnShortCircuit);
		//outObj.Current(1);

		//outObj.SetOn(6, 250, 500);
		//outObj.SetOn(5, 1000, 100);

		CANLib::obj_side_beam.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				bool result = outObj.SetOn(1);
				response = ((result) ? can_frame.data[0] : 0x00);
			}
			else
			{
				outObj.SetOff(1);
				response = 0x00;
			}
			CANLib::obj_side_beam.SetValue(0, response, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
			
			return CAN_RESULT_IGNORE;
		});
		
		CANLib::obj_low_brake_beam.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				bool result = outObj.SetOn(2);
				response = ((result) ? can_frame.data[0] : 0x00);
			}
			else
			{
				outObj.SetOff(2);
				response = 0x00;
			}
			CANLib::obj_low_brake_beam.SetValue(0, response, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
			
			return CAN_RESULT_IGNORE;
		});
		
		CANLib::obj_high_reverse_beam.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				bool result = outObj.SetOn(3);
				response = ((result) ? can_frame.data[0] : 0x00);
			}
			else
			{
				outObj.SetOff(3);
				response = 0x00;
			}
			CANLib::obj_high_reverse_beam.SetValue(0, response, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
			
			return CAN_RESULT_IGNORE;
		});
		
		CANLib::obj_left_indicator.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				bool result = outObj.SetOn(4, CFG_TurnTimeOn, CFG_TurnTimeOf);
				response = ((result) ? can_frame.data[0] : 0x00);
			}
			else
			{
				outObj.SetOff(4);
				response = 0x00;
			}
			CANLib::obj_left_indicator.SetValue(0, response, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
			
			return CAN_RESULT_IGNORE;
		});
		
		CANLib::obj_right_indicator.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				bool result = outObj.SetOn(5, CFG_TurnTimeOn, CFG_TurnTimeOf);
				response = ((result) ? can_frame.data[0] : 0x00);
			}
			else
			{
				outObj.SetOff(5);
				response = 0x00;
			}
			CANLib::obj_right_indicator.SetValue(0, response, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
			
			return CAN_RESULT_IGNORE;
		});
		
		CANLib::obj_hazard_beam.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				bool result1 = outObj.SetOn(4, CFG_TurnTimeOn, CFG_TurnTimeOf);
				bool result2 = outObj.SetOn(5, CFG_TurnTimeOn, CFG_TurnTimeOf);
				response = ((result1 && result2) ? can_frame.data[0] : 0x00);
			}
			else
			{
				outObj.SetOff(4);
				outObj.SetOff(5);
				response = 0x00;
			}
			CANLib::obj_hazard_beam.SetValue(0, response, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
			
			return CAN_RESULT_IGNORE;
		});
		
		CANLib::obj_custom_beam.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				bool result = outObj.SetOn(6);
				response = ((result) ? can_frame.data[0] : 0x00);
			}
			else
			{
				outObj.SetOff(6);
				response = 0x00;
			}
			CANLib::obj_custom_beam.SetValue(0, response, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
			
			return CAN_RESULT_IGNORE;
		});
		
		return;
	}


	uint8_t test_iter = 1;
	
	inline void Loop(uint32_t &current_time)
	{
		outObj.Processing(current_time);
		
		static uint32_t last_time = 0;
		if(current_time - last_time > 250)
		{
			last_time = current_time;

/*
			outObj.SetOff(test_iter++);
			if(test_iter == 9) test_iter = 1;
			outObj.SetOn(test_iter);
*/			
			Logger.PrintTopic("POUT");
			for(uint8_t i = 1; i < CFG_PortCount+1; ++i)
			{
				Logger.Printf("%05d;", outObj.GetCurrent(i));
			}
			Logger.PrintNewLine();
		}
		
		current_time = HAL_GetTick();
		
		return;
	}
}
