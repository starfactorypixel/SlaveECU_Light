#pragma once
#include <PowerOutV2.h>
#include <DrakePinD.hpp>
#include <DrakePinA.hpp>
#include <CUtils.h>

extern ADC_HandleTypeDef hadc2;

namespace Outputs
{
	static constexpr uint16_t CFG_TurnTimeOn = 550;
	static constexpr uint16_t CFG_TurnTimeOf = 400;
	
	void OnControl(uint8_t port, uint8_t id, uint8_t state);
	uint16_t OnCurrentGet(uint8_t port, uint8_t id);
	void OnCurrentLimit(uint8_t port, uint16_t current);
	
	DrakePinD pinsd[] = 
	{
		{{GPIOC, GPIO_PIN_4}, DrakePin::Output, DrakePin::High},
		{{GPIOC, GPIO_PIN_5}, DrakePin::Output, DrakePin::High},
		{{GPIOB, GPIO_PIN_0}, DrakePin::Output, DrakePin::High},
		{{GPIOB, GPIO_PIN_1}, DrakePin::Output, DrakePin::High},
		{{GPIOB, GPIO_PIN_2}, DrakePin::Output, DrakePin::High},
		{{GPIOE, GPIO_PIN_7}, DrakePin::Output, DrakePin::High}
	};

	PowerOutV2<8> ports(HAL_GetTick, OnControl, OnCurrentGet);
	INACurrentCalc ina_calc(12, 3300, 2, 100);

	DrakePinA ntc_in({&hadc2, GPIOC, GPIO_PIN_0, ADC_CHANNEL_10}, ADC_SAMPLETIME_8CYCLES_5);

	enum port_t : uint8_t
	{
		PORT_NONE, 
		PORT_1, PORT_2, PORT_3, PORT_4, PORT_5, PORT_6
	};
	
	
	void OnControl(uint8_t port, uint8_t id, uint8_t state)
	{
		DrakePin::LevelD_t new_state = (state == PowerOutBase::STATE_ON) ? DrakePin::Low : DrakePin::High;
		pinsd[id].Write(new_state);
	}
	
	uint16_t OnCurrentGet(uint8_t port, uint8_t id)
	{
		uint16_t adc = Analog::GetRegularValue(id);

		return ina_calc.Get_mA(adc);
	}
	
	void OnCurrentLimit(uint8_t port, uint16_t current)
	{
		//CANLib::SoftEventOutputs(CANLib::EVENT_CURR_LIMIT, port, current);
		//BlockInfoSender.SendErrorMsg(port, 10, current);
	}


	
	
	inline void Setup()
	{
		for(auto &pins : pinsd)
		{
			pins.Init();
		}

		ports.SetPort(PORT_1, 0, Analog::PORT_REG1, 1000);	// Выход 1, Габариты
		ports.SetPort(PORT_2, 1, Analog::PORT_REG2, 1000);	// Выход 2, Ближний свет или Стоп сигнал
		ports.SetPort(PORT_3, 2, Analog::PORT_REG3, 1000);	// Выход 3, Дальний свет или Задний ход
		ports.SetPort(PORT_4, 3, Analog::PORT_REG4, 1000);	// Выход 4, Левый поворотник
		ports.SetPort(PORT_5, 4, Analog::PORT_REG5, 1000);	// Выход 5, Правый поворотник
		ports.SetPort(PORT_6, 5, Analog::PORT_REG6, 1000);	// Выход 6, Доп. свет
		ports.Init();

		//outObj.On(4);
		//outObj.On(6);
		//outObj.Off(1);
		ports.SetCallbackCurrentLimit(OnCurrentLimit);
		//outObj.Current(1);

		//ports.CtrlOn(6, 250, 500);
		//ports.CtrlOn(5, 1000, 100);

		ntc_in.Init();

		CANLib::obj_side_beam.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			uint8_t response;
			if(can_frame.data[0] > 0)
			{
				ports.CtrlOn(PORT_1);
				response = 0xFF;
			}
			else
			{
				ports.CtrlOff(PORT_1);
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
				ports.CtrlOn(PORT_2);
				response = 0xFF;
			}
			else
			{
				ports.CtrlOff(PORT_2);
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
				ports.CtrlOn(PORT_3);
				response = 0xFF;
			}
			else
			{
				ports.CtrlOff(PORT_3);
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
				ports.CtrlOn(PORT_4, CFG_TurnTimeOn, CFG_TurnTimeOf);
				response = 0xFF;
			}
			else
			{
				ports.CtrlOff(PORT_4);
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
				ports.CtrlOn(PORT_5, CFG_TurnTimeOn, CFG_TurnTimeOf);
				response = 0xFF;
			}
			else
			{
				ports.CtrlOff(PORT_5);
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
				ports.CtrlOn(PORT_4, CFG_TurnTimeOn, CFG_TurnTimeOf);
				ports.CtrlOn(PORT_5, CFG_TurnTimeOn, CFG_TurnTimeOf);
				response = 0xFF;
			}
			else
			{
				ports.CtrlOff(PORT_4);
				ports.CtrlOff(PORT_5);
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
				ports.CtrlOn(PORT_6);
				response = 0xFF;
			}
			else
			{
				ports.CtrlOff(PORT_6);
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
		ports.Processing(current_time);
		
		static uint32_t last_time = 0;
		if(current_time - last_time > 250)
		{
			last_time = current_time;

/*
			ports.CtrlOff(test_iter++);
			if(test_iter == 9) test_iter = 1;
			ports.CtrlOn(test_iter);
*/

/*
			Logger.PrintTopic("POUT");
			for(uint8_t i = 1; i < CFG_PortCount+1; ++i)
			{
				Logger.Printf("%05d;", outObj.GetCurrent(i));
			}
			Logger.PrintNewLine();
*/
		}

	
		current_time = HAL_GetTick();
		
		return;
	}
}
