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


	enum light_t : uint8_t
	{
		LIGHT_NONE = 0,
		LIGHT_SIDEBEAM,
		LIGHT_LOW_BRAKE_BEAM,
		LIGHT_HIGH_REVERSE_BEAM,
		LIGHT_LEFT_INDICATOR,
		LIGHT_RIGHT_INDICATOR,
		LIGHT_HAZARD_BEAM,
		LIGHT_CUSTOM_BEAM
	};

	void LightControl(light_t type, uint8_t value)
	{
		PowerOutBase::state_t state = (value > 0) ? PowerOutBase::STATE_ON : PowerOutBase::STATE_OFF;
		
		switch(type)
		{
			case LIGHT_SIDEBEAM:
			{
				ports.CtrlWrite(PORT_1, state);
				break;
			}
			case LIGHT_LOW_BRAKE_BEAM:
			{
				ports.CtrlWrite(PORT_2, state);
				break;
			}
			case LIGHT_HIGH_REVERSE_BEAM:
			{
				ports.CtrlWrite(PORT_3, state);
				break;
			}
			case LIGHT_LEFT_INDICATOR:
			{
				// Добавить CtrlWrite в режиме blink
				if(state == PowerOutBase::STATE_ON)
					ports.CtrlOn(PORT_4, CFG_TurnTimeOn, CFG_TurnTimeOf);
				else
					ports.CtrlOff(PORT_4);
				break;
			}
			case LIGHT_RIGHT_INDICATOR:
			{
				if(state == PowerOutBase::STATE_ON)
					ports.CtrlOn(PORT_5, CFG_TurnTimeOn, CFG_TurnTimeOf);
				else
					ports.CtrlOff(PORT_5);
				break;
			}
			case LIGHT_HAZARD_BEAM:
			{
				if(state == PowerOutBase::STATE_ON)
				{
					ports.CtrlOn(PORT_4, CFG_TurnTimeOn, CFG_TurnTimeOf);
					ports.CtrlOn(PORT_5, CFG_TurnTimeOn, CFG_TurnTimeOf);
				}
				else
				{
					ports.CtrlOff(PORT_4);
					ports.CtrlOff(PORT_5);
				}
				break;
			}
			case LIGHT_CUSTOM_BEAM:
			{
				ports.CtrlWrite(PORT_6, state);
				break;
			}
			default:
			{
				break;
			}
		}

		return;
	}
	
	uint8_t LightState(light_t type)
	{
		PowerOutBase::state_t state;
		
		switch(type)
		{
			case LIGHT_SIDEBEAM:
			{
				state = ports.GetState(PORT_1);
				break;
			}
			case LIGHT_LOW_BRAKE_BEAM:
			{
				state = ports.GetState(PORT_2);
				break;
			}
			case LIGHT_HIGH_REVERSE_BEAM:
			{
				state = ports.GetState(PORT_3);
				break;
			}
			case LIGHT_LEFT_INDICATOR:
			{
				state = ports.GetState(PORT_4);
				break;
			}
			case LIGHT_RIGHT_INDICATOR:
			{
				state = ports.GetState(PORT_5);
				break;
			}
			case LIGHT_HAZARD_BEAM:
			{
				if(ports.GetState(PORT_4) == PowerOutBase::STATE_ON && ports.GetState(PORT_5) == PowerOutBase::STATE_ON)
					state = PowerOutBase::STATE_ON;
				break;
			}
			case LIGHT_CUSTOM_BEAM:
			{
				state = ports.GetState(PORT_6);
				break;
			}
			default:
			{
				break;
			}
		}

		return (state == PowerOutBase::STATE_ON) ? 0xFF : 0x00;
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
