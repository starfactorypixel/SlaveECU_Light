#pragma once
#include <CANLibrary.h>
#include "CanObj/CanBlockInfo.hpp"
#include "CanObj/CanBlockCfg.hpp"
#include "CanObj/CanLightCtrl.hpp"
#include "CANFunc.h"
#include <DrakePinD.hpp>

extern FDCAN_HandleTypeDef hfdcan1;
extern bool HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

namespace CANLib
{
	static constexpr uint8_t CFG_CANObjectsCount = 10;
	static constexpr uint16_t CAN_BASE_ID = 0x01C0;
	
	DrakePinD can_rs({GPIOA, GPIO_PIN_15}, DrakePin::OutputOpenDrain, DrakePin::High);
	
	CANManager<CFG_CANObjectsCount> can_manager(&HAL_CAN_Send, &HAL_GetTick, &OnInterruptCtrl);
	
	CanBlockInfo obj_block_info(CAN_BASE_ID+0, OnStaticInfoReq, OnDynamicInfoReq);
	CanBlockCfg obj_block_cfg(CAN_BASE_ID+1, OnCfgSaveReset, block_cfg_table, block_cfg_table_count);
	
	CanLightCtrl obj_side_beam(CAN_BASE_ID+4, Outputs::LIGHT_SIDEBEAM);
	CanLightCtrl obj_low_brake_beam(CAN_BASE_ID+5, Outputs::LIGHT_LOW_BRAKE_BEAM);
	CanLightCtrl obj_high_reverse_beam(CAN_BASE_ID+6, Outputs::LIGHT_HIGH_REVERSE_BEAM);
	CanLightCtrl obj_left_indicator(CAN_BASE_ID+7, Outputs::LIGHT_LEFT_INDICATOR);
	CanLightCtrl obj_right_indicator(CAN_BASE_ID+8, Outputs::LIGHT_RIGHT_INDICATOR);
	CanLightCtrl obj_hazard_beam(CAN_BASE_ID+9, Outputs::LIGHT_HAZARD_BEAM);
	CanLightCtrl obj_custom_beam(CAN_BASE_ID+10, Outputs::LIGHT_CUSTOM_BEAM);
	//CANObject<uint8_t,  1> obj_led_control(CFG_CANFirstId + 11);			// Управление WS2812
	
	
	void CAN_Enable()
	{
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0);
		HAL_FDCAN_Start(&hfdcan1);
		
		can_rs.Off();
		
		return;
	}
	
	void CAN_Disable()
	{
		HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING);
		HAL_FDCAN_Stop(&hfdcan1);
		
		can_rs.On();
		
		return;
	}
	
	inline void Setup()
	{
		can_rs.Init();

		can_manager.AddObject(obj_block_info);
		can_manager.AddObject(obj_block_cfg);
		can_manager.AddObject(obj_side_beam);
		can_manager.AddObject(obj_low_brake_beam);
		can_manager.AddObject(obj_high_reverse_beam);
		can_manager.AddObject(obj_left_indicator);
		can_manager.AddObject(obj_right_indicator);
		can_manager.AddObject(obj_hazard_beam);
		can_manager.AddObject(obj_custom_beam);
		//can_manager.AddObject(obj_led_control);
		
		CAN_Enable();
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Processing();
		
		current_time = HAL_GetTick();
		return;
	}
}

IBlockInfoSender &BlockInfoSender = CANLib::obj_block_info;
