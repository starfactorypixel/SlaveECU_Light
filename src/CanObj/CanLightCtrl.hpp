#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>

class CanLightCtrl : public CANObjectBase
{
	struct __attribute__((packed)) set_t { uint8_t fId; uint8_t val; };
	struct __attribute__((packed)) request_t { uint8_t fId; };
	struct __attribute__((packed)) event_ok_t { uint8_t fId = CAN_FUNC_EVENT_OK; uint8_t val; };
	
	public:
		CanLightCtrl(can_object_id_t id, Outputs::light_t light) : CANObjectBase(id), _light(light)
		{

		};

		/*
		void SoftEvent(CANLib::backevent_type_t type, uint16_t val)
		{
			DEBUG_LOG_SIMPLE("type: %d, val: %d, port: %d\n", type, val, _light);
		}
		*/

		
	protected:
		virtual void OnProcessFrame(can_frame_t &can_frame) noexcept override
		{
			uint8_t fId = can_frame.raw_data[0];
			switch(fId)
			{
				case CAN_FUNC_SET_IN:
				{
					set_t *obj = (set_t *)can_frame.raw_data;
					Outputs::LightControl(_light, obj->val);
					_GetAndSendPortState();
					
					break;
				}

				case CAN_FUNC_REQUEST_IN:
				{
					//request_t *obj = (request_t *)can_frame.raw_data;
					_GetAndSendPortState();
					
					break;
				}
			}

			return;
		}
		
	private:
		void _GetAndSendPortState()
		{
			event_ok_t answer = {};
			answer.val = Outputs::LightState(_light);
			this->SendFrame((uint8_t *)&answer, sizeof(answer));
		}
		
		Outputs::light_t _light;
};
