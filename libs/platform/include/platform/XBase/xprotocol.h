#ifndef __MR133_PROTOCOL_H__
#define __MR133_PROTOCOL_H__

#include "stdint.h"

#ifndef FJ212_PROTOCOL
#define FJ212_PROTOCOL
#endif

/* FRAME  */
#define     FRAME_HEAD    0xFEFEFEFE
#define     FRAME_TAIL    0xFDFDFDFD

/* BUMPER NUM */
#define		BUMPER_NUM			2
#define		BUMPER_INDEX_LEFT	0
#define		BUMPER_INDEX_RIGHT	1

/* IR Section */
#define		IR_SENSOR_NUM		11
#define 	IR_INDEX_CLIFF_L	0
#define		IR_INDEX_CLIFF_LL	1
#define		IR_INDEX_CLIFF_R	2
#define		IR_INDEX_CLIFF_RR	3
#define		IR_INDEX_HOME_L		4
#define		IR_INDEX_HOME_R		5
#define		IR_INDEX_WALL		6
#define		IR_INDEX_LIGHTOUCH  7		// only 1 front, not use anymore
#define		IR_INDEX_LT_LEFT	8
#define		IR_INDEX_LT_CENTER	9
#define		IR_INDEX_LT_RIGHT	10

#define		BITMASK_DUST_BOX_ERR		0x0001
#define		BITMASK_WATER_BOX_ERR		0x0002
#define		BITMASK_BUMER_L_HIT			0x0004 
#define		BITMASK_BUMER_R_HIT			0x0008
#define		BITMASK_WHEEL_L_ERR			0x0010 
#define		BITMASK_WHEEL_R_ERR			0x0020 
#define		BITMASK_WHEEL_L_SUSPEND		0x0040
#define		BITMASK_WHEEL_R_SUSPEND		0x0080
#define		BITMASK_POWER_CHARGING		0x0100


#define     BITMASK_CMD_WHEEL_ENABLE    0x0004

#pragma pack(1)

//#ifdef FJ212_PROTOCOL  // 192 also need this
typedef struct
{
	uint16_t ticks;
	uint8_t base_addr_L;
	uint8_t robot_addr_L;
	uint16_t base_addr_H;
	uint16_t robot_addr_H;
	union{
		struct
		{
			struct
			{
				uint16_t SetCleanWaterPumpPower : 7;
				uint16_t SetCleanLiquidPumpPower : 7;
				uint16_t res : 2;
			}Motor;
			struct
			{
				uint16_t OpenHomeLed : 1;
				uint16_t EnablePtcModule : 1;
				uint16_t OpenDustCollection : 1;
				uint16_t EnableHornSilence : 1;
				uint16_t OpenWaterInjectionValve : 1;

				uint16_t SetDrainagePumpPower : 1;
				uint16_t SetFanPower : 1;
				uint16_t SetSewagePumpPower : 1;

				uint16_t SetRobotIsBaseStaton : 1;

				uint16_t RobotFullCharge : 1;
				uint16_t ClosePanelLCD : 1;
				uint16_t PanelChildLock : 1;

				uint16_t res : 4;
			}IO;
			struct 
			{
				uint16_t WorkingCondition : 4;//RobotRunningStatus_t
				uint16_t WorkingMode : 4;//RobotRunModeStatus_t
				uint16_t WorkType : 4;//RobotRunModeStatus_t
				uint16_t res : 4;
			}RobotStatus;
		}robot2station;
		struct
		{
			struct 
			{
				uint16_t CleanWaterBox : 1;
				uint16_t SewageBoxOrFull : 1;
				uint16_t CleaningSolutionBox : 1;
				uint16_t DustBox : 1;
				uint16_t CleanTray : 1;
				uint16_t PtcModule : 1;
				uint16_t HomeLed : 1;
				uint16_t CleanWaterValve : 1;
				uint16_t WaterInjectionValve : 1;
				uint16_t WaterInjectionModule : 1;
				uint16_t CleanWaterBoxEmpty : 1;
				uint16_t CleaningSolutionBoxEmpty : 1;
				uint16_t CleanWaterBoxFull : 1;
				uint16_t CleanTrayWaterFull : 1;
				uint16_t HornSilence : 1;
				uint16_t PowerOn : 1;
			}IO;
			struct 
			{
				uint16_t BaseStationStatus : 4;//BaseStationStatus_t
				uint16_t PanelCtrlMsg : 4;//PanelCtrlMsg_t
				uint16_t PanelLed : 1;
				uint16_t PanelChildLock : 1;
				uint16_t res : 6;
			}WorkStatus;
			struct 
			{
				uint16_t Version1 : 4;
				uint16_t Version2 : 4;
				uint16_t Version3 : 4;
				uint16_t res : 4;
			}Version;
		}station2robot;
		uint8_t raw[6];
	}data;
	uint16_t checksum;
}RFData;
//#endif

typedef struct
{
	uint32_t frame_header;
	uint16_t seq;

	// PWM duct-cycle, -100~100 
	int8_t left_wheel_duty_cycle;
	int8_t right_wheel_duty_cycle;
	int8_t fan_duty_cycle;
#ifdef FJ212_PROTOCOL
	int8_t mop_motor_duty_cycle;
#else
	int8_t left_brush_duty_cycle;
#endif
	int8_t right_brush_duty_cycle;
	int8_t main_brush_duty_cycle;
#ifdef FJ212_PROTOCOL
	int8_t mid_brush_lift_motor_duty_cycle;
#else
	int8_t pump_duty_cycle;
#endif


	// ADC value
	uint8_t left_wheel_current;
	uint8_t right_wheel_current;
	uint8_t fan_current;
#ifdef FJ212_PROTOCOL
	uint8_t mop_motor_current;
#else
	uint8_t left_brush_current;
#endif
	uint8_t right_brush_current;
	uint8_t main_brush_current;
	uint8_t cliff_l_adc;
	uint8_t cliff_ll_adc;
	uint8_t cliff_r_adc;
	uint8_t cliff_rr_adc;
	uint8_t bat_adc;

	// error status (bit filed)
	union
	{
		struct
		{
			uint16_t dust_box_missing : 1;
			uint16_t water_box_missing : 1;
			uint16_t bumper_left_hit : 1;
			uint16_t bumper_right_hit : 1;
			uint16_t left_wheel_err : 1;
			uint16_t right_wheel_err : 1;
			uint16_t left_wheel_suspend : 1;
			uint16_t right_wheel_suspend : 1;

			uint16_t power_is_chagering : 1;
			uint16_t bumper_processing : 1;
			uint16_t mcu_jitter : 1;
#ifdef FJ212_PROTOCOL
			uint16_t mid_brush_lift_motor_limit2 : 1;
			uint16_t is_hard_floor : 1;
			uint16_t is_bat_on : 1;
			uint16_t mid_brush_lift_motor_limit : 1;
			uint16_t power_charge_complete : 1;
#endif
		}bit_field;
		uint16_t all;
	}err_status;

	// encoder
	int32_t left_wheel_pulses;
	int32_t right_wheel_pulses;
	uint16_t fan_pulses;
	int8_t left_wheel_temp;
	int8_t right_wheel_temp;

#ifdef FJ212_PROTOCOL
	uint8_t bat_ntc_adc;
#else
	uint8_t left_IR_status : 4;
	uint8_t right_IR_status : 4;
#endif
#ifdef FJ212_PROTOCOL
	union
	{
		struct
		{
			uint8_t major : 4;
			uint8_t minor : 4;
		}bit_field;
		uint8_t all;
	}fw_version;
	uint8_t reserved;
#else
	uint16_t proximity_dist_mm;
#endif

	// TODO: IMU data
	int16_t q0;
	int16_t q1;
	int16_t q2;
	int16_t q3;

	int16_t extra_imu_angle_x100;
	int16_t extra_imu_angle_rate_x100;
#ifdef FJ212_PROTOCOL
	uint8_t mid_brush_lift_motor_current;
	union
	{
		struct
		{
			uint8_t right_side_brush_hall : 1;
			uint8_t mop_left_hall : 1;
			uint8_t mop_right_hall : 1;
			uint8_t IMU_need_stop : 1;
			uint8_t mop_left_IR : 1;
			uint8_t mop_right_IR : 1;
			uint8_t reserved2 : 1;
			uint8_t reserved1 : 1;

		}bit_field;
		uint8_t all;
	}extra_status_212;
	uint8_t cliff_lf_adc;
	uint8_t cliff_rf_adc;

#else
	union
	{
		struct
		{
			uint32_t left : 10;
			uint32_t middle : 10;
			uint32_t right : 10;
			uint32_t reserved : 2;
		}bit_field;
		uint32_t all;
	}lighttouch;
#endif

	int16_t odometry_x_mm;
	int16_t odometry_y_mm;
	int16_t odometry_yaw_mrad;

	int16_t v_fdk_mm_s;
	int16_t w_fdk_mrad_s;

	int16_t v_set_mm_s;
	int16_t w_set_mrad_s;

#ifdef FJ212_PROTOCOL
	RFData rf_data;
	int16_t imu_raw_accel[3];
	int16_t imu_raw_gyro[3];
#endif
	uint32_t frame_tail;
}mcu_mr133_data_t;
typedef struct
{
	uint32_t frame_header;
	uint16_t seq;

	// PWM duty-cycle, -100~100 
	int8_t left_wheel_duty_cycle;
	int8_t right_wheel_duty_cycle;
	int8_t fan_duty_cycle;
#ifdef FJ212_PROTOCOL
	int8_t mop_motor_duty_cycle; // KP
#else
	int8_t brush_l_duty_cycle; // KP
#endif
	int8_t brush_r_duty_cycle; // KI
	int8_t brush_m_duty_cycle; // KD
#ifdef FJ212_PROTOCOL
	int8_t mid_brush_lift_motor_duty_cycle;
#else
	int8_t pump_duty_cycle;
#endif

	// cmd (bit filed)
	union
	{
		struct
		{
			uint8_t spd_loop_enable : 1;
#ifdef FJ212_PROTOCOL
			uint8_t ultrasonic_debug : 1;
#else
			uint8_t cut_enable : 1;
#endif
			uint8_t wheel_enable : 1;
			uint8_t clear_err : 1;
			uint8_t reset_odometry : 1;
			uint8_t bumper_manual_back : 1;
			uint8_t spd_loop_pid_tunning : 1;
#ifdef FJ212_PROTOCOL
			uint8_t rf_response_enable : 1;
#else
			uint8_t reserved4 : 1;
#endif
		}bit_field;
		uint8_t all;
	}cmd;

	// spd
	int16_t v_tar_mm_s;
	int16_t w_tar_mrad_s;

#ifdef FJ212_PROTOCOL
	RFData rf_data;
#endif
	uint32_t frame_tail;
}mr133_mcu_data_t;
#pragma pack()



#endif