
#ifndef __NAV_COMPRESSED_MAP_CONFIG_H__
#define __NAV_COMPRESSED_MAP_CONFIG_H__

#define NAV_COM_MAP_MAX_WIDTH_MM  (51200)  // -25.6 m ~ 25.6m
#define NAV_COM_MAP_MAX_HEIGHT_MM (51200) //  -25.6 m ~ 25.6m
#define NAV_COM_MAP_BLOCK_SIZE (32)

typedef enum
{
	NAV_COM_MAP_TYPE_NONE = 0,
	NAV_COM_MAP_TYPE_SLAM = 0x01,
	NAV_COM_MAP_TYPE_OBS = 0x02,
	NAV_COM_MAP_TYPE_STUCK = 0x04,
	NAV_COM_MAP_TYPE_CLEANED = 0x08,
	NAV_COM_MAP_TYPE_OBS_CLEANED = 0x10,
	NAV_COM_MAP_TYPE_DYNAMIC_MAP = 0x20,
	NAV_COM_MAP_TYPE_DSTAR_MAP = 0x40,
	NAV_COM_MAP_TYPE_END = 0x80
}nav_com_map_type_t;

#define NAV_COM_SLAM_MAP_UNIT_MM        (50)
#define NAV_COM_OBS_MAP_UNIT_MM         (50)
#define NAV_COM_STUCK_MAP_UNIT_MM       (50)
#define NAV_COM_CLEANED_MAP_UNIT_MM     (20)
#define NAV_COM_OBS_CLEANED_MAP_UNIT_MM (20)
#define NAV_COM_DYNAMIC_MAP_UNIT_MM     (50)
#define NAV_COM_DSTAR_MAP_UNIT_MM       (50)

#define NAV_COM_SLAM_MAP_bpp            (2)
#define NAV_COM_OBS_MAP_bpp             (8)
#define NAV_COM_STUCK_MAP_bpp           (1)
#define NAV_COM_CLEANED_MAP_bpp         (1)
#define NAV_COM_OBS_CLEANED_MAP_bpp     (1)
#define NAV_COM_DYNAMIC_MAP_bpp         (1)
#define NAV_COM_DSTAR_MAP_bpp           (1)


#define NCM_OBS_MODEL_SIZE_MM  (750)
#define NCM_OBS_MODEL_SIZE     (NCM_OBS_MODEL_SIZE_MM / NAV_COM_OBS_MAP_UNIT_MM)

#define NCM_OBS_POINT_WEIGHT_DEFAULT (80)
#define NCM_OBSMAP2STUCKMAP_THRESHOLD (40)
struct ncm_obs_point
{
	float x;
	float y;
	uint8_t weight;
	ncm_obs_point()
	{
		weight = NCM_OBS_POINT_WEIGHT_DEFAULT;
	}
};
typedef struct ncm_obs_point ncm_obs_point_t;




#define NCM_MAX_CB_NUM 10

// GUI-Related 
#define NAV_COM_MAP_DEBUG_PORT 9999
#define NAV_COM_MAP_COMM_MAX_BLOCK_CNT (32)

#endif
