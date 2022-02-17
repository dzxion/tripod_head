#ifndef __SUPERX_II_H__
#define __SUPERX_II_H__
#include <stdint.h>

#define SUPERX_II_API_VERSION 3.9

/*
 * version note:
 * v3.0 uses superx ii v3.x protocol, but firmware upgarde uses v1.x protocol
 * v3.0 uses crc16 checksum
*/

#define get_module_no(module_type, module_index) (module_type << 0x016 | module_index)
//#define VERSION(ver, sub,area, biuld) (((uint32_t)ver << 24)|((uint32_t)sub << 16)|((uint32_t)area << 8)|(biuld & 0xFF))
#define MODULE_STATUS_OFFLINE 0
#define MODULE_STATUS_ONLINE  1
#define MODULE_MAXNUM  20		//最大模块数目

typedef enum
{
	MODULE_TYPE_FC = 0x0001,
	MODULE_TYPE_AHRS = 0x0002,
	MODULE_TYPE_GPS = 0x0003,
	MODULE_TYPE_IO = 0x0004,
	MODULE_TYPE_XLINK = 0x0005,
	MODULE_TYPE_OA = 0x0006,
	MODULE_TYPE_XFDT = 0x0007,	//BMS
	MODULE_TYPE_TEST = 0x0008,
	MODULE_TYPE_REV3 = 0x0009,
	MODULE_TYPE_SPRAY = 0x000A,
	MODULE_TYPE_SONAR = 0x000B,
	MODULE_TYPE_BATTERY = 0x000C,	//BMU
	MODULE_TYPE_TOF = 0x000D,
	MODULE_TYPE_CAMERA = 0x000E,
	MODULE_TYPE_P20LED = 0x0010,
	MODULE_TYPE_EXPAND = 0x0011,	//NFC-->EXPAND
	MODULE_TYPE_HUB = 0x0012,
	MODULE_TYPE_ESC1 = 0x0013,
	MODULE_TYPE_ESC2 = 0x0014,
	MODULE_TYPE_XSENSE = 0x0015,
	MODULE_TYPE_FLOW = 0x0016,
	MODULE_TYPE_OMNI_RADAR = 0x0017,
	MODULE_TYPE_GIMBAL = 0x0018,
	MODULE_TYPE_SPREAD = 0x0019,
	MODULE_TYPE_FUSEBOX = 0x001A,
	MODULE_TYPE_STEER = 0x001B,
	MODULE_TYPE_SPEAKER = 0x001C,
	MODULE_TYPE_MOWER = 0x001D,
	MODULE_TYPE_KEY = 0x001F,
	MODULE_TYPE_DEBUGGER = 0x000F, 
	MODULE_TYPE_EXT = 0x0009,
	MODULE_TYPE_MEDICINE_BOX = 0x0A10,	//喷洒之模块--智能药箱
	MODULE_TYPE_SPRAY_ESC = 0x0A20,	//喷洒之模块--电机电调
	MODULE_TYPE_SPRAY_MATERIAL_BPX = 0x0A30,	//喷洒之模块--智能料箱
	MODULE_TYPE_SPRAY_SUB_MODULE1 = 0x0A40,
	MODULE_TYPE_SPRAY_SUB_MODULE2 = 0x0A50,
	MODULE_TYPE_SPRAY_SUB_MODULE3 = 0x0A60,
	MODULE_TYPE_SPRAY_SUB_MODULE4 = 0x0A70,
	MODULE_TYPE_SPRAY_SUB_MODULE5 = 0x0A80,
	MODULE_TYPE_SPRAY_SUB_MODULE6 = 0x0A90,
	MODULE_TYPE_SPRAY_SUB_MODULE7 = 0x0AA0,
	MODULE_TYPE_SPRAY_SUB_MODULE8 = 0x0AB0,
	MODULE_TYPE_SPRAY_SUB_MODULE9 = 0x0AC0,
	MODULE_TYPE_SPRAY_SUB_MODULE10 = 0x0AD0,
	MODULE_TYPE_SPRAY_SUB_MODULE11 = 0x0AE0,
	MODULE_TYPE_SPRAY_SUB_MODULE12 = 0x0AF0,
	NODE_TYPE_ROUTER = 0x0041,  //65
	NODE_TYPE_NAV = 0x0042,	  
	NODE_TYPE_TPS = 0x0043,
	NODE_TYPE_DLS = 0x0044,
	NODE_TYPE_HDLS_XLINK = 0x0045,
	NODE_TYPE_ACS_XLINK = 0x0046,
	NODE_TYPE_ACS_GPS = 0x0047,
	NODE_TYPE_ACS_RC = 0x0048,
	NODE_TYPE_ACB_XLINK = 0x0049,
	NODE_TYPE_ACB_RC = 0x004a,
	NODE_TYPE_APP = 0x004b,
	NODE_TYPE_CLOUD = 0x004c,
	NODE_TYPE_AGENCY_RC = 0x004d,
	NODE_TYPE_AGENCY_SPRAY = 0x004e,
	NODE_TYPE_BATTERY_APP = 0x004f,
} API_ModuleType_t;

typedef struct {
	uint8_t cmd;
	uint32_t to_or_from;	// to/from
	uint8_t * Data;
	uint16_t Length;
} MsgData_t;

typedef struct {
	uint32_t From_Port;		//消息发送到大脑端应用时必须按实际填写，发送给小脑及小脑上设备可填0也可按实际填写
	uint32_t To_Port;		//消息发送到大脑端应用时必须按实际填写，发送给小脑及小脑上设备可填0也可按实际填写
	uint8_t Request_Id;		//消息请求ID，自增	From_Port、To_Port为0时此字段无效
	uint8_t Respond_Id;		//应答ID，与接收到的消息请求ID一致返回	From_Port、To_Port为0时此字段无效
	uint32_t From;       //发送消息的时候，From为模块自己 0x18000001
	uint32_t To;         //接收消息的时候，To为模块自己
	MsgData_t msg_data;
} API_Message_t;

typedef struct {
	uint32_t ModuleID[3];
	uint16_t ModuleType;		
	uint16_t ModuleIndex;		    //由API设置
	uint32_t ModuleNo;			    //模块自动注册成功时设置
	uint8_t Status;                 //MODULE_STATUS_ONLINE, MODULE_STATUS_OFFLINE
	uint32_t HardwareVersion;       //硬件版本信息
	uint32_t SoftwareVersion;       //软件版本信息
	uint16_t DataSize;		        //数据区大小
	uint16_t ModuleTimeout;	        //设置模块超时时间
	uint8_t Baudrate;               //0,115200;1,19200;2,38400;3,57600;4,115200;5,256000;6,921600
	uint16_t RequireIndex;           //0为自动分配（默认），1~7为指定
	uint16_t UseId;
	uint8_t Database;				//是否与CAN数据仓库共享，对电调舵机有效，0x88为共享，其他值不共享
	uint8_t NodeDegree;				//节点等级
	uint32_t Reserve;				//保留
} API_Config_t;

typedef struct {
	uint16_t Type;
	uint16_t Index;
	uint8_t Port;
	uint8_t Status;
	uint32_t HardwareVersion;
	uint32_t SoftwareVersion;
	uint32_t id[3];
	uint8_t ProtocolVersion;
	uint16_t use_id;
	uint8_t node_degree;
	uint8_t reserve[3];
} __attribute__((packed)) API_ModuleInfo_t;		//模块信息

typedef struct {
	uint8_t module_count;	//当前页返回的模块数目
	uint8_t page_index;		//当前页码，从0开始
	uint8_t page_count;		//总页数
	API_ModuleInfo_t moduleinfo[MODULE_MAXNUM];
} __attribute__((packed)) API_ModuleLite_t;	//模块列表

typedef struct {
	uint32_t rule_type;
	uint8_t rule_ver;
	uint8_t* data_buff;
	uint16_t data_len;
} UniverLog_t;

typedef struct {
	uint8_t sub_cmd;
	uint8_t status;
	uint8_t reserve[6];
} UpgradeStatus_t;

void api_init(API_Config_t * config);

void api_loop(uint32_t time);

void api_get_fc_protocol_version(uint8_t *fc_protocol_version); // 1,2,3
void api_get_data(int32_t offset, int32_t len);
void api_set_data(int32_t offset, int32_t len, void * data, uint8_t no_ack);
void api_send_message(API_Message_t *msg);
void api_logger_sendUniverLog(UniverLog_t* univerlog);		//send cloud log 
void api_get_modulelist(uint8_t page_index);	//list_version, 0 - old_list, 1 - new_list 返回第一页模块列表
void api_auto_load_data(int32_t offset, int32_t len, uint16_t enabled, uint16_t interval);
void api_upgrade_request(void);
void api_module_upgrade_ack(uint8_t ack);	//升级传输应答 ack 1：成功 0：失败 2：请求重传
void api_module_upgrade_finish(void);
void api_port_received(int32_t len, uint8_t * data);
void api_sub_module_register(API_Config_t* config);		//注册子模块	//timeout:0x7fff  可不发心跳，通过注销接口通知飞控子模块离线
void api_sub_module_unregister(API_Config_t* config);	//注销子模块

void api_port_send(int32_t len, uint8_t * data); //weak
void api_on_data_set(uint8_t result); //weak, 0 - successful, 1 - failed
void api_on_data_returned(int32_t offset, int32_t len, uint8_t * data); //weak
void api_on_message_received(API_Message_t message); //weak
void api_on_upgrade(void);
void api_on_upgrade_requst(uint32_t file_size,uint16_t TotalPackage);	//weak
void api_on_upgrade_pack(uint8_t pack_size,uint8_t* data);	//weak
void api_on_upgrade_finsh(void);	//weak
void api_on_modulelist_received(API_ModuleLite_t *modublelist);	
void api_on_upgrade_status_returned(UpgradeStatus_t *upgtade_status);	// get upgrade status 0-idle 1-during upgrade 2-upgrade success 3-upgrade fail

//Utils
#define CRC16_INIT_VALUE 0xFFFF
typedef struct{
    uint16_t current; //termporary value during crc16 calculation,original value is 0xffff
    uint16_t crc16; //the crc16 value
} crc16_t;

void crc16(uint8_t input, crc16_t * output);
uint16_t crc_ccitt(uint8_t * input, int len);

#endif  /* endif __SUPERX_II_H__ */

