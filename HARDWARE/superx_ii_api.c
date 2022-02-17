#include "superx_ii_api.h"
#include <string.h>
#include <stdint.h>

/* Compiler Related Definitions */
#ifdef __CC_ARM                         /* ARM Compiler */
#include <stdarg.h>
#define ALIGN(n)                    __attribute__((aligned(n)))
#define WEAK			            __weak
#elif defined ( __GNUC__ )
#include <stdarg.h>
#define ALIGN(n)                    __attribute__((aligned(n)))
#define WEAK			            __weak
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
#include <stdarg.h>
#define PRAGMA(x)                   _Pragma(#x)
#define ALIGN(n)                    PRAGMA(data_alignment=n)
#define WEAK                        __weak
#endif

// frame state
#define FRAME_SYNC     0
#define FRAME_HEADER   1
#define FRAME_DATA     2

enum Command
{
    CMD_NOMODULE = 0x00,
    CMD_REGISTER = 0x01,
    CMD_HEARTBEAT = 0x02,
    CMD_GET_MODULE = 0x03,
    CMD_GET_DATA = 0x04,
    CMD_SET_DATA = 0x05,
    CMD_SEND_MSG = 0x06,
    CMD_AUTO_LOAD_DATA = 0x07,
    CMD_SEND_NET_MSG = 0x08,
	CMD_SUB_MODULE_REGISTER = 0x11,
	CMD_SUB_MODULE_UNREGISTER = 0x12,
    CMD_UPGRADE = 0xf0,
    CMD_UPGRADE_REQUEST = 0xf1,
    CMD_UPGRADE_PK = 0xf2,
    CMD_UPGRADE_FINISH = 0xf3,
	CMD_UPGRADE_STATUS = 0xf4,
};

#define NET_MSG_OFFSET  (10)
#define FRAME_BUFFER_SIZE       (520+8+4+NET_MSG_OFFSET+2+3) //len(2), cmd(1), to(1)
#define FRAME_HEAD_SIZE	(15)
#define FRAME_OLDHEAD_SIZE	(9)
#define LOG_DATA_SIZE	(512)

typedef struct
{
    uint8_t Start;
	uint8_t Checksum;
	uint16_t Length;
	uint8_t From;
	uint8_t To;
	uint8_t CMD;
} __attribute__((packed)) header_t;

typedef struct
{
    uint16_t Checksum;
    uint16_t Length;
    uint8_t From;
    uint8_t To;
    uint8_t CMD;
} __attribute__((packed)) header_v2_t;

typedef struct
{
    //ignore two bytes of start
    uint16_t Checksum;
    uint16_t Length;
    uint32_t  From;
    uint32_t  To;
    uint8_t  CMD;
} __attribute__((packed)) header_v3_t;

typedef struct
{
    union
    {
        uint8_t RawData[FRAME_BUFFER_SIZE];
        header_t Header;
        header_v2_t HeaderV2;
		header_v3_t HeaderV3;
    } DataFrame;
    uint16_t Count;
    uint8_t Status;
    uint8_t Version;
    uint16_t Checksum;
    crc16_t crc16;
} DataFrameBuffer_t;

typedef struct
{	
	uint16_t ModueType;
	uint16_t ModuleIndex;
	uint8_t log_data[LOG_DATA_SIZE];
}__attribute__((packed))LoggerData_t;

typedef struct
{
    long long TimeStamp;          //UTC in ms
	uint16_t ControlBits;
    uint32_t RuleType;
    uint16_t  RuleVersion;
	uint32_t data_len;
    LoggerData_t LogData;
}__attribute__((packed))LoggerUniverLog_t;

typedef struct
{
    uint8_t  RequestType;        //0x0E, 3 net gate request
    uint32_t MessageID;          //self increate
    uint8_t  Subject[16];        //subject
    uint32_t MessageTag;
    LoggerUniverLog_t  Data;
}__attribute__((packed))LoggerUniverLogMsg_t;

typedef struct {
	uint8_t Type;
	uint8_t Index;
	uint8_t Port;
	uint8_t Status;
	uint32_t HardwareVersion;
	uint32_t SoftwareVersion;
	uint32_t id[3];
} __attribute__((packed)) API_OldModuleInfo_t;	//模块信息

typedef struct {
	uint8_t num;	//模块数目
	API_OldModuleInfo_t moduleinfo[MODULE_MAXNUM];
} __attribute__((packed)) API_OldModuleLite_t;	//模块列表

static DataFrameBuffer_t Recv_Buffer;
static API_Config_t* api_config = NULL;
static API_Config_t* api_sub_config = NULL;

static uint8_t module_key = 0;
static uint32_t heartbeat = 0;
static uint32_t api_time = 0;
static uint32_t to=0x00010001;
static uint8_t fc_version=2;
static uint8_t api_version=3;
static API_ModuleLite_t module_list;

static uint8_t SendBuffer[FRAME_BUFFER_SIZE];

static void XorData(uint8_t* data, uint16_t length, uint8_t key);
static uint32_t check_sum(uint8_t* data, uint16_t len);

void api_get_fc_protocol_version(uint8_t *fc_protocol_version)
{
	*fc_protocol_version = fc_version;
}

static void api_tx_init(uint32_t from, uint32_t to, uint8_t cmd, uint16_t data_len)
{
	if(fc_version==3){
		SendBuffer[0] = 0x5C;   //START
		SendBuffer[1] = 0x5C;   //START
		SendBuffer[4] = data_len;   //Length
		SendBuffer[5] = data_len >> 8;
		SendBuffer[6] = (uint8_t)from;
		SendBuffer[7] = (uint8_t)(from>>8);
		SendBuffer[8] = (uint8_t)(from>>16);
		SendBuffer[9] = (uint8_t)(from>>24);
		SendBuffer[10] = (uint8_t) to;
		SendBuffer[11] = (uint8_t)(to>>8);
		SendBuffer[12] = (uint8_t)(to>>16);
		SendBuffer[13] = (uint8_t)(to>>24);
		SendBuffer[14] = cmd;  //Command

		SendBuffer[15 + data_len] = 0x66; //END
		SendBuffer[16 + data_len] = 0x66; //END
	}
	else
	{
		SendBuffer[0] = 0x5B;   //START
		SendBuffer[1] = 0x5B;   //START
		SendBuffer[4] = data_len;   //Length
		SendBuffer[5] = data_len >> 8;
		SendBuffer[6] = ((uint8_t)(from>>16))<<3|((uint8_t)from);
		SendBuffer[7] = ((uint8_t)(to>>16))<<3|((uint8_t)to);
		SendBuffer[8] = cmd;  //Command

		SendBuffer[9 + data_len] = 0x66; //END
		SendBuffer[10 + data_len] = 0x66; //END
	}
}

static void api_tx_checksum(uint16_t data_len)
{
    uint16_t checksum=0;
	if(fc_version==3){
		checksum= crc_ccitt(&SendBuffer[6], data_len + 9);
	}
	else{
		checksum= crc_ccitt(&SendBuffer[6], data_len + 3);
	}
	SendBuffer[2] = checksum;
	SendBuffer[3] = checksum >> 8;
}

void api_sub_module_register(API_Config_t* config)
{
    uint16_t msg_len = 36,frame_len=0;
	api_sub_config = config;
	if (api_sub_config == NULL) return;
    api_sub_config->Status = MODULE_STATUS_OFFLINE;
    api_sub_config->ModuleNo = api_sub_config->ModuleType << 16;
	if(api_sub_config->ModuleType>0x001f)
		fc_version=3;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    api_tx_init(api_sub_config->ModuleNo, to, CMD_SUB_MODULE_REGISTER, msg_len);
    memcpy(&SendBuffer[frame_len], &api_sub_config->ModuleID, 12);
	frame_len+=12;
    memcpy(&SendBuffer[frame_len], &api_sub_config->DataSize, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &api_sub_config->HardwareVersion, 4);
	frame_len+=4;
    memcpy(&SendBuffer[frame_len], &api_sub_config->SoftwareVersion, 4);
	frame_len+=4;
    memcpy(&SendBuffer[frame_len], &api_sub_config->ModuleTimeout, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &api_config->Baudrate, 1);
	frame_len+=1;
    memcpy(&SendBuffer[frame_len], &api_sub_config->RequireIndex, 2);
	frame_len+=2;
	memcpy(&SendBuffer[frame_len], &api_version, 1);
	frame_len+=1;
	memcpy(&SendBuffer[frame_len], &api_sub_config->UseId, 2);
	frame_len+=2;
	memcpy(&SendBuffer[frame_len], &api_sub_config->Database, 1);
	frame_len+=1;
	memcpy(&SendBuffer[frame_len], &api_sub_config->NodeDegree, 1);
	frame_len+=1;
	memcpy(&SendBuffer[frame_len], &api_sub_config->Reserve, 4);
	frame_len+=4;
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
}

void api_sub_module_unregister(API_Config_t* config)
{
    uint16_t msg_len = 4,frame_len=0;
	api_sub_config = config;
	if (api_sub_config == NULL) return;
	if(api_sub_config->ModuleType>0x001f)
		fc_version=3;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    api_tx_init(api_sub_config->ModuleNo, to, CMD_SUB_MODULE_UNREGISTER, msg_len);
    memcpy(&SendBuffer[frame_len], &api_sub_config->ModuleNo, 4);
	frame_len+=4;
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
}

static void module_register(void)
{
    uint16_t msg_len = 36,frame_len=0;
    if (api_config == NULL) return;
	if(api_config->ModuleType>0x001f)
		fc_version=3;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    api_tx_init(api_config->ModuleNo, to, CMD_REGISTER, msg_len);
    memcpy(&SendBuffer[frame_len], &api_config->ModuleID, 12);
	frame_len+=12;
    memcpy(&SendBuffer[frame_len], &api_config->DataSize, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &api_config->HardwareVersion, 4);
	frame_len+=4;
    memcpy(&SendBuffer[frame_len], &api_config->SoftwareVersion, 4);
	frame_len+=4;
    memcpy(&SendBuffer[frame_len], &api_config->ModuleTimeout, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &api_config->Baudrate, 1);
	frame_len+=1;
    memcpy(&SendBuffer[frame_len], &api_config->RequireIndex, 2);
	frame_len+=2;
	memcpy(&SendBuffer[frame_len], &api_version, 1);
	frame_len+=1;
	memcpy(&SendBuffer[frame_len], &api_config->UseId, 2);
	frame_len+=2;
	memcpy(&SendBuffer[frame_len], &api_config->Database, 1);
	frame_len+=1;
	memcpy(&SendBuffer[frame_len], &api_config->NodeDegree, 1);
	frame_len+=1;
	memcpy(&SendBuffer[frame_len], &api_config->Reserve, 4);
	frame_len+=4;
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
}

uint8_t module_heartbeat(void)
{
    uint16_t msg_len = 0,frame_len=0;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    api_tx_init(api_config->ModuleNo, to, CMD_HEARTBEAT, msg_len);
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
    return 0;
}


void api_get_modulelist(uint8_t page_index)
{
	uint16_t msg_len = 2,frame_len=0;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
	if(fc_version==3)
	{
		msg_len = 2;
		frame_len=FRAME_HEAD_SIZE;
		api_tx_init(api_config->ModuleNo, to, CMD_GET_MODULE, msg_len);
		SendBuffer[frame_len]=2;
		frame_len+=1;
		SendBuffer[frame_len]=page_index;
		frame_len+=1;
		api_tx_checksum(msg_len);
		api_port_send(frame_len + 2, SendBuffer);
	}
	else
	{
		msg_len = 0;
		frame_len=FRAME_OLDHEAD_SIZE;
		api_tx_init(api_config->ModuleNo, to, CMD_GET_MODULE, msg_len);
		api_tx_checksum(msg_len);
		api_port_send(frame_len + 2, SendBuffer);
	}
}

void api_get_data(int32_t offset, int32_t len)
{
    uint16_t msg_len = 4,frame_len=0;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    api_tx_init(api_config->ModuleNo, to, CMD_GET_DATA, msg_len);
    memcpy(&SendBuffer[frame_len], &offset, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &len, 2);
	frame_len+=2;
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
}

void api_set_data(int32_t offset, int32_t len, void* data, uint8_t no_ack)
{
    uint16_t msg_len = len + 4;
	uint16_t frame_len=0;
    if (len == 0 || api_config->Status == MODULE_STATUS_OFFLINE) return;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    if (no_ack)
        api_tx_init(api_config->ModuleNo, to, CMD_SET_DATA | 0x80, msg_len);
    else
        api_tx_init(api_config->ModuleNo, to, CMD_SET_DATA, msg_len);
    memcpy(&SendBuffer[frame_len], &offset, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &len, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], data, len);
    XorData(&SendBuffer[frame_len], len, module_key);
	frame_len+=len;
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
}

static void api_send_message_old(API_Message_t* msg)
{
    uint16_t msg_len = 0;
	uint16_t msg_data_len = 0;
	uint16_t frame_len=0;
	msg_data_len = msg->msg_data.Length + 1 + 4;	//datalen + 1byte cmd + 4bytes to/from
	msg_len = msg_data_len + 2;			// + 2bytes len  (2bytes len + 1byte cmd + 4bytes to/from)
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
	if (fc_version == 1)
    {
		msg_len-=3;
		msg_data_len-=3;	//to/from  4bytes->1byte so len-3
        SendBuffer[0] = 0x5A;   //START
        SendBuffer[2] = msg_len;   //Length
        SendBuffer[3] = (uint8_t)(msg_len >> 8);
        SendBuffer[4] = ((uint8_t)(api_config->ModuleNo>>16))<<3|((uint8_t)api_config->ModuleNo);
        SendBuffer[5] = ((uint8_t)(msg->To>>16))<<3|((uint8_t)msg->To);
        SendBuffer[6] = CMD_SEND_MSG;  //Command
        SendBuffer[7] = msg_data_len;
        SendBuffer[8] = msg_data_len >> 0x08;
		SendBuffer[9] = msg->msg_data.cmd;
		SendBuffer[10] =(uint8_t)((msg->msg_data.to_or_from>>16)<<3)|(msg->msg_data.to_or_from&0x00000007);
		if(msg->msg_data.Length<=(FRAME_BUFFER_SIZE-11))
		{
			memcpy(&SendBuffer[11], msg->msg_data.Data, msg->msg_data.Length);
		}
        SendBuffer[1] = check_sum(&SendBuffer[4], msg_len + 3);	//msg_len + 1byte from +1byte to +1byte cmd
        api_port_send(msg_len + 7, SendBuffer);
    }
	else if(fc_version == 2)
	{
		frame_len=FRAME_OLDHEAD_SIZE;
		msg_len-=3;
		msg_data_len-=3;	//to/from  4bytes->1byte so len-3
		api_tx_init(api_config->ModuleNo, msg->To, CMD_SEND_MSG, msg_len);
		SendBuffer[frame_len] = msg_data_len;
		frame_len+=1;
		SendBuffer[frame_len] = msg_data_len >> 0x08;
		frame_len+=1;
		SendBuffer[frame_len] = msg->msg_data.cmd;
		frame_len++;
		SendBuffer[frame_len] = (uint8_t)((msg->msg_data.to_or_from>>16)<<3)|(msg->msg_data.to_or_from&0x00000007);
		frame_len++;
		if(msg->msg_data.Length<=(FRAME_BUFFER_SIZE-frame_len))
		{
			memcpy(&SendBuffer[frame_len],msg->msg_data.Data, msg->msg_data.Length);
		}
		frame_len=frame_len+msg->msg_data.Length;
		api_tx_checksum(msg_len);
		api_port_send(frame_len + 2, SendBuffer);
	}
	else
	{
		frame_len=FRAME_HEAD_SIZE;
		api_tx_init(api_config->ModuleNo, msg->To, CMD_SEND_MSG, msg_len);
		SendBuffer[frame_len] = msg_data_len;
		frame_len+=1;
		SendBuffer[frame_len] = msg_data_len >> 0x08;
		frame_len+=1;
		SendBuffer[frame_len]=msg->msg_data.cmd;
		frame_len+=1;
		memcpy(&SendBuffer[frame_len], &msg->msg_data.to_or_from, 4);
		frame_len+=4;
		if(msg->msg_data.Length<=(FRAME_BUFFER_SIZE-frame_len))
		{
			memcpy(&SendBuffer[frame_len], msg->msg_data.Data, msg->msg_data.Length);
		}
		frame_len=frame_len+msg->msg_data.Length;
		api_tx_checksum(msg_len);
		api_port_send(frame_len + 2, SendBuffer);
	}
}

void api_send_message(API_Message_t* msg)
{
    uint16_t msg_len = 0;
    uint16_t msg_data_len = 0;
	uint16_t frame_len=0;
	msg_data_len = msg->msg_data.Length + NET_MSG_OFFSET + 1 + 4;//datalen + NET_MSG_OFFSET + 1byte cmd + 4bytes to/from
	msg_len = msg_data_len + 2;		//msg_data_len + 2bytes len
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
	
	if(msg_data_len>(FRAME_BUFFER_SIZE-17-2))return;	//17 (header、end) 2 len
	
	if((msg->From_Port==0&&msg->To_Port==0)||fc_version<3)
	{
		msg->From_Port=0;
		msg->To_Port=0;
		api_send_message_old(msg);
	}
	else
	{
		frame_len=FRAME_HEAD_SIZE;
		api_tx_init(api_config->ModuleNo, msg->To, CMD_SEND_NET_MSG, msg_len);
		SendBuffer[frame_len] = msg_data_len;
		frame_len+=1;
		SendBuffer[frame_len] = msg_data_len >> 0x08;
		frame_len+=1;
		memcpy(&SendBuffer[frame_len], &msg->From_Port, 4);
		frame_len+=4;
		memcpy(&SendBuffer[frame_len], &msg->To_Port, 4);
		frame_len+=4;
		SendBuffer[frame_len]=msg->Request_Id;
		frame_len+=1;
		SendBuffer[frame_len]=msg->Respond_Id;
		frame_len+=1;
		SendBuffer[frame_len]=msg->msg_data.cmd;
		frame_len+=1;
		memcpy(&SendBuffer[frame_len], &msg->msg_data.to_or_from, 4);
		frame_len+=4;
		if((msg->msg_data.Length)<=(FRAME_BUFFER_SIZE-frame_len))
		{
			memcpy(&SendBuffer[frame_len],msg->msg_data.Data, msg->msg_data.Length);
		}
		frame_len+=msg->msg_data.Length;
		api_tx_checksum(msg_len);
		api_port_send(frame_len + 2, SendBuffer);
	}
}

void api_get_upgrade_status(void)
{
	uint16_t msg_len = 1,frame_len=0;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    api_tx_init(api_config->ModuleNo, to, CMD_UPGRADE_STATUS, msg_len);
	SendBuffer[frame_len]=0x01;
	frame_len+=1;
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
}

void api_logger_sendUniverLog(UniverLog_t* univerlog)
{
	static uint32_t msg_count=0;
	static API_Message_t cloud_log_msg;
    static LoggerUniverLogMsg_t UniverLogMsg;
	static uint8_t UniverLog_subject[] = "UNIVERLOG";
	static uint32_t fy_time=0;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
	
	if(api_time-fy_time<50&&fy_time!=0)
	{
		return;
	}
	else
	{
		fy_time=api_time;
	}
	
    UniverLogMsg.RequestType = 0x0E; //0x0e 三网关请求
    UniverLogMsg.MessageID = ++msg_count;
    memcpy(&UniverLogMsg.Subject, &UniverLog_subject, sizeof(UniverLog_subject));
    UniverLogMsg.MessageTag = 0;
    UniverLogMsg.Data.TimeStamp = 0;
    UniverLogMsg.Data.RuleType = univerlog->rule_type;
    UniverLogMsg.Data.RuleVersion = univerlog->rule_ver;
	UniverLogMsg.Data.LogData.ModuleIndex = (uint16_t)api_config->ModuleNo;
	UniverLogMsg.Data.LogData.ModueType = api_config->ModuleType;
    UniverLogMsg.Data.ControlBits = 0;
	UniverLogMsg.Data.data_len = univerlog->data_len + 4;	//2bytes module_type 2bytes module_index
	if(univerlog->data_len<LOG_DATA_SIZE)
	{
		memcpy(UniverLogMsg.Data.LogData.log_data,univerlog->data_buff,univerlog->data_len);
	}
	cloud_log_msg.From=api_config->ModuleNo;
	cloud_log_msg.To=0x00050001;
	cloud_log_msg.From_Port=api_config->ModuleNo;
	cloud_log_msg.To_Port=0x00440000;
	cloud_log_msg.Request_Id=msg_count;
	cloud_log_msg.Respond_Id=0;
	cloud_log_msg.msg_data.cmd=0x86;
	cloud_log_msg.msg_data.to_or_from=0x00050001;
	cloud_log_msg.msg_data.Data=(uint8_t*)&UniverLogMsg;
	cloud_log_msg.msg_data.Length=sizeof(LoggerUniverLogMsg_t)-sizeof(LoggerData_t)+ univerlog->data_len + 4;	//2bytes module_type 2bytes module_index
	api_send_message(&cloud_log_msg);
}

void api_auto_load_data(int32_t offset, int32_t len, uint16_t enabled, uint16_t interval)
{
    uint16_t msg_len = 8, t = enabled;
	uint16_t frame_len=0;
	if(fc_version==3)
		frame_len=FRAME_HEAD_SIZE;
	else
		frame_len=FRAME_OLDHEAD_SIZE;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
    api_tx_init(api_config->ModuleNo, to, CMD_AUTO_LOAD_DATA, msg_len);
    memcpy(&SendBuffer[frame_len], &offset, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &len, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &t, 2);
	frame_len+=2;
    memcpy(&SendBuffer[frame_len], &interval, 2);
	frame_len+=2;
    api_tx_checksum(msg_len);
    api_port_send(frame_len + 2, SendBuffer);
}

void api_upgrade_request(void)
{
	uint16_t sum = 0;
    SendBuffer[0] = 0x5c;   //V3.5 START
	SendBuffer[1] = 0x5c;   //V3.5 START
	
    SendBuffer[4] = 0x00;   //Length low 8 bit
    SendBuffer[5] = 0x00;	//Length high 8 bit
	
	SendBuffer[6] = api_config->ModuleIndex;	//from index low 8 bit
	SendBuffer[7] = api_config->ModuleIndex>>8;	//from index high 8 bit
    SendBuffer[8] = api_config->ModuleType;		//from type low 8 bit
	SendBuffer[9] = api_config->ModuleType>>8; 	//from type high 8 bit

	SendBuffer[10] = 0x01;	//to type low 8 bit
	SendBuffer[11] = 0x00; 	//to type high 8 bit
	SendBuffer[12] = 0x01;	//to index low 8 bit
	SendBuffer[13] = 0x00;	//to index high 8 bit

    SendBuffer[14] = CMD_UPGRADE_REQUEST;  //Command
	
	SendBuffer[15] = 0x66;   //V3.5 END
	SendBuffer[16] = 0x66;   //V3.5 END
	
	sum	= crc_ccitt(&SendBuffer[6], 3 + 6);
    SendBuffer[2] = (sum & 0xff);//check_sum low 8 bit
	SendBuffer[3] = (sum >> 8);	//check_sum high 8 bit
	api_port_send((SendBuffer[4] | (SendBuffer[5] << 8)) + 6 + 9 + 2, SendBuffer);
}

void api_module_upgrade_ack(uint8_t ack)
{
    uint16_t sum = 0;
    SendBuffer[0] = 0x5c;   //V3.5 START
	SendBuffer[1] = 0x5c;   //V3.5 START
	
    SendBuffer[4] = 0x01;   //Length low 8 bit
    SendBuffer[5] = 0x00;	//Length high 8 bit
	
	SendBuffer[6] = api_config->ModuleIndex;	//from index low 8 bit
	SendBuffer[7] = api_config->ModuleIndex>>8;	//from index high 8 bit
    SendBuffer[8] = api_config->ModuleType;		//from type low 8 bit
	SendBuffer[9] = api_config->ModuleType>>8; 	//from type high 8 bit
	
	SendBuffer[10] = 0x01;	//to type low 8 bit
	SendBuffer[11] = 0x00; 	//to type high 8 bit
	SendBuffer[12] = 0x01;	//to index low 8 bit
	SendBuffer[13] = 0x00;	//to index high 8 bit

    SendBuffer[14] = CMD_UPGRADE_PK;  //Command
	
    SendBuffer[15] = ack; //data
	
	SendBuffer[16] = 0x66;   //V3.5 END
	SendBuffer[17] = 0x66;   //V3.5 END
	
	sum	= crc_ccitt(&SendBuffer[6], (SendBuffer[4] | (SendBuffer[5] << 8)) + 3 + 6);
    SendBuffer[2] = (sum & 0xff);//check_sum low 8 bit
	SendBuffer[3] = (sum >> 8);	//check_sum high 8 bit
	
    api_port_send((SendBuffer[4] | (SendBuffer[5] << 8)) + 6 + 9 + 2,SendBuffer);
}

void api_module_upgrade_finish(void)
{
    uint16_t sum = 0;
    SendBuffer[0] = 0x5c;   //V3.5 START
	SendBuffer[1] = 0x5c;   //V3.5 START
	
    SendBuffer[4] = 0x00;   //Length low 8 bit
    SendBuffer[5] = 0x00;	//Length high 8 bit
	
	SendBuffer[6] = api_config->ModuleIndex;	//from index low 8 bit
	SendBuffer[7] = api_config->ModuleIndex>>8;	//from index high 8 bit
    SendBuffer[8] = api_config->ModuleType;		//from type low 8 bit
	SendBuffer[9] = api_config->ModuleType>>8; 	//from type high 8 bit
	
	SendBuffer[10] = 0x01;	//to type low 8 bit
	SendBuffer[11] = 0x00; 	//to type high 8 bit
	SendBuffer[12] = 0x01;	//to index low 8 bit
	SendBuffer[13] = 0x00;	//to index high 8 bit

    SendBuffer[14] = CMD_UPGRADE_FINISH;  //Command
	
	SendBuffer[15] = 0x66;   //V3.5 END
	SendBuffer[16] = 0x66;   //V3.5 END
	
	sum	= crc_ccitt(&SendBuffer[6], (SendBuffer[4] | (SendBuffer[5] << 8)) + 3 + 6);
    SendBuffer[2] = (sum & 0xff);//check_sum low 8 bit
	SendBuffer[3] = (sum >> 8);	//check_sum high 8 bit
	
    api_port_send((SendBuffer[4] | (SendBuffer[5] << 8)) + 6 + 9 + 2,SendBuffer);
}

static void module_response(void)
{
    uint8_t cmd;
    uint16_t length;
    uint8_t* data;
	uint8_t version=Recv_Buffer.Version;
	uint8_t module_count=0;
	uint8_t module_no;
	uint32_t file_size;
	uint16_t TotalPackage;
	API_Message_t api_msg;
	switch(version)
	{
		case 1:
			cmd = Recv_Buffer.DataFrame.Header.CMD;
			length = Recv_Buffer.DataFrame.Header.Length;
			data = &Recv_Buffer.DataFrame.RawData[7];
			break;
		case 2:
			cmd = Recv_Buffer.DataFrame.HeaderV2.CMD;
			length = Recv_Buffer.DataFrame.HeaderV2.Length;
			data = &Recv_Buffer.DataFrame.RawData[7];
			break;
		case 3:
			cmd = Recv_Buffer.DataFrame.HeaderV3.CMD;
			length = Recv_Buffer.DataFrame.HeaderV3.Length;
			data = &Recv_Buffer.DataFrame.RawData[13];
			break;
		default:
			cmd = Recv_Buffer.DataFrame.HeaderV3.CMD;
			length = Recv_Buffer.DataFrame.HeaderV3.Length;
			data = &Recv_Buffer.DataFrame.RawData[13];
			break;
	}
    switch (cmd)
    {
        case CMD_NOMODULE:
			if((version==1||version==2))
			{
				module_no=(uint8_t)(((api_config->ModuleNo>>16)<<3)|(api_config->ModuleNo&0x00000007));
				if(Recv_Buffer.DataFrame.Header.From!=0&&Recv_Buffer.DataFrame.Header.To==module_no)
				{
					api_config->Status = MODULE_STATUS_OFFLINE;
				}
			}
			else
			{
				if(Recv_Buffer.DataFrame.Header.From!=0&&Recv_Buffer.DataFrame.HeaderV3.To==api_config->ModuleNo)
				{
					api_config->Status = MODULE_STATUS_OFFLINE;
				}
			}
            break;
        case CMD_REGISTER:
            if (length == 13)
            {
                //module_key = 0x00;
                module_key = data[12];
				if(version==1||version==2)
				{
					api_config->ModuleNo = ((uint16_t)(Recv_Buffer.DataFrame.Header.To>>3)<<16)|(Recv_Buffer.DataFrame.Header.To&0x07);
				}
				else
				{
					api_config->ModuleNo = Recv_Buffer.DataFrame.HeaderV3.To;
					fc_version=3;
				}
				api_config->ModuleType = api_config->ModuleNo >> 16;
				api_config->ModuleIndex = (uint16_t)api_config->ModuleNo;
                heartbeat = api_time;
				api_config->Status = MODULE_STATUS_ONLINE;
            }
            break;
		case CMD_SUB_MODULE_REGISTER:
            if (length == 13)
            {
                module_key = data[12];
				if(version==1||version==2)
				{
					api_sub_config->ModuleNo = ((uint16_t)(Recv_Buffer.DataFrame.Header.To>>3)<<16)|(Recv_Buffer.DataFrame.Header.To&0x07);
				}
				else
				{
					api_sub_config->ModuleNo = Recv_Buffer.DataFrame.HeaderV3.To;
				}
				api_sub_config->ModuleType = api_sub_config->ModuleNo >> 16;
				api_sub_config->ModuleIndex = (uint16_t)api_sub_config->ModuleNo;
                heartbeat = api_time;
				api_sub_config->Status = MODULE_STATUS_ONLINE;
            }
            break;
		case CMD_SUB_MODULE_UNREGISTER:
			api_sub_config->Status = MODULE_STATUS_OFFLINE;
			api_sub_config->ModuleIndex = 0;
			api_sub_config->ModuleNo &= 0xFFFF0000;
            break;
        case CMD_HEARTBEAT:
            heartbeat = api_time;
            break;
		case CMD_GET_MODULE:
			heartbeat = api_time;
			API_OldModuleLite_t *old_module_list;
			module_count=data[0];
			if((module_count*sizeof(API_OldModuleInfo_t))==(length-1))
			{
				old_module_list=(API_OldModuleLite_t *)data;
				module_list.module_count=module_count;
				module_list.page_index=0;
				module_list.page_count=1;
				for(int module_i=0;module_i<module_count;module_i++)
				{
					module_list.moduleinfo[module_i].Type=old_module_list->moduleinfo[module_i].Type;
					module_list.moduleinfo[module_i].Index=old_module_list->moduleinfo[module_i].Index;
					module_list.moduleinfo[module_i].Port=old_module_list->moduleinfo[module_i].Port;
					module_list.moduleinfo[module_i].Status=old_module_list->moduleinfo[module_i].Status;
					module_list.moduleinfo[module_i].HardwareVersion=old_module_list->moduleinfo[module_i].HardwareVersion;
					module_list.moduleinfo[module_i].SoftwareVersion=old_module_list->moduleinfo[module_i].SoftwareVersion;
					memcpy((uint8_t*)&module_list.moduleinfo[module_i].id[0],(uint8_t*)&old_module_list->moduleinfo[module_i].id[0],4);
					memcpy((uint8_t*)&module_list.moduleinfo[module_i].id[1],(uint8_t*)&old_module_list->moduleinfo[module_i].id[1],4);
					memcpy((uint8_t*)&module_list.moduleinfo[module_i].id[2],(uint8_t*)&old_module_list->moduleinfo[module_i].id[2],4);
					module_list.moduleinfo[module_i].ProtocolVersion=0;
					module_list.moduleinfo[module_i].use_id=0;
					module_list.moduleinfo[module_i].node_degree=0;
					module_list.moduleinfo[module_i].reserve[0]=0;
					module_list.moduleinfo[module_i].reserve[1]=0;
					module_list.moduleinfo[module_i].reserve[2]=0;
				}
			}
			else if((module_count*sizeof(API_ModuleInfo_t))==(length-1))
			{
				module_list.module_count=module_count;
				module_list.page_index=0;
				module_list.page_count=1;
				memcpy((uint8_t*)module_list.moduleinfo,&data[1],(length-1));
			}
			else
			{
				module_list.module_count=module_count;
				module_list.page_index=data[1];
				module_list.page_count=data[2];
				memcpy((uint8_t*)module_list.moduleinfo,&data[3],(length-3));
				//api_on_modulelist_received((API_ModuleLite_t *)data);
			}
			api_on_modulelist_received(&module_list);
			break;
        case CMD_GET_DATA:
            heartbeat = api_time;
            if (length > 4)
            {
                uint16_t offset = 0, len = 0;
                memcpy(&offset, &data[0], 2);
                memcpy(&len, &data[2], 2);
                XorData(&data[4], len, module_key);
                api_on_data_returned(offset, len, &data[4]);
            }
            break;
        case CMD_SET_DATA:
            heartbeat = api_time;
            api_on_data_set(1);
            break;
        case CMD_SEND_MSG:
            {
                heartbeat = api_time;
				api_msg.From_Port=0;
				api_msg.To_Port=0;
				api_msg.Request_Id=0;
				api_msg.Respond_Id=0;
				if(version==1||version==2)
				{
					api_msg.From = ((uint16_t)(Recv_Buffer.DataFrame.Header.From>>3)<<16)|(Recv_Buffer.DataFrame.Header.From&0x07);
					api_msg.To = ((uint16_t)(Recv_Buffer.DataFrame.Header.To>>3)<<16)|(Recv_Buffer.DataFrame.Header.To&0x07);
					if(Recv_Buffer.DataFrame.Header.Length>=4)
					{
						api_msg.msg_data.Length = Recv_Buffer.DataFrame.Header.Length - 2 - 1 - 1;	//-2bytes len - 1byte cmd -1byte to/from
					}
					else
					{
						api_msg.msg_data.Length = 0;
					}
					api_msg.msg_data.cmd=Recv_Buffer.DataFrame.RawData[9];
					api_msg.msg_data.to_or_from=((Recv_Buffer.DataFrame.RawData[10]>>3)<<16)|(Recv_Buffer.DataFrame.RawData[10]&0x07);
					api_msg.msg_data.Data=&Recv_Buffer.DataFrame.RawData[11];
				}
				else
				{
					api_msg.From = Recv_Buffer.DataFrame.HeaderV3.From;
					api_msg.To = Recv_Buffer.DataFrame.HeaderV3.To;
					if(Recv_Buffer.DataFrame.HeaderV3.Length>=7)
					{
						api_msg.msg_data.Length = Recv_Buffer.DataFrame.HeaderV3.Length  - 2 - 1 - 4;	//-2bytes len - 1byte cmd -4byte to/from
					}
					else
					{
						api_msg.msg_data.Length = 0;
					}
					api_msg.msg_data.cmd=Recv_Buffer.DataFrame.RawData[15];
					memcpy(&api_msg.msg_data.to_or_from,&Recv_Buffer.DataFrame.RawData[16],4);
					api_msg.msg_data.Data = &Recv_Buffer.DataFrame.RawData[20];
				}
                api_on_message_received(api_msg);
            }
            break;
        case CMD_SEND_NET_MSG:
            {
                heartbeat = api_time;
				api_msg.From = Recv_Buffer.DataFrame.HeaderV3.From;
				api_msg.To = Recv_Buffer.DataFrame.HeaderV3.To;
				if(Recv_Buffer.DataFrame.HeaderV3.Length>=(2+NET_MSG_OFFSET+1+4))
				{
					api_msg.msg_data.Length = Recv_Buffer.DataFrame.HeaderV3.Length-2-NET_MSG_OFFSET-1-4;	//-2bytes len -NET_MSG_OFFSET - 1byte cmd -4byte to/from
				}
				else
				{
					api_msg.msg_data.Length = 0;
				}
				memcpy(&api_msg.From_Port,&Recv_Buffer.DataFrame.RawData[15],4);
				memcpy(&api_msg.To_Port,&Recv_Buffer.DataFrame.RawData[19],4);
				api_msg.Request_Id=Recv_Buffer.DataFrame.RawData[23];
				api_msg.Respond_Id=Recv_Buffer.DataFrame.RawData[24];
				api_msg.msg_data.cmd=Recv_Buffer.DataFrame.RawData[15+NET_MSG_OFFSET];
				memcpy(&api_msg.msg_data.to_or_from,&Recv_Buffer.DataFrame.RawData[16+NET_MSG_OFFSET],4);
				api_msg.msg_data.Data = &Recv_Buffer.DataFrame.RawData[20+NET_MSG_OFFSET];
                api_on_message_received(api_msg);
            }
            break;
        case CMD_UPGRADE:
            api_on_upgrade();
		    break;
		case CMD_UPGRADE_REQUEST:
			{				
                heartbeat = api_time;
				memcpy(&file_size,&data[0],4);
				memcpy(&TotalPackage,&data[4],2);
                api_on_upgrade_requst(file_size,TotalPackage);
            }
			break;
		case CMD_UPGRADE_PK:
			{
                heartbeat = api_time;
                api_on_upgrade_pack(data[0],&data[1]);
            }
			break;
		case CMD_UPGRADE_FINISH:
			{
                heartbeat = api_time;
                api_on_upgrade_finsh();
            }
		    break;
		case CMD_UPGRADE_STATUS:
			heartbeat = api_time;
			api_on_upgrade_status_returned((UpgradeStatus_t *)data);
			break;
    }

}

void api_init(API_Config_t* config)
{
    api_config = config;
    api_config->Status = MODULE_STATUS_OFFLINE;
    api_config->ModuleNo = api_config->ModuleType << 16;
}

void api_loop(uint32_t time)
{
    static uint32_t last_tick = 0;
    static uint16_t heartbeat_interval = 1000;
    api_time = time;
    if (api_config == NULL) return;

    if (api_config->Status == MODULE_STATUS_OFFLINE)
    {
        if (last_tick + 500 < time || last_tick == 0)
        {
            last_tick = time;
            heartbeat_interval = api_config->ModuleTimeout <= 1500 ? 500 : api_config->ModuleTimeout / 3;
            module_register();
        }
        return;
    }

    //if ((last_tick + api_config->ModuleTimeout - 1000) <= time)
    if (time - last_tick >= heartbeat_interval)
    {
        last_tick = time;
        module_heartbeat();
    }
    else if (time - heartbeat >= api_config->ModuleTimeout)
    {
        api_config->Status = MODULE_STATUS_OFFLINE;
        api_config->Status = 0;
        api_config->ModuleIndex = 0;
        api_config->ModuleNo &= 0xFFFF0000;
    }

}

static uint8_t module_parse_frame(uint8_t data)
{
    static uint8_t v2Counter = 0;
	static uint8_t v3Counter = 0;
    switch (Recv_Buffer.Status)
    {
        case FRAME_SYNC:
            if (data == 0x5A)
            {
                Recv_Buffer.DataFrame.RawData[Recv_Buffer.Count++] = data;
                Recv_Buffer.Version = 1;
                Recv_Buffer.Status = FRAME_HEADER;
                break;
            }
            if (data == 0x5B)
            {
                if (++v2Counter == 2)
                {
                    Recv_Buffer.Version = 2;
                    Recv_Buffer.Status = FRAME_HEADER;
                    v2Counter = 0;
                }
                break;
            }
			if (data == 0x5C)
            {
                if (++v3Counter == 2)
                {
                    Recv_Buffer.Version = 3;
                    Recv_Buffer.Status = FRAME_HEADER;
                    v3Counter = 0;
                }
                break;
            }
            goto API_FRAME_RESET;

        case FRAME_HEADER:
            Recv_Buffer.DataFrame.RawData[Recv_Buffer.Count++] = data;
            if (Recv_Buffer.Count == 4)//sizeof(header_t))
            {
                if (((Recv_Buffer.Version==1||Recv_Buffer.Version==2)&&Recv_Buffer.DataFrame.Header.Length <= (FRAME_BUFFER_SIZE - 7))
					||(Recv_Buffer.Version==3&&Recv_Buffer.DataFrame.Header.Length <= (FRAME_BUFFER_SIZE - 13)))
                {
                    Recv_Buffer.Checksum = 0;
                    Recv_Buffer.crc16.current = CRC16_INIT_VALUE;
                    Recv_Buffer.crc16.crc16 = CRC16_INIT_VALUE;
                    Recv_Buffer.Status = FRAME_DATA;
                }
                else    //帧长度异常
                {
                    goto API_FRAME_RESET;
                }
            }
            break;

        case FRAME_DATA: //from to cmd & data
            Recv_Buffer.DataFrame.RawData[Recv_Buffer.Count++] = data;
            if (Recv_Buffer.Version == 1) Recv_Buffer.Checksum += data;
            else crc16(data, &Recv_Buffer.crc16);
            if (((Recv_Buffer.Version==1||Recv_Buffer.Version==2)&&(Recv_Buffer.Count >= Recv_Buffer.DataFrame.Header.Length + 7))
				||(Recv_Buffer.Version==3&&(Recv_Buffer.Count >= Recv_Buffer.DataFrame.Header.Length + 13)))
            {
                if ((Recv_Buffer.Version == 1 && (Recv_Buffer.Checksum & 0xFF) == Recv_Buffer.DataFrame.Header.Checksum) ||
                    (Recv_Buffer.Version == 2 && Recv_Buffer.crc16.crc16 == Recv_Buffer.DataFrame.HeaderV2.Checksum)||
					(Recv_Buffer.Version == 3 && Recv_Buffer.crc16.crc16 == Recv_Buffer.DataFrame.HeaderV3.Checksum))
                {//帧数据校验通过
                    //处理模块帧数据
                    module_response();
                }
                else
                {
                    //帧数据校验错误
                }
                //接收一帧数据完成
                goto API_FRAME_RESET;
            }
            break;

        default:
            goto API_FRAME_RESET;
    }

    return 0;
    
    API_FRAME_RESET:
    v2Counter = 0;
	v3Counter = 0;
    Recv_Buffer.Count = 0;
    Recv_Buffer.Status = FRAME_SYNC;
    return 0;
}

void api_port_received(int32_t len, uint8_t* data)
{
    while (len--)
    {
        module_parse_frame(*data++);
    }
}
/* Compiler Related Definitions */
#ifdef __CC_ARM                         /* ARM Compiler */

__weak void api_port_send(int32_t len, uint8_t* data)
{

}
__weak void api_on_data_set(uint8_t result)
{

}

__weak void api_on_data_returned(int32_t offset, int32_t len, uint8_t* data)
{

}

__weak void api_on_message_received(API_Message_t message)
{

}

__weak void api_on_upgrade(void)
{

}

__weak void api_on_upgrade_requst(uint32_t file_size,uint16_t TotalPackage)
{

}

__weak void api_on_upgrade_pack(uint8_t pack_size,uint8_t* data)
{

}

__weak void api_on_upgrade_finsh(void)
{

}

__weak void api_on_modulelist_received(API_ModuleLite_t *modublelist)
{

}

__weak void api_on_upgrade_status_returned(UpgradeStatus_t *upgtade_status)
{

}
#elif defined ( __GNUC__ )
__weak void api_port_send(int32_t len, uint8_t* data)
{

}
__weak void api_on_data_set(uint8_t result)
{

}

__weak void api_on_data_returned(int32_t offset, int32_t len, uint8_t* data)
{

}
__weak void api_on_message_received(API_Message_t message)
{

}

__weak void api_on_upgrade(void)
{

}

__weak void api_on_modulelist_received(API_ModuleLite_t *modublelist)
{

}

__weak void api_on_oldmodulelist_received(API_OldModuleLite_t *modublelist)
{

}

__weak void api_on_upgrade_status_returned(UpgradeStatus_t *upgtade_status)
{

}

#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */

#pragma weak api_port_send
#pragma weak api_on_data_set
#pragma weak api_on_data_returned
#pragma weak api_on_message_received
#pragma weak api_on_upgrade
#pragma weak api_on_modulelist_received
#pragma weak api_on_oldmodulelist_received
#pragma weak api_on_upgrade_status_returned
#endif

//util
static void XorData(uint8_t* data, uint16_t length, uint8_t key)
{
    uint16_t index = 0;
    for (index = 0; index < length; index++)
    {
        data[index] = data[index] ^ key;
    }
}

static uint32_t check_sum(uint8_t* data, uint16_t len)
{
    uint32_t checksum = 0;
    while (len--)
    {
        checksum += *data++;
    }
    return checksum;
}


# ifdef __CC_ARM                         /* ARM Compiler */
//static uint16_t ccitt_table[256] __attribute__((section(".ccm"))) = {
static uint16_t ccitt_table[256] = {
#elif defined ( __GNUC__ )
static uint16_t ccitt_table[256] = {
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    static uint16_t ccitt_table[256] = {
#endif
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

void crc16(uint8_t input, crc16_t* output)
{
    output->current = ccitt_table[(output->current >> 8 ^ input) & 0xff] ^ (output->current << 8);
    output->crc16 = ~output->current;
}

uint16_t crc_ccitt(uint8_t* input, int len)
{
    uint16_t crc = CRC16_INIT_VALUE;
    while (len-- > 0)
        crc = ccitt_table[(crc >> 8 ^ *input++) & 0xff] ^ (crc << 8);
    return ~crc;
}
