#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>

#define RECEIVE_BUFFER_SIZE 32
#define TRANSMIT_BUFFER_SIZE (1+1+1+32)
static char* TAG = "SERIAL";

static enum
{
    FINDING_0X55 = 1,
    NEED_0XAA,
    NEED_LENGTH,
    GETTING_DATA
} com_serial_receiveStatus = FINDING_0X55;
static int32_t  com_serial_receiveLength = 0;
static uint8_t  com_serial_receiveBuffer[RECEIVE_BUFFER_SIZE]; // |version|type|content|crc|
static int32_t  com_serial_transmitLength = 0;
static uint16_t com_serial_transmitCrc = 0;
static uint8_t  com_serial_transmitBuffer[TRANSMIT_BUFFER_SIZE]; // |0x55|0xAA|length|version|type|content|crc|

int32_t com_serial_init(void);
void    com_serial_checkEvent(void);
int32_t com_serial_transmitBegin(uint8_t packetType);
int32_t com_serial_transmitContent(const void * part,int32_t length);
int32_t com_serial_transmitEnd(void);


extern rt_device_t _console_device;


/*************************
函数名：com_serial_init
功能：没看出来有啥功能
参数：无
返回值：0
**************************/
int32_t com_serial_init(void)
{
    return 0;
}



/*************************
函数名：com_serial_checkEvent
功能：串口事件检测 （有待确认）
参数：无
返回值: 无
**************************/
void com_serial_checkEvent(void)
{
    //log(TAG , "com_serial_checkEvent: receiveStatus=%d" , com_serial_receiveStatus);
    switch(com_serial_receiveStatus)
    {
        case FINDING_0X55 :
        {
//            if(uart1_receiveValidBufferSize() < 1)
//                break;
//            uint8_t byte = 0;
//            uart1_readReceiveBuffer(&byte,1);
			
			uint8_t byte = 0 , r_byte;
			r_byte = rt_device_read(_console_device , 0 , &byte , 1);
			if(!r_byte)
				break;
			
            //
            if(byte == (uint8_t)(0x55))
                com_serial_receiveStatus = NEED_0XAA;
        }
        break;
        case NEED_0XAA :
        {
//            if(uart1_receiveValidBufferSize() < 1)
//                break;
//            uint8_t byte = 0;
//            uart1_readReceiveBuffer(&byte,1);
			
			uint8_t byte = 0 , r_byte;
			r_byte = rt_device_read(_console_device , 0 , &byte , 1);
			if(!r_byte)
				break;
			
            //
            if(byte == (uint8_t)(0xAA))
                com_serial_receiveStatus = NEED_LENGTH;
            else if(byte == (uint8_t)(0x55))
                break; // 连续两个0x55的情况。
            else
                com_serial_receiveStatus = FINDING_0X55;
        }
        break;
        case NEED_LENGTH :
        {
//            if(uart1_receiveValidBufferSize() < 1)
//                break;
//            uint8_t length = 0;
//            uart1_readReceiveBuffer(&length,1);
			
			uint8_t length = 0 , r_byte;
			r_byte = rt_device_read(_console_device , 0 , &length , 1);
			if(!r_byte)
				break;
			
            //
            if(length < 4 || length > RECEIVE_BUFFER_SIZE)
                break;
            //
            com_serial_receiveLength = length;
            com_serial_receiveStatus = GETTING_DATA;
        }
        break;
        case GETTING_DATA :
        {
            //log(TAG , "com_serial_checkEvent: GETTING_DATA");
//            if(uart1_receiveValidBufferSize() < com_serial_receiveLength)
//                break;
//            //
//            uart1_readReceiveBuffer(com_serial_receiveBuffer,com_serial_receiveLength);
			
			uint32_t len;
			rt_device_control(_console_device , RT_DEVICE_CTRL_GET_INT , &len);
			if(len < com_serial_receiveLength)
				break;
			rt_device_read(_console_device , 0 , com_serial_receiveBuffer , com_serial_receiveLength);
			
            if(math_crc16(0,com_serial_receiveBuffer,com_serial_receiveLength) == 0)
                engine_handlePacket_serial(com_serial_receiveBuffer,com_serial_receiveLength-2);
            //
            com_serial_receiveStatus = FINDING_0X55;
        }
        break;
    }
}



/*************************
函数名：com_serial_transmitBegin
功能：串口开始传输数据
参数：uint8_t packetType
返回值: 0
**************************/
int32_t com_serial_transmitBegin(uint8_t packetType)
{
    com_serial_transmitBuffer[0] = 0x55;
    com_serial_transmitBuffer[1] = 0xAA;
    com_serial_transmitBuffer[2] = 0;
    com_serial_transmitBuffer[3] = PROTOCOL_VERSION;
    com_serial_transmitBuffer[4] = packetType;
    com_serial_transmitLength = 5;
    //
    uint8_t tmp[] = {PROTOCOL_VERSION,packetType};
    com_serial_transmitCrc = math_crc16(0,tmp,sizeof(tmp));
    //
    return 0;
}




/**************************************
函数名：com_serial_transmitContent
功能：串口传输的内容
参数：const void * part,int32_t length
返回值: 0
***************************************/
int32_t com_serial_transmitContent(const void * part,int32_t length)
{
    //
    // 判断是否有足够空间。-2以保留CRC的位置。
    if(com_serial_transmitLength+length > TRANSMIT_BUFFER_SIZE-2)
        return 1;
    //
    // 复制数据。
    for(int32_t i=0;i<length;i++)
        com_serial_transmitBuffer[com_serial_transmitLength+i] = ((uint8_t *)part)[i];
    com_serial_transmitLength += length;
    //
    // 更新CRC。
    com_serial_transmitCrc = math_crc16(com_serial_transmitCrc,part,length);
    //
    return 0;
}



/**************************************
函数名：com_serial_transmitEnd
功能：串口传输结束
参数：无
返回值: 0
***************************************/
int32_t com_serial_transmitEnd(void)
{
    //
    // 把CRC放到包后面。
    com_serial_transmitBuffer[com_serial_transmitLength ++] = ((uint8_t *)&com_serial_transmitCrc)[1];
    com_serial_transmitBuffer[com_serial_transmitLength ++] = ((uint8_t *)&com_serial_transmitCrc)[0];
    //
    // 发送包。
    com_serial_transmitBuffer[2] = com_serial_transmitLength-(1+1+1);
    //uart1_transmit(com_serial_transmitBuffer,com_serial_transmitLength);
	rt_uint16_t old_flag = _console_device->open_flag;
	_console_device->open_flag |= RT_DEVICE_FLAG_STREAM;
	rt_device_write(_console_device, 0, com_serial_transmitBuffer, com_serial_transmitLength);
	_console_device->open_flag = old_flag;
    com_serial_transmitLength = 0;
    //
    return 0;
}

//void send_Attitude(void)
//{   
//    com_serial_transmitBegin(PROTOCOL_RETURN_ATTITUDE_QUATERNION);
//    com_serial_transmitContent(attitude_getAttitude(),4*4);
//    com_serial_transmitEnd();
//}
