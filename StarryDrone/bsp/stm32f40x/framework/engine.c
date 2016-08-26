/*
 * File      : engine.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>

static enum
{
    ENGINE_SERIAL = 1,
    ENGINE_NRF
} engine_lastReceiveSource = ENGINE_SERIAL;

int32_t engine_init(void);
void    engine_checkEvent(void);
int32_t engine_handlePacket_serial(const uint8_t * packet,int32_t length);
int32_t engine_handlePacket_nrf(const uint8_t * packet,int32_t length);
int32_t engine_transmitBegin(uint8_t packetType);
int32_t engine_transmitContent(const void * part,int32_t length);
int32_t engine_transmitEnd(void);
int32_t engine_maxTransmitContentSize(void);
//
static int32_t engine_handlePacket(const uint8_t * packet,int32_t length);



/*************************
函数名：engine_init
功能：engine初始化
参数：无
返回值：0
**************************/
int32_t engine_init(void)      
{
    //com_serial_init();         //串口初始化
    //com_nrf24l01p_init();      //2410初始化
    //
    return 0;
}


/*************************
函数名：engine_checkEvent
功能：engine 事件检测
参数：无
返回值：0
**************************/
void engine_checkEvent(void)
{
    com_serial_checkEvent();    //串口事件检测
    com_nrf24l01p_checkEvent(); //2410事件检测
    com_encoder_checkEvent();   //编码器事件监测
}


/*************************
函数名：engine_transmitBegin
功能：engine 传输开始
      两种模式可供选择 ：
                        串口     ；  2410
参数：无
返回值：0
**************************/
int32_t engine_transmitBegin(uint8_t packetType)
{
    switch(engine_lastReceiveSource)
    {
        case ENGINE_SERIAL :
            return com_serial_transmitBegin(packetType);
        case ENGINE_NRF :
            return com_nrf24l01p_transmitBegin(packetType);
    }
    return 0;
}



/*************************
函数名：engine_transmitContent
功能：engine 传输内容
      两种模式可供选择 ：
                        串口     ；  2410
参数：无
返回值：0
**************************/
int32_t engine_transmitContent(const void * part,int32_t length)
{
    switch(engine_lastReceiveSource)
    {
        case ENGINE_SERIAL :
            return com_serial_transmitContent(part,length);
        case ENGINE_NRF :
            return com_nrf24l01p_transmitContent(part,length);
    }
    return 0;
}



/*************************
函数名：engine_transmitEnd
功能：engine 传输结束
      两种模式可供选择 ：
                        串口     ；  2410
参数：无
返回值：0
**************************/
int32_t engine_transmitEnd(void)
{
    switch(engine_lastReceiveSource)
    {
        case ENGINE_SERIAL :
            return com_serial_transmitEnd();
        case ENGINE_NRF :
            return com_nrf24l01p_transmitEnd();
    }
    return 0;
}



/*****************************************
函数名：engine_maxTransmitContentSize
功能：返回内容大小的最大值
参数：无
返回值：COM_SERIAL_CONTENT_SIZE_MAX   
        COM_NRF24L01P_CONTENT_SIZE_MAX
******************************************/
int32_t engine_maxTransmitContentSize(void)
{
    switch(engine_lastReceiveSource)
    {
        case ENGINE_SERIAL :
            return COM_SERIAL_CONTENT_SIZE_MAX;
        case ENGINE_NRF :
            return COM_NRF24L01P_CONTENT_SIZE_MAX;
    }
    return 0;
}

// packet:|version|type|content|。
int32_t engine_handlePacket_serial(const uint8_t * packet,int32_t length)
{
    engine_lastReceiveSource = ENGINE_SERIAL;
    return engine_handlePacket(packet,length);
}

// packet:|version|type|content|。
int32_t engine_handlePacket_nrf(const uint8_t * packet,int32_t length)
{
    engine_lastReceiveSource = ENGINE_NRF;      //可以看到无线和串口的数据包格式都是一样的，最低层都是同一个函数处理包
    
//    //收到数据
//    my_led_reverse();
    
    return engine_handlePacket(packet,length);
}






/*****************************************
函数名：engine_handlePacket
功能：处理引擎包数据
参数：const uint8_t * packet,int32_t length
返回值：0
******************************************/
// packet:|version|type|content|。
int32_t engine_handlePacket(const uint8_t * packet,int32_t length)
{
    if(packet[0] != PROTOCOL_VERSION)
        return 1;
    //led_reverse();
    //
    const uint8_t * param = packet + 2;
    int32_t param_length = length - 2;
    //Log(TAG , "engine_handlePacket: packet[1]=%x" , packet[1]);
    switch(packet[1])
    {
    case PROTOCOL_GET_ATTITUDE_QUATERNION :
        {
            if(param_length != 0)
                break;
            //
            cmd_getAttitude();
            break;
        }
    case PROTOCOL_GET_VECTOR :
        {
            if(param_length != 1)
                break;
            //
            cmd_getVector(param[0]);
            break;
        }
    case PROTOCOL_SET_CONTROL_MODE :
        {
            if(param_length != 1)
                break;
            //
            cmd_setControlMode(param[0]);
            break;
        }
    case PROTOCOL_LOCK_THROTTLE_SET_THROTTLE :
        {
            if(param_length != 4*4)
                break;
            //
            float th[4];
            for(int i=0;i<4*4;i++)
                ((uint8_t *)th)[i] = param[i];
            //
            cmd_lockThrottle_setThrottle(th);
            //
            break;
        }
    case PROTOCOL_GET_STATUS :
        {
            if(param_length < 1)
                break;
            //
            cmd_getStatus(param,param_length);
            break;
        }
    case PROTOCOL_BOOTLOADER_CMD :
        {
            if(param_length < 1)
                break;
            cmd_bootloaderCmd(param[0],param+1,param_length);
        }
        break;
    case PROTOCOL_PARAMETER :
        cmd_parameter(param,param_length);
        break;
    case PROTOCOL_LOCK_ATTITUDE:
        cmd_lockAttitude(param,param_length);
        break;
    }
    return 0;
}

