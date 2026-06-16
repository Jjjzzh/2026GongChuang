#include "Emm_V5.h"
#include "community.h"

/* Emm_V5 串口收发 (USART6), 中断接收 + 帧解析 */
extern CommPort_t emm_port;
#define EMM_SEND(cmd, len)  CommPort_SendBytes(&emm_port, (uint8_t *)(cmd), (len))

/* ============================================================
 * USART6 中断接收 — 帧解析
 * 响应帧格式: [addr][cmd][data...][0x6B]
 * ============================================================ */
#define EMM_RX_BUF_SIZE   16
static uint8_t  emm_rx_buf[EMM_RX_BUF_SIZE];
static uint8_t  emm_rx_idx;
static uint8_t  emm_rx_byte;          /* 单字节中断接收 */
static uint8_t  emm_rx_started = 0;

/* 最近一次电机状态 */
static uint8_t  emm_motor_flag   = 0;   /* S_FLAG 电机状态标志 */
static uint8_t  emm_driver_flag  = 0;   /* S_OFLAG 驱动状态标志 */
static uint32_t emm_last_pos     = 0;   /* S_CPOS 实时位置 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6) return;

    if (emm_rx_idx == 0) {
        /* 帧首 = 电机地址 */
        emm_rx_buf[0] = emm_rx_byte;
        emm_rx_idx = 1;
    } else {
        emm_rx_buf[emm_rx_idx] = emm_rx_byte;
        emm_rx_idx++;

        /* 收到校验字节 0x6B 表示帧结束 */
        if (emm_rx_byte == 0x6B && emm_rx_idx >= 3) {
            /* 解析帧: [addr][cmd][data...][0x6B] */
            uint8_t cmd = emm_rx_buf[1];
            switch (cmd) {
            case 0x3A:  /* S_FLAG: 电机状态标志 */
                emm_motor_flag = emm_rx_buf[2];
                break;
            case 0x3B:  /* S_OFLAG: 驱动状态标志 */
                emm_driver_flag = emm_rx_buf[2];
                break;
            case 0x3C:  /* S_OAF: 电机+驱动标志 */
                emm_motor_flag  = emm_rx_buf[2];
                emm_driver_flag = emm_rx_buf[3];
                break;
            case 0x36:  /* S_CPOS: 实时位置 (32bit) */
                emm_last_pos = ((uint32_t)emm_rx_buf[2] << 24)
                             | ((uint32_t)emm_rx_buf[3] << 16)
                             | ((uint32_t)emm_rx_buf[4] << 8)
                             |  (uint32_t)emm_rx_buf[5];
                break;
            }
            emm_rx_idx = 0;  /* 准备下一帧 */
        }

        /* 防止越界 */
        if (emm_rx_idx >= EMM_RX_BUF_SIZE) {
            emm_rx_idx = 0;
        }
    }

    /* 重新使能中断接收下一个字节 */
    HAL_UART_Receive_IT(&huart6, &emm_rx_byte, 1);
}

/**
 * @brief  初始化电机串口接收
 * @note   在 CommPort_Init 之后调用
 */
void Emm_V5_RX_Init(void)
{
    emm_rx_idx = 0;
    emm_rx_started = 1;
    HAL_UART_Receive_IT(&huart6, &emm_rx_byte, 1);
}

/* ============================================================
 * 状态查询 API
 * ============================================================ */

/**
 * @brief  获取电机状态标志 (S_FLAG)
 * @return 标志字节, bit2=回零中, bit3=到位
 *         bit4=堵转, bit6=回零完成, bit7=错误
 */
uint8_t Emm_V5_GetMotorFlag(void)
{
    return emm_motor_flag;
}

/**
 * @brief  获取驱动状态标志 (S_OFLAG)
 */
uint8_t Emm_V5_GetDriverFlag(void)
{
    return emm_driver_flag;
}

/**
 * @brief  查询回零是否完成
 * @return 1=回零完成/到位, 0=仍在回零中
 * @note   通过读取 S_FLAG 判断, 轮询间隔 ≥ 50ms
 */
uint8_t Emm_V5_IsHomed(void)
{
    /* 回零完成后电机进入到位状态, 且不在回零运动中 */
    if (emm_motor_flag & 0x40) return 1;   /* bit6: 回零完成   */
    if (emm_motor_flag & 0x08) return 1;   /* bit3: 到位      */
    return 0;
}

/**
 * @brief  查询电机是否在运动中
 * @return 1=运动中, 0=停止
 */
uint8_t Emm_V5_IsMoving(void)
{
    return (emm_motor_flag & 0x04) ? 1 : 0;  /* bit2: 运动中/回零中 */
}

/**
 * @brief  获取电机实时位置 (S_CPOS 返回值)
 */
uint32_t Emm_V5_GetPosition(void)
{
    return emm_last_pos;
}

/**********************************************************
***	Emm_V5.0 闭环步进驱动器
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝地址：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

__IO uint16_t MMCL_count = 0, MMCL_cmd[MMCL_LEN] = {0};

/**********************************************************
*** 系统控制函数
**********************************************************/
/**
  * @brief    触发编码器校准
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Trig_Encoder_Cal(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x06;                       // 命令字
  cmd[2] =  0x45;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
	EMM_SEND(cmd, 4);
}

/**
  * @brief    重启电机（Y42）
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Reset_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x08;                       // 命令字
  cmd[2] =  0x97;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
	EMM_SEND(cmd, 4);
}

/**
  * @brief    当前位置清零
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 命令字
  cmd[2] =  0x6D;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
	EMM_SEND(cmd, 4);
}

/**
  * @brief    清除堵转保护
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 命令字
  cmd[2] =  0x52;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 4);
}

/**
  * @brief    恢复出厂设置
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Restore_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0F;                       // 命令字
  cmd[2] =  0x5F;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
	EMM_SEND(cmd, 4);
}

/**********************************************************
*** 运动控制函数
**********************************************************/
/**
  * @brief    组合指令（Y42）
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Multi_Motor_Cmd(uint8_t addr)
{
  uint16_t i = 0, j = 0, len = 0; __IO static uint8_t cmd[MMCL_LEN] = {0};

	// 组合指令长度大于0
	if(MMCL_count > 0)
	{
		// 计算命令总字节数
		len = MMCL_count + 5;

		// 装载命令
		cmd[0] = addr;                       // 地址
		cmd[1] = 0xAA;                       // 命令字
		cmd[2] = (uint8_t)(len >> 8);				 // 长度高8位
		cmd[3] = (uint8_t)(len); 		 				 // 长度低8位
		for(i=0,j=4; i < MMCL_count; i++,j++) { cmd[j] = MMCL_cmd[i]; }
		cmd[j] = 0x6B; ++j;                  // 校验字节

		// 发送命令
		EMM_SEND(cmd, j); MMCL_count = 0;
	}
	else
	{
		MMCL_count = 0;
	}
}

/**
  * @brief    使能信号控制
  * @param    addr  电机地址
  * @param    state 使能状态     true为使能电机，false为关闭电机
  * @param    snF   同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 命令字
  cmd[2] =  0xAB;                       // 命令字
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 同步运动标志
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr 电机地址
  * @param    dir  方向         0为CW，其他值为CCW
  * @param    vel  速度(RPM)    范围0 - 5000RPM
  * @param    acc  加速度       范围0 - 255，注意：0直接启动
  * @param    snF  同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 命令字
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0直接启动
  cmd[6] =  snF;                        // 同步运动标志
  cmd[7] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr 电机地址
  * @param    dir  方向          0为CW，其他值为CCW
  * @param    vel  速度(RPM)     范围0 - 5000RPM
  * @param    acc  加速度        范围0 - 255，注意：0直接启动
  * @param    clk  脉冲数        范围0- (2^32 - 1)
  * @param    raF  相对/绝对标志 false为相对运动，true为绝对值运动
  * @param    snF  同步运动标志  false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 命令字
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
  cmd[5]  =  acc;                       // 加速度，注意：0直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相对/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 同步运动标志，false为不使用，true为使用
  cmd[12] =  0x6B;                      // 校验字节

  // 发送命令
  EMM_SEND(cmd, 13);
}

/**
  * @brief    立即停止
  * @param    addr  电机地址
  * @param    snF   同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 命令字
  cmd[2] =  0x98;                       // 命令字
  cmd[3] =  snF;                        // 同步运动标志
  cmd[4] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 5);
}

/**
  * @brief    触发同步运动
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 命令字
  cmd[2] =  0x66;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 4);
}

/**********************************************************
*** 原点控制函数
**********************************************************/
/**
  * @brief    设置线圈当前位置为原点
  * @param    addr  电机地址
  * @param    svF   是否存储标志 false为不存储，true为存储
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 命令字
  cmd[2] =  0x88;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 5);
}

/**
  * @brief    触发回零
  * @param    addr   电机地址
  * @param    o_mode 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  * @param    snF   同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 命令字
  cmd[2] =  o_mode;                     // 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  cmd[3] =  snF;                        // 同步运动标志，false为不使用，true为使用
  cmd[4] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 命令字
  cmd[2] =  0x48;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 4);
}

/**
  * @brief    读取回零参数
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Origin_Read_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x22;                       // 命令字
  cmd[2] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 3);
}

/**
  * @brief    修改回零参数
  * @param    addr   电机地址
  * @param    svF    是否存储标志 false为不存储，true为存储
  * @param    o_mode 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  * @param    o_dir  回零方向，0为CW，其他值为CCW
  * @param    o_vel  回零速度，单位RPM（转/分钟）
  * @param    o_tm   回零超时时间，单位毫秒
  * @param    sl_vel 回零无需限位碰撞回零转速，单位RPM（转/分钟）
  * @param    sl_ma  回零无需限位碰撞回零电流，单位Ma（毫安）
  * @param    sl_ms  回零无需限位碰撞回零时间，单位Ms（毫秒）
  * @param    potF   上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  __IO static uint8_t cmd[32] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 命令字
  cmd[2] =  0xAE;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 回零无需限位碰撞回零转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 回零无需限位碰撞回零转速(RPM)低8位字节
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 回零无需限位碰撞回零电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 回零无需限位碰撞回零电流(Ma)低8位字节
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 回零无需限位碰撞回零时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 回零无需限位碰撞回零时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节

  // 发送命令
  EMM_SEND(cmd, 20);
}

/**********************************************************
*** 获取系统参数函数
**********************************************************/
/**
  * @brief    定时自动返回信息指令（Y42）
  * @param    addr  	 电机地址
  * @param    s     	 系统参数类型
	* @param    time_ms  定时时间
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  cmd[i] = 0x11; ++i;                   // 命令字

  cmd[i] = 0x18; ++i;                   // 命令字

  switch(s)                             // 信息类型
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	// 获取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	// 获取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	// 获取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	// 获取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	// 获取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	// 获取经过线性校准后的编码器值
		case S_CLKI : cmd[i] = 0x32; ++i; break;	// 获取输入脉冲数
    case S_TPOS : cmd[i] = 0x33; ++i; break;	// 获取电机目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	// 获取电机实时设定的目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	// 获取电机实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	// 获取电机实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	// 获取电机位置误差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	// 获取线圈功率级电源电压（Y42）
		case S_TEMP : cmd[i] = 0x39; ++i; break;	// 获取电机实时温度（Y42）
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	// 获取电机状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// 获取驱动状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	// 获取电机状态标志位 + 驱动状态标志位（Y42）
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	// 获取端口状态（Y42）
    default: break;
  }

	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;	// 定时时间
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

  cmd[i] = 0x6B; ++i;                   	// 校验字节

  // 发送命令
  EMM_SEND(cmd, i);
}

/**
  * @brief    读取系统参数
  * @param    addr  电机地址
  * @param    s     系统参数类型
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 命令字
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	// 获取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	// 获取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	// 获取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	// 获取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	// 获取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	// 获取经过线性校准后的编码器值
		case S_CLKI : cmd[i] = 0x32; ++i; break;	// 获取输入脉冲数
    case S_TPOS : cmd[i] = 0x33; ++i; break;	// 获取电机目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	// 获取电机实时设定的目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	// 获取电机实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	// 获取电机实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	// 获取电机位置误差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	// 获取线圈功率级电源电压（Y42）
		case S_TEMP : cmd[i] = 0x39; ++i; break;	// 获取电机实时温度（Y42）
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	// 获取电机状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// 获取驱动状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	// 获取电机状态标志位 + 驱动状态标志位（Y42）
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	// 获取端口状态（Y42）
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节

  // 发送命令
  EMM_SEND(cmd, i);
}

/**********************************************************
*** 读写配置参数函数
**********************************************************/
/**
  * @brief    修改电机ID地址
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    id			 默认电机ID为1，可修改为1-255，0为广播地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xAE;                       // 命令字
  cmd[2] =  0x4B;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  id;                  				// 默认电机ID为1，可修改为1-255，0为广播地址
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改细分值
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    mstep		 默认细分为16，可修改为1-255，0为256细分
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x84;                       // 命令字
  cmd[2] =  0x8A;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  mstep;                 	 		// 默认细分为16，可修改为1-255，0为256细分
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改PD标志
  * @param    addr     电机地址
  * @param    pdf		 	 PD标志
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x50;                       // 命令字
  cmd[2] =  pdf;                 	 			// PD标志
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 4);
}

/**
  * @brief    读取选项参数状态（Y42）
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x1A;                       // 命令字
  cmd[2] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 3);
}

/**
  * @brief    修改电机类型（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    mottype	 电机类型，默认为0，0表示1.8度步进电机，1表示0.9度步进电机
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype)
{
  __IO static uint8_t cmd[16] = {0}; uint8_t MotType = 0;

	if(mottype) { MotType = 25; } else { MotType = 50; }

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xD7;                       // 命令字
  cmd[2] =  0x35;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  MotType;                 	 	// 电机类型，0表示0.9度步进电机，1表示1.8度步进电机
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改固件类型（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    fwtype	  固件类型，默认为0，0为X固件，1为Emm固件
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xD5;                       // 命令字
  cmd[2] =  0x69;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  fwtype;                 	 	// 固件类型，25表示0.9度步进电机，50表示1.8度步进电机
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改开环/闭环控制模式（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    ctrl_mode 控制模式，默认为1，0为开环模式，1为闭环FOC模式
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 命令字
  cmd[2] =  0x69;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式，默认为1，0为开环模式，1为闭环FOC模式
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改电机运动方向（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    dir			 电机运动方向，默认为CW，0为CW（顺时针方向），1为CCW
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xD4;                       // 命令字
  cmd[2] =  0x60;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  dir;                  			// 电机运动方向，默认为CW，0为CW（顺时针方向），1为CCW
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改按键锁定功能（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    lock		 按键锁定功能，默认为Disable，0为Disable，1为Enable
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xD0;                       // 命令字
  cmd[2] =  0xB3;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  lock;                  			// 按键锁定功能，默认为Disable，0为Disable，1为Enable
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改显示速度值是否缩小10倍显示（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    s_vel		 显示速度值是否缩小10倍显示，默认为Disable，0为Disable，1为Enable
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4F;                       // 命令字
  cmd[2] =  0x71;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  s_vel;                  		// 显示速度值是否缩小10倍显示，默认为Disable，0为Disable，1为Enable
  cmd[5] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 6);
}

/**
  * @brief    修改开环模式输出电流
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    om_ma 	 开环模式输出电流，单位mA
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x44;                       // 命令字
  cmd[2] =  0x33;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  (uint8_t)(om_ma >> 8);			// 开环模式输出电流，单位mA
	cmd[5] =  (uint8_t)(om_ma >> 0);
  cmd[6] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 7);
}

/**
  * @brief    修改闭环模式输出电流
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    foc_mA 	 闭环模式输出电流，单位mA
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x45;                       // 命令字
  cmd[2] =  0x66;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  (uint8_t)(foc_mA >> 8);			// 闭环模式输出电流，单位mA
	cmd[5] =  (uint8_t)(foc_mA >> 0);
  cmd[6] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 7);
}

/**
  * @brief    读取PID参数
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_PID_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x21;                       // 命令字
  cmd[2] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 3);
}

/**
  * @brief    修改PID参数
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    kp 	 		 比例系数，默认为Y42/18000
	* @param    ki 	 		 积分系数，默认为Y42/10
	* @param    kd 	 		 微分系数，默认为Y42/18000
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd)
{
  __IO static uint8_t cmd[20] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0x4A;                      // 命令字
  cmd[2]  =  0xC3;                      // 命令字
  cmd[3]  =  svF;                       // 是否存储标志，false为不存储，true为存储
  cmd[4]  =  (uint8_t)(kp >> 24);				// kp
	cmd[5]  =  (uint8_t)(kp >> 16);
	cmd[6]  =  (uint8_t)(kp >> 8);
	cmd[7]  =  (uint8_t)(kp >> 0);
	cmd[8]  =  (uint8_t)(ki >> 24);				// ki
	cmd[9]  =  (uint8_t)(ki >> 16);
	cmd[10] =  (uint8_t)(ki >> 8);
	cmd[11] =  (uint8_t)(ki >> 0);
	cmd[12] =  (uint8_t)(kd >> 24);				// kd
	cmd[13] =  (uint8_t)(kd >> 16);
	cmd[14] =  (uint8_t)(kd >> 8);
	cmd[15] =  (uint8_t)(kd >> 0);
  cmd[16] =  0x6B;                      // 校验字节

  // 发送命令
  EMM_SEND(cmd, 17);
}

/**
  * @brief    读取DMX512协议参数（Y42）
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_DMX512_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x49;                       // 命令字
	cmd[2] =  0x78;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 4);
}

/**
  * @brief    修改DMX512协议参数（Y42）
  * @param    addr  		电机地址
  * @param    svF   		是否存储标志 false为不存储，true为存储
  * @param    tch				起始通道号，默认为192，该值要小于 DMX512 灯具的通道号一个
	* @param    nch				每个灯占用的通道数，默认为1，1为单通道模式，2为双通道模式
	* @param    mode			运动模式，默认为1，0表示往返位置模式运动，1表示往复绝对位置运动
	* @param    vel				单通道模式运动速度，默认值为1000， 单位RPM， 即1000RPM
	* @param    acc				加速度，acc=设定值/8=125，使用时请参考说明书"5.3.12 位置模式控制（Emm）
	* @param    vel_step	双通道模式速度参数，默认值为 10， 则实际运动速度为(通道值 * 10)RPM
	* @param    pos_step	双通道模式运动步长，默认值为 100， 则电机转动角度为(通道值 * 10.0)度
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step)
{
  __IO static uint8_t cmd[32] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xD9;                      // 命令字
  cmd[2]  =  0x90;                      // 命令字
  cmd[3]  =  svF;                       // 是否存储标志，false为不存储，true为存储
  cmd[4]  =  (uint8_t)(tch >> 8);     	// 起始通道号
  cmd[5]  =  (uint8_t)(tch >> 0);
	cmd[6]  =  nch;                       // 每个灯占用的通道数
	cmd[7]  =  mode;                      // 运动模式
	cmd[8]  =  (uint8_t)(vel >> 8);     	// 单通道模式运动速度
  cmd[9]  =  (uint8_t)(vel >> 0);
	cmd[10] =  (uint8_t)(acc >> 8);     	// 双通道模式速度参数
  cmd[11] =  (uint8_t)(acc >> 0);
	cmd[12] =  (uint8_t)(vel_step >> 8);  // 双通道模式速度参数
  cmd[13] =  (uint8_t)(vel_step >> 0);
  cmd[14]  = (uint8_t)(pos_step >> 24);	// 双通道模式运动步长
  cmd[15]  = (uint8_t)(pos_step >> 16);
  cmd[16] =  (uint8_t)(pos_step >> 8);
  cmd[17] =  (uint8_t)(pos_step >> 0);
  cmd[18] =  0x6B;                      // 校验字节

  // 发送命令
  EMM_SEND(cmd, 19);
}

/**
  * @brief    读取位置到达窗口（Y42）
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_Pos_Window(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x41;                       // 命令字
  cmd[2] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 3);
}

/**
  * @brief    修改位置到达窗口（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    prw 	 	 位置到达窗口，默认值为8，即0.8度
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xD1;                       // 命令字
  cmd[2] =  0x07;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  (uint8_t)(prw >> 8);				// 位置到达窗口，默认值为8，即0.8度
	cmd[5] =  (uint8_t)(prw >> 0);
  cmd[6] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 7);
}

/**
  * @brief    读取过热过流保护设定值（Y42）
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_Otocp(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x13;                       // 命令字
  cmd[2] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 3);
}

/**
  * @brief    修改过热过流保护设定值（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    otp 	 	 过热保护温度值，默认100
	* @param    ocp 	 	 过流保护电流值，默认6600mA
	* @param    time_ms   过热过流检测时间，默认1000ms
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xD3;                      // 命令字
  cmd[2]  =  0x56;                      // 命令字
  cmd[3]  =  svF;                       // 是否存储标志，false为不存储，true为存储
  cmd[4]  =  (uint8_t)(otp >> 8);				// 过热保护温度值
	cmd[5]  =  (uint8_t)(otp >> 0);
	cmd[6]  =  (uint8_t)(ocp >> 8);				// 过流保护电流值
	cmd[7]  =  (uint8_t)(ocp >> 0);
	cmd[8]  =  (uint8_t)(time_ms >> 8);		// 过热过流检测时间
	cmd[9]  =  (uint8_t)(time_ms >> 0);
  cmd[10] =  0x6B;                      // 校验字节

  // 发送命令
  EMM_SEND(cmd, 11);
}

/**
  * @brief    读取心跳保护间隔时间（Y42）
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_Heart_Protect(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x16;                       // 命令字
  cmd[2] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 3);
}

/**
  * @brief    修改心跳保护间隔时间（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    hp 	 	 	 心跳保护时间，单位ms
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0x68;                      // 命令字
  cmd[2]  =  0x38;                      // 命令字
  cmd[3]  =  svF;                       // 是否存储标志，false为不存储，true为存储
  cmd[4]  =  (uint8_t)(hp >> 24);				// 心跳保护时间，单位ms
	cmd[5]  =  (uint8_t)(hp >> 16);
	cmd[6]  =  (uint8_t)(hp >> 8);
	cmd[7]  =  (uint8_t)(hp >> 0);
  cmd[8]  =  0x6B;                      // 校验字节

  // 发送命令
  EMM_SEND(cmd, 9);
}

/**
  * @brief    读取积分限幅/积分分离系数（Y42）
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_Integral_Limit(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x23;                       // 命令字
  cmd[2] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 3);
}

/**
  * @brief    修改积分限幅/积分分离系数（Y42）
  * @param    addr     电机地址
  * @param    svF      是否存储标志 false为不存储，true为存储
  * @param    il 	 	 	 积分限幅值，默认值为65535
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0x4B;                      // 命令字
  cmd[2]  =  0x57;                      // 命令字
  cmd[3]  =  svF;                       // 是否存储标志，false为不存储，true为存储
  cmd[4]  =  (uint8_t)(il >> 24);				// 积分限幅值，默认值为65535
	cmd[5]  =  (uint8_t)(il >> 16);
	cmd[6]  =  (uint8_t)(il >> 8);
	cmd[7]  =  (uint8_t)(il >> 0);
  cmd[8]  =  0x6B;                      // 校验字节

  // 发送命令
  EMM_SEND(cmd, 9);
}

/**********************************************************
*** 读取配置与状态参数
**********************************************************/
/**
  * @brief    读取系统状态参数
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_System_State_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x43;                       // 命令字
	cmd[2] =  0x7A;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 4);
}

/**
  * @brief    读取电机配置参数
  * @param    addr     电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x42;                       // 命令字
	cmd[2] =  0x6C;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令
  EMM_SEND(cmd, 4);
}



/**
***********************************************************
***********************************************************
***
***
*** @brief	下面是队列缓冲相关的Y42组合指令函数（Y42）
***
***
***********************************************************
***********************************************************
***/
/**********************************************************
*** 系统控制函数
**********************************************************/
/**
  * @brief    触发编码器校准 - 合并到指令队列
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x06;                       // 命令字
  cmd[2] =  0x45;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    重启电机（Y42） - 合并到指令队列
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Reset_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x08;                       // 命令字
  cmd[2] =  0x97;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    当前位置清零 - 合并到指令队列
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 命令字
  cmd[2] =  0x6D;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    清除堵转保护 - 合并到指令队列
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 命令字
  cmd[2] =  0x52;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    恢复出厂设置 - 合并到指令队列
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Restore_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0F;                       // 命令字
  cmd[2] =  0x5F;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**********************************************************
*** 运动控制函数
**********************************************************/
/**
  * @brief    使能信号控制 - 合并到指令队列
  * @param    addr  电机地址
  * @param    state 使能状态     true为使能电机，false为关闭电机
  * @param    snF   同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 命令字
  cmd[2] =  0xAB;                       // 命令字
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 同步运动标志
  cmd[5] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 6; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    速度模式 - 合并到指令队列
  * @param    addr 电机地址
  * @param    dir  方向         0为CW，其他值为CCW
  * @param    vel  速度(RPM)    范围0 - 5000RPM
  * @param    acc  加速度       范围0 - 255，注意：0直接启动
  * @param    snF  同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 命令字
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0直接启动
  cmd[6] =  snF;                        // 同步运动标志
  cmd[7] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 8; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    位置模式 - 合并到指令队列
  * @param    addr 电机地址
  * @param    dir  方向          0为CW，其他值为CCW
  * @param    vel  速度(RPM)     范围0 - 5000RPM
  * @param    acc  加速度        范围0 - 255，注意：0直接启动
  * @param    clk  脉冲数        范围0- (2^32 - 1)
  * @param    raF  相对/绝对标志 false为相对运动，true为绝对值运动
  * @param    snF  同步运动标志  false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 命令字
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
  cmd[5]  =  acc;                       // 加速度，注意：0直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相对/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 同步运动标志，false为不使用，true为使用
  cmd[12] =  0x6B;                      // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 13; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    立即停止 - 合并到指令队列
  * @param    addr  电机地址
  * @param    snF   同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 命令字
  cmd[2] =  0x98;                       // 命令字
  cmd[3] =  snF;                        // 同步运动标志
  cmd[4] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    触发同步运动 - 合并到指令队列
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Synchronous_motion(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 命令字
  cmd[2] =  0x66;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**********************************************************
*** 原点控制函数
**********************************************************/
/**
  * @brief    设置线圈当前位置为原点 - 合并到指令队列
  * @param    addr  电机地址
  * @param    svF   是否存储标志 false为不存储，true为存储
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 命令字
  cmd[2] =  0x88;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    触发回零 - 合并到指令队列
  * @param    addr   电机地址
  * @param    o_mode 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  * @param    snF   同步运动标志 false为不使用，true为使用
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 命令字
  cmd[2] =  o_mode;                     // 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  cmd[3] =  snF;                        // 同步运动标志，false为不使用，true为使用
  cmd[4] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    强制中断并退出回零 - 合并到指令队列
  * @param    addr  电机地址
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 命令字
  cmd[2] =  0x48;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    修改回零参数 - 合并到指令队列
  * @param    addr   电机地址
  * @param    svF    是否存储标志 false为不存储，true为存储
  * @param    o_mode 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  * @param    o_dir  回零方向，0为CW，其他值为CCW
  * @param    o_vel  回零速度，单位RPM（转/分钟）
  * @param    o_tm   回零超时时间，单位毫秒
  * @param    sl_vel 回零无需限位碰撞回零转速，单位RPM（转/分钟）
  * @param    sl_ma  回零无需限位碰撞回零电流，单位Ma（毫安）
  * @param    sl_ms  回零无需限位碰撞回零时间，单位Ms（毫秒）
  * @param    potF   上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t j = 0, cmd[32] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 命令字
  cmd[2] =  0xAE;                       // 命令字
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为线圈近点回零，1为线圈限位回零，2为线圈无需限位碰撞回零，3为线圈无需限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 回零无需限位碰撞回零转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 回零无需限位碰撞回零转速(RPM)低8位字节
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 回零无需限位碰撞回零电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 回零无需限位碰撞回零电流(Ma)低8位字节
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 回零无需限位碰撞回零时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 回零无需限位碰撞回零时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < 20; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**********************************************************
*** 获取系统参数函数
**********************************************************/
/**********************************************************
*** 获取系统参数函数
**********************************************************/
/**
  * @brief    定时自动返回信息指令（Y42）
  * @param    addr  	 电机地址
  * @param    s     	 系统参数类型
	* @param    time_ms  定时时间
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};

  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  cmd[i] = 0x11; ++i;                   // 命令字

  cmd[i] = 0x18; ++i;                   // 命令字

  switch(s)                             // 信息类型
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	// 获取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	// 获取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	// 获取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	// 获取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	// 获取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	// 获取经过线性校准后的编码器值
		case S_CLKI : cmd[i] = 0x32; ++i; break;	// 获取输入脉冲数
    case S_TPOS : cmd[i] = 0x33; ++i; break;	// 获取电机目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	// 获取电机实时设定的目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	// 获取电机实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	// 获取电机实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	// 获取电机位置误差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	// 获取线圈功率级电源电压（Y42）
		case S_TEMP : cmd[i] = 0x39; ++i; break;	// 获取电机实时温度（Y42）
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	// 获取电机状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// 获取驱动状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	// 获取电机状态标志位 + 驱动状态标志位（Y42）
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	// 获取端口状态（Y42）
    default: break;
  }

	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;	// 定时时间
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

  cmd[i] = 0x6B; ++i;                   	// 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
  * @brief    读取系统参数 - 合并到指令队列
  * @param    addr  电机地址
  * @param    s     系统参数类型
  * @retval   地址 + 命令字 + 运动状态 + 校验字节
  */
void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};

  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 命令字
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	// 获取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	// 获取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	// 获取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	// 获取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	// 获取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	// 获取经过线性校准后的编码器值
		case S_CLKI : cmd[i] = 0x32; ++i; break;	// 获取输入脉冲数
    case S_TPOS : cmd[i] = 0x33; ++i; break;	// 获取电机目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	// 获取电机实时设定的目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	// 获取电机实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	// 获取电机实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	// 获取电机位置误差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	// 获取线圈功率级电源电压（Y42）
		case S_TEMP : cmd[i] = 0x39; ++i; break;	// 获取电机实时温度（Y42）
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	// 获取电机状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// 获取驱动状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	// 获取电机状态标志位 + 驱动状态标志位（Y42）
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	// 获取端口状态（Y42）
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节

  // 保存当前命令到指令队列
  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**********************************************************
*** 读写配置参数函数
**********************************************************/
