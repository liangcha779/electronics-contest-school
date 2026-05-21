#include "stm32f10x.h"
#include "sys.h"
#include "oled.h"
#include "usart.h"
#include "tracking.h"

//  我是马千喜 傻逼


/* 巡线模块启动前的等待时间。
 *
 * 上电后先给整车一点稳定时间，避免：
 * 1. MPU6050 和 DMP 刚初始化完成时姿态还没完全稳定
 * 2. 八路巡线模块刚上电时串口数据还没准备好
 * 3. 小车刚通电就立刻进入巡线导致误动作
 */
#define TRACKING_START_DELAY_MS 10000

/* 主循环的节拍延时。
 *
 * 主控制闭环主要在外部中断里跑，
 * main() 里的 while 循环主要负责：
 * 1. 处理串口接收后的非中断逻辑
 * 2. 刷新 OLED 显示
 * 3. 定时打印调试信息
 *
 * 因此这里不需要很快，固定 5ms 足够。
 */
#define MAIN_LOOP_DELAY_MS 5

/* USART1 调试打印周期。
 *
 * 过快打印会占用串口时间，也会影响调试可读性；
 * 过慢则不利于观察实时状态。
 * 这里每 300ms 输出一次比较平衡。
 */
#define USART1_PRINT_PERIOD_MS 300

/* 姿态角。
 *
 * Roll  : 横滚角
 * Pitch : 俯仰角
 * Yaw   : 偏航角
 *
 * 这三个值由 MPU6050 + DMP 解算得到，
 * 其中平衡车最核心使用的是 Pitch。
 */
float Pitch,Roll,Yaw;

/* 陀螺仪原始角速度。
 *
 * gyroy 常用于直立环，
 * gyroz 常用于转向环，
 * gyrox 当前主要作为调试保留。
 */
short gyrox,gyroy,gyroz;

/* 加速度计原始数据。
 *
 * 当前主流程里没有直接参与闭环计算，
 * 主要保留给 MPU/DMP 和后续调试扩展使用。
 */
short aacx,aacy,aacz;

/* 左右编码器速度。
 *
 * 由定时器编码器接口读取，
 * 供速度环计算整车前后运动趋势。
 */
int Encoder_Left,Encoder_Right;

/* 电机 PWM 上下限。
 *
 * 用于底层限幅，防止输出超过驱动允许范围。
 */
int PWM_MAX=7200,PWM_MIN=-7200;

/* 左右电机最终装载值。 */
int MOTO1,MOTO2;

/* 三环输出调试量。
 *
 * 这几个变量在 control.c 中定义，
 * 这里用 extern 引用，方便必要时观察控制效果。
 */
extern int Vertical_out,Velocity_out,Turn_out;

int main(void)
{
	/* 用于主循环里的定时打印累计。 */
	u16 print_elapsed_ms = 0;

	/* 1. 基础时基与中断配置。 */
	delay_init();
	NVIC_Config();

	/* 2. 初始化串口。
	 * USART1：上位机调试、在线调参
	 * USART3：K230 视觉颜色识别
	 * USART2：八路巡线模块
	 */
	uart1_init(9600);
	uart3_init(9600);
	Tracking_Usart2_Init(115200);

	/* 3. 初始化 OLED，用于现场观察关键状态。 */
	OLED_Init();
	OLED_Clear();

	/* 4. 初始化姿态传感器与对应中断。 */
	MPU_Init();
	mpu_dmp_init();
	MPU6050_EXTI_Init();

	/* 5. 初始化编码器、电机和 PWM。 */
	Encoder_TIM2_Init();
	Encoder_TIM4_Init();
	Motor_Init();
	PWM_Init_TIM1(0,7199);

	/* 6. OLED 固定标签。
	 * 第一行显示角度
	 * 第二行显示巡线偏差
	 * 第三行显示速度估计值
	 */
	OLED_ShowString(0,1,"jiao du:",12);
	OLED_ShowString(0,2,"track:",12);
	OLED_ShowString(0,3,"sd:",12);

	/* 7. 等待系统稳定后，再通知巡线模块开始回传数字量数据。 */
	delay_ms(TRACKING_START_DELAY_MS);
	Tracking_SendControlData(0,1);

	while(1)
	{
		/* 8. 处理巡线串口接收缓存。
		 * USART2 中断只负责把字节塞进 FIFO，
		 * 真正的组包和解析放在主循环里做，减少中断负担。
		 */
		Tracking_ProcessRx();

		/* 9. 处理 USART1 的在线调参命令。 */
		USART1_ProcessCommand();

		/* 10. OLED 实时显示核心状态。 */
		OLED_Float(1,70,Pitch,1);
		OLED_Num3(8,2,Tracking_GetError());
		OLED_Num3(5,3,(int)((Encoder_Left+Encoder_Right)*2.38));

		/* 11. 周期性向上位机打印调试信息，便于串口观察：
		 * 1. 当前姿态角
		 * 2. 巡线偏差
		 * 3. 左右编码器速度
		 * 4. 八路数字量巡线状态
		 */
		if(print_elapsed_ms >= USART1_PRINT_PERIOD_MS)
		{
			print_elapsed_ms = 0;
			printf("Pitch=%.2f Roll=%.2f Yaw=%.2f\nTrackError=%d EncoderLeft=%d EncoderRight=%d SpeedDisplay=%.2f\nDigital=[%u,%u,%u,%u,%u,%u,%u,%u]\r\n",
			       Pitch,
			       Roll,
			       Yaw,
			       Tracking_GetError(),
			       Encoder_Left,
			       Encoder_Right,
			       (Encoder_Left + Encoder_Right) * 2.38f,
			       Tracking_IR_Data[0], Tracking_IR_Data[1], Tracking_IR_Data[2], Tracking_IR_Data[3],
			       Tracking_IR_Data[4], Tracking_IR_Data[5], Tracking_IR_Data[6], Tracking_IR_Data[7]);
		}

		/* 12. 主循环固定节拍。 */
		delay_ms(MAIN_LOOP_DELAY_MS);
		print_elapsed_ms += MAIN_LOOP_DELAY_MS;
	}
}
