#include "Usrt.h"

//By LJR
//在mian函数中写这个
//__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);       //打开IDLE中断
//  HAL_UART_Receive_DMA(&huart1,rx1_buffer,BUFFER_SIZE);    //开启DMA接受
//在stm32f1xx_it.c串口中断函数中使用
//记得在stm32f1xx_it.c中引用该头文件
//Usart1_IDLE();                        //IDIE接受
	
#if 1   
//  struct __FILE //该段在keil内不报错，vscode中报错，如不报错可去除注释
//  { 
//    int handle; 
//  }; 

//FILE __stdout;  //定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x) 
{ 
  x = x; 
} 
//重定向fputc函数S
int fputc(int ch,FILE *p)
{
	while(!(USART1->SR & (1<<7)));
	USART1->DR = ch;
	return ch;
}
#endif

//volatile保证共享内存使所有线程的工作都可以获得数据改变
volatile uint32_t rx1_len = 0;        //接收一帧数据的长度
volatile uint8_t rec1_end_flag = 0;   //一帧数据接收完成标志
uint8_t rx1_buffer[BUFFER_SIZE]={0};  //接收数据缓存数组

uint8_t rx1_buffer_lll[BUFFER_SIZE]="";  //接收数据缓存数组
uint32_t i;
uint8_t multi1,multi2;
uint8_t Subscript_Flag;
float Data_Buffer1,Data_Buffer2;

extern float Target;
extern char Mode;

void Usart1_IDLE(void)
{
  uint32_t tmp_flag = 0;
  uint32_t temp;
  tmp_flag = __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);         //获取IDIE标志位
  if((tmp_flag != RESET))                                         //判断是否进入空闲中断
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);                           //清除标志位
    HAL_UART_DMAStop(&huart1);                                    //停止DMA传输
    temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);                //获取未传输的数据个数
    rx1_len = BUFFER_SIZE - temp;
    rec1_end_flag = 1;
  }
}

void Usart1_Handle(void)                                         //对接受的一帧数据进行处理
{
  // if((rx1_buffer[0] == 'A') && (rx1_buffer[rx1_len-1] == 'S'))   //当串口发送器不发送新行时，有时串口2,3(不是串口1的时钟线)用这个判断
	if((rx1_buffer[0] == 'P') && (rx1_buffer[rx1_len-2] == 0x0D) && ((rx1_buffer[rx1_len-1] == 0x0A)))
  {
	  for(int p = rx1_len - 3;p > 0;p--)
	  {
		  rx1_buffer_lll[p-1] = rx1_buffer[p];
	  }
	  PID_Velocity.P = atof((const char *)(rx1_buffer_lll));
	  memset(rx1_buffer_lll,0,sizeof(rx1_buffer_lll));
	  printf("PID_Velocity.P = %.2f\r\n",PID_Velocity.P);
  }
  	if((rx1_buffer[0] == 'I') && (rx1_buffer[rx1_len-2] == 0x0D) && ((rx1_buffer[rx1_len-1] == 0x0A)))
  {
	  for(int p = rx1_len - 3;p > 0;p--)
	  {
		  rx1_buffer_lll[p-1] = rx1_buffer[p];
	  }
	  PID_Velocity.I = atof((const char *)(rx1_buffer_lll));
	  memset(rx1_buffer_lll,0,sizeof(rx1_buffer_lll));
	  printf("PID_Velocity.I = %.2f\r\n",PID_Velocity.I);
  }
  	if((rx1_buffer[0] == 'D') && (rx1_buffer[rx1_len-2] == 0x0D) && ((rx1_buffer[rx1_len-1] == 0x0A)))
  {
	  for(int p = rx1_len - 3;p > 0;p--)
	  {
		  rx1_buffer_lll[p-1] = rx1_buffer[p];
	  }
	  PID_Velocity.D = atof((const char *)(rx1_buffer_lll));
	  memset(rx1_buffer_lll,0,sizeof(rx1_buffer_lll));
	  printf("PID_Velocity.D = %.2f\r\n",PID_Velocity.D);
  }
  if((rx1_buffer[0] == 'T') && (rx1_buffer[rx1_len-2] == 0x0D) && ((rx1_buffer[rx1_len-1] == 0x0A)))
  {
	  for(int p = rx1_len - 3;p > 0;p--)
	  {
		  rx1_buffer_lll[p-1] = rx1_buffer[p];
	  }
	  Target = atof((const char *)(rx1_buffer_lll));
	  memset(rx1_buffer_lll,0,sizeof(rx1_buffer_lll));
	  printf("Target = %.2f\r\n",Target);
  }
  if((rx1_buffer[0] == 'M') && (rx1_buffer[rx1_len-2] == 0x0D) && ((rx1_buffer[rx1_len-1] == 0x0A)))
  {
		Data_Buffer1 = 0;
		multi1 = 1;
		for(i = 0;i < rx1_len;i++)                //取值函数
		  {
			  if((rx1_buffer[i] >= '0') && (rx1_buffer[i] <= '9'))
			  {
				  rx1_buffer[i] = rx1_buffer[i]-48;
			  }
		  }
		for(int p = rx1_len - 3;p > 0;p--)
		  {
			  Data_Buffer1 += multi1 * rx1_buffer[p];
			  multi1 *= 10;
		  }
		if(Data_Buffer1 <= 5)
		{
			Mode = Data_Buffer1;
			switch(Mode)
			{
				case 0:printf("Type_torque\r\n");
					break;
				case 1:printf("Type_velocity\r\n");
					break;
				case 2:printf("Type_angle\r\n");
					break;
				case 3:printf("Type_velocity_openloop\r\n");
					break;
				case 4:printf("Type_angle_openloop\r\n");
					break;
			}
			Controller = Mode;
		}
		else
			printf("ERROR\r\n");
  }
	  rx1_len = 0;                                                   //清除计数
	  rec1_end_flag = 0;                                             //清除结束标志位
	  HAL_UART_Receive_DMA(&huart1,rx1_buffer,BUFFER_SIZE);          //重新打开DMA接受
}

void DMA_Usart1_Send(uint8_t *buf,uint8_t len)                   //向串口发送
{
  if(HAL_UART_Transmit_DMA(&huart1,buf,len) != HAL_OK)           //如果异常进入异常中断
  {
    Error_Handler();
  }
}


