/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rx_buf1=0;                           //串口通讯数据储存
uint8_t rx_buf2[10] = {0};
uint8_t rx_buf3[5]="aaaa";
float capture_buf3=0,capture_buf4=0;        //pid计算时间控制参数
int capture_flag3=0,capture_flag4=0;
# define paraspeed 330000                   //速度计算参数
#define  targestangz 80
int startflag=0;
//小车姿态结构体
struct gesture
{
	float angz;			//车头偏差角度
//	float angx;
//	float angy;
	float w;
	float x3;
	float x4;
	float v3;            //right speed
	float v4;            //left speed
	float x;
	float pwm_angz;			 //小车角度PID输出
  float pwm_v3;
  float pwm_v4;        //PID的输出
};

struct pidstruct
{
	float kp,ki,kd;
	float p,i,d;
	float thisde,lastde,beforede;
	float integralde;
};

struct gesture curgest;   //当前姿态
struct gesture targest;   //目标姿态
//struct gesture inigest;
//struct pidstruct angxpid;
//struct pidstruct angypid;
struct pidstruct angzpid;  //角度PID配置  
struct pidstruct x3pid;
struct pidstruct x4pid;
struct pidstruct v3pid;
struct pidstruct v4pid;   //速度PID配置
float pidK;										//pid的系数（）方便调试
float inittarspeed;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void SystemClock_Config(void);
void Task_Delay(int time);                              //任务定时器
void Wheel(int num,int pwm);                                            //选择轮子并设定pwm控速
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);//Input capture NVIC   //测速使用的中断函数
float PID(struct pidstruct *e,float err,float outlow,float outhigh);    //位置式PID函数
float Incremental_PID(struct pidstruct *e,float err, float MaxOutput,float MinOutput) ;     //增量式PID
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch,FILE *f)
//fputc是c语言中printf的底层函数，为了使其能在串口中使用，需要重新定义一下这个函数
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}
//接收字符串转换成数字
int transfer_int(uint8_t *data)
{
	int transfer=0;
	for(int i=0;i<5;i++)
	{
		
	 if(data[i]<='9'&&data[i]>='0')
	 {
     transfer+=(data[i]-48);
		 if(data[i+1]<='9'&&data[i+1]>='0')
		 {
			 transfer*=10;
		 }
		 else
		 {
			 break;
		 }
	 }
  }
	return transfer;
}
void Wheel(int num,int pwm)      //通过PWM信号直接控制电机转速
{
	int direct;
	if(pwm>1000) pwm=1000;
	if(pwm<-1000) pwm=-1000;
	
	if(pwm>=0)  direct=1;
	if(pwm<0)   direct=0;
	
	switch(num){
		case 3:
			switch(direct){
				case 0:
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,1000+pwm);//w3_pwm_out
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);//w3_gpio_out
					break;
				case 1:
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm);//w3_pwm_out
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);//w3_gpio_out
					break;
			}
			break;
		case 4:
			switch(direct){
				case 0:
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,-pwm);//w4_pwm_out
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,0);//w4_gpio_out
					break;
				case 1:
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,1000-pwm);//w4_pwm_out
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,1);//w4_gpio_out
					break;
			}
			break;
	}
		
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//input capture NVIC     测速
{
	static float catch1[3], catch2[3];
	int judge1, judge2;
	if (htim==&htim1 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)//w3_gpio_in
	{
		judge1=HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_14) ;		
		catch1[1]=HAL_TIM_ReadCapturedValue (htim, TIM_CHANNEL_3) ;		
		catch1[2]=catch1[1]-catch1[0];		
		if (catch1[2]<0) catch1[2]+=0xffff;		
		catch1[0]=catch1[1];	
		capture_buf3+=catch1[2];       //累计时间
		capture_flag3++;              //标志位 标记累计次数
		if(capture_flag3>11)
		{			
		 curgest.v3 =paraspeed/capture_buf3;
		 capture_buf3=0;
		 capture_flag3=0;
		}
	}
	if (htim==&htim1 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)//w4_gpio_in
	{
		judge2=HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_15) ;
		catch2[1]=HAL_TIM_ReadCapturedValue (htim, TIM_CHANNEL_4) ;
		catch2[2]=catch2[1]-catch2[0];		
		if (catch2[2]<0) catch2[2]+=0xffff;		
		catch2[0]=catch2[1];	
		curgest.x4+=0.52f;
		capture_buf4+=catch2[2];       //累计时间
		capture_flag4++;              //标志位 标记累计次数
		if(capture_flag4>11)
		{			
		   curgest.v4 =paraspeed/capture_buf4;
		   capture_buf4=0;
		   capture_flag4=0;
		}
	}
		
}

void DataGet1(uint8_t rx_buf)//蓝牙 通过手机直接来调节PID并且可以显示速度
{
		switch(rx_buf)
		{
			case 'a': v3pid.kp-=1;printf("v3pid.kp%f\r\n",v3pid.kp);break  ;
			case 'b': v3pid.ki-=0.1;printf("v3pid.ki%f\r\n",v3pid.ki);break ;
			case 'c': v3pid.kd-=1;printf("v3pid.kd%f\r\n",v3pid.kd);break  ;
			case 'A': v3pid.kp+=1;printf("v3pid.kp%f\r\n",v3pid.kp);break  ;
			case 'B': v3pid.ki+=1;printf("v3pid.ki%f\r\n",v3pid.ki);break  ;
			case 'C': v3pid.kd+=1;printf("v3pid.kd%f\r\n",v3pid.kd);break  ;
			case 'd': v4pid.kp-=1;printf("v4pid,kp%f\r\n",v4pid.kp);break  ;
			case 'e': v4pid.ki-=0.1;printf("v4pid.ki%f\r\n",v4pid.ki);break  ;
			case 'f': v4pid.kd-=1;printf("v4pid.kd%f\r\n",v4pid.kd);break  ;
			case 'D': v4pid.kp+=1;printf("v4pid.kp%f\r\n",v4pid.kp);break  ;
			case 'E': v4pid.ki+=0.1;printf("v4pid.ki%f\r\n",v4pid.ki);break  ;
			case 'F': v4pid.kd+=0.1;printf("v4pid.kd%f\r\n",v4pid.kd);break  ;
			
			case 'g': printf("l s%f n%f\r\n",targest.v4,curgest.v4);break;
			case 'G': printf("r s%f n%f\r\n",targest.v3,curgest.v3);break;	
			
//			case 'a': angzpid.kp-=0.1;printf("angzpid.kp%f\r\n",angzpid.kp);break  ;
//			case 'b': angzpid.ki-=0.01;printf("angzpid.ki%f\r\n",angzpid.ki);break  ;
//			case 'c': angzpid.kd-=0.1;printf("angzpid.kd%f\r\n",angzpid.kd);break  ;
//			case 'A': angzpid.kp+=0.1;printf("angzpid.kp%f\r\n",angzpid.kp);break  ;
//			case 'B': angzpid.ki+=0.01;printf("angzpid.ki%f\r\n",angzpid.ki);break  ;
//			case 'C': angzpid.kd+=0.1;printf("angzpid.kd%f\r\n",angzpid.kd);break  ;
//			case 'd': pidK -= 0.1; printf("current pidk is %f\r\n",pidK);break;
//			case 'D': pidK += 0.1; printf("current pidk is %f\r\n",pidK);break;	
//			case 'e': printf("pwmright%f\r\n",curgest.pwm_v3 );break  ;
//		  case 'E': printf("pwmleft%f\r\n",curgest.pwm_v4 );break  ;
//			case 'h': inittarspeed-= 1; printf("inittar is %f\r\n",inittarspeed);break;	
//			case 'H': inittarspeed+= 1; printf("inittar is %f\r\n",inittarspeed);break;	
      default: break  ;				
		}
		HAL_UART_Receive_IT(&huart1,&rx_buf1,1);
}
void DataGet2(uint8_t data)//陀螺仪
{
	
}
void DataGet3(uint8_t *data)              //树莓派通讯 输入角度参数
{
	  int angz;
	 angz=transfer_int(data);
   curgest.angz=angz;  
		if(curgest.angz>83&&curgest.angz<250)
		{
		targest.v4=inittarspeed-PID(&angzpid,targest.angz-curgest.angz,-20,20) * pidK;
	  targest.v3=inittarspeed+PID(&angzpid,targest.angz-curgest.angz,-20,20) * pidK;
		}
		else if(curgest.angz<77)
		{
		targest.v4=inittarspeed-PID(&angzpid,targest.angz-curgest.angz,-20,20) * pidK;
	  targest.v3=inittarspeed+PID(&angzpid,targest.angz-curgest.angz,-20,20) * pidK;
		}
//		else if(curgest.angz==255)
//		{
//			inittarspeed=0;
//			targest.v4=0;
//      targest.v3=0;
//      HAL_TIM_Base_Start_IT(&htim3);
//		}
//		else if(curgest.angz==254)
//		{
//			inittarspeed=0;
//			targest.v4=0;
//      targest.v3=0;
//		}
		else
		{
		targest.v4=inittarspeed;
	  targest.v3=inittarspeed;
		}
	
	 HAL_UART_Receive_IT(&huart3,rx_buf3,4);
	 HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);



}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口中断 接收完成回调函数  stm32的三个串口一旦接收完成了数据都会进入这个函数
{
	if(huart == &huart3)
	{
		DataGet3(rx_buf3);
	}
	if(huart == &huart1)            //蓝牙通讯串口 
	{
		DataGet1(rx_buf1);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)        //定时器计算PID 频率为50Hz
{
	if(htim->Instance == TIM2)
	{
		targest.pwm_v3+=Incremental_PID(&v3pid,targest.v3-curgest.v3, 500,-500);
		targest.pwm_v4+=Incremental_PID(&v4pid,targest.v4-curgest.v4, 500,-500);	
  }	
	if(htim->Instance == TIM3)
	{

//		startflag+=1;
//		if(startflag==5)
//		{
//			HAL_TIM_Base_Stop_IT(&htim3);
//			inittarspeed=45;
//			targest.v4=inittarspeed;
//      targest.v3=inittarspeed;
//			
//		}
	}
}
float PID(struct pidstruct *e,float err,float outlow,float outhigh)     //位置式PID
{
	float out;
	e->thisde=err;
	e->integralde+=e->thisde;
	if (e->integralde >= 8) e->integralde = 8;
	e->p = e->kp * e->thisde;
	e->i = e->ki * e->integralde;
	e->d = e->kd * (e->thisde - e->lastde);
	e->lastde = e->thisde;
	out = e->p + e->i + e->d;
	if(out>outhigh) {out=outhigh;}
	if(out<outlow) {out=outlow ;}
	return out;
}

float Incremental_PID(struct pidstruct *e,float err, float MaxOutput,float MinOutput)  //增量式PID
{
	float out;
	e->thisde=err;
  e->p = e->kp*(e->thisde - e->lastde);
	e->i =  e->ki * e->thisde;
	e->d = e->kd *(e->thisde - 2.0f*e->lastde + e->beforede);	
	//积分限幅	
	out= (e->p + e->i + e->d );	
	//输出限幅
	e->beforede = e->lastde;
	e->lastde = e->thisde;
	if(out>MaxOutput)
	{
		out=MaxOutput;
	}
	if(out<MinOutput)
	{
		out=MinOutput;
	}
	return out;
}


struct pidstruct initpid(float kp,float ki,float kd,struct pidstruct e)//初始化pid
{
	e.kp=kp;
	e.ki=ki;
	e.kd=kd;
	e.p=0;
	e.i=0;
	e.d=0;
	e.lastde=0;
	e.thisde=0;
	e.beforede=0;
	e.integralde=0;
	return e;
}
void Task_Delay(int time)
{
	
}
void CAR_init()
{
//*************************************************pid 初始化*************************************// 	
//	x3pid=initpid(9,0.7,0,x3pid);              
//	x4pid=initpid(9,0.7,0,x4pid);
	v3pid=initpid(50,150,7,v3pid);
	v4pid=initpid(20,10.9,6.6,v4pid);
	angzpid=initpid(0.6,0 ,0.1,angzpid);
	pidK=1.0;	
//************************************************初始姿态初始化**************************************//
//	curgest.angx=inigest.angx;
//	curgest.angy=inigest.angy;
//	curgest.angz=inigest.angz;
	inittarspeed=40;
	targest.angz=targestangz;
	curgest.v4=0;
	curgest.v3=0;
	targest.v3=inittarspeed;
	targest.v4=inittarspeed;
	curgest.pwm_v3=0;
	curgest.pwm_v4=0;
	Wheel(4,curgest.pwm_v4);
	Wheel(3,curgest.pwm_v3);
	HAL_Delay(1000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//*********************************************初始化***********************************************//	
	HAL_UART_Receive_IT(&huart1,&rx_buf1,1);   //串口通讯初始化 
	HAL_UART_Receive_IT(&huart3,rx_buf3,4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);   //pwm通道初始化
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim2);             //定时器开始
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1); //编码器采集通道初始化
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
	CAR_init();                                 //小车状态初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Wheel(4,targest.pwm_v4);
		Wheel(3,targest.pwm_v3);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
