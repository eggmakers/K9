#include "drv_IntIIC.hpp"
#include "Basic.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

//IIC传输配置
	#define IIC_AutoEND (1<<25)
	#define IIC_Reload (1<<24)
	#define IIC_NBytes(x) (x<<16)
	#define IIC_NACK (1<<15)
	#define IIC_STOP (1<<14)
	#define IIC_START (1<<13)
	#define IIC_HEAD10R (1<<12)
	#define IIC_ADD10 (1<<11)
	#define IIC_READ (1<<10)
	#define IIC_ADDR7(x) (x<<1)
//IIC状态
	#define IIC_ARLO (1<<9)
	#define IIC_BERR (1<<8)
	#define IIC_TCR (1<<7)
	#define IIC_TC (1<<6)
	#define IIC_STOPF (1<<5)
	#define IIC_NACKF (1<<4)
#define IIC_TIMEOUT 1.0*configTICK_RATE_HZ

#ifdef SYSFREQ480
	#define TIM400k 0xb03fdb
	#define TIM100k 0x307075B1
	#define TIM50k 0xe0901f76
#else
	#define TIM400k 0x9034B6
	#define TIM100k 0x10C0ECFF
	#define TIM50k 0xc0901e71
#endif

//IIC互斥锁
static SemaphoreHandle_t IICMutex = xSemaphoreCreateRecursiveMutex();
//发送完成标志
static EventGroupHandle_t events = xEventGroupCreate();
//待发送数据
static uint16_t RMTxLength = 0;
static uint16_t RMRxLength = 0;

/*上锁保证通信连续性
	上锁之后必须解锁
	Sync_waitTime：超时时间
*/
bool Lock_IntIIC( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTime >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	if( xSemaphoreTakeRecursive( IICMutex , Sync_waitTicks ) == pdTRUE )
		return true;
	return false;
}
void Unlock_IntIIC()
{
	xSemaphoreGiveRecursive( IICMutex );
}

/*7位地址发送数据
	addr：7位从器件地址
	datas：要发送的数据指针
	length：数据长度
	Sync_waitTime：超时时间
*/
bool IntIIC_SendAddr7( uint8_t addr, const uint8_t* datas, uint16_t length, IICSpeed speed, double Sync_waitTime )
{
	if( length == 0 )
		return false;
	
#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)datas ) )
	{	//缓冲区为非cache区域
	}
	else if( ((uint32_t)datas & 0x1f) == 0 )
	{	//缓冲区32字节对齐
		SCB_CleanDCache_by_Addr((uint32_t*)datas, length);
	}
	else	//无缓冲区返回错误
		return false;
#endif
	
	if( Lock_IntIIC(Sync_waitTime) )
	{
		//计算当次发送长度
		RMTxLength = length;
		if( RMTxLength > 255 )
			length = 255;
		else
			length = RMTxLength;
		RMTxLength -= length;
		
		//清空状态
		xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
		I2C2->CR1 &= ~( (1<<15) | (1<<14) );	
		//IIC时间配置
		if( speed == IICSpeed_Fast400k )
			I2C2->TIMINGR = TIM400k;	//400k
		else if( speed == IICSpeed_Standard50k )
			I2C2->TIMINGR = TIM50k;	//35k
		else
			I2C2->TIMINGR = TIM100k;	//100k
		
		I2C2->CR1 |= (1<<0);
		/*配置DMA*/
			//清空DMA状态
			DMA1_Stream2->CR &= ~(1<<0);
			DMA1->LIFCR = (1<<21) | (1<<20) | (1<<19)  | (1<<18)  | (1<<16);
			//设置DMA存储器地址
			DMA1_Stream2->M0AR = (uint32_t)datas;
			//设置DMA传输数量
			DMA1_Stream2->NDTR = RMTxLength + length;
			//使能DMA
			DMA1_Stream2->CR |= (1<<0);	
		/*配置DMA*/
		I2C2->CR1 |= (1<<14);
		
		//开始传输
		if( RMTxLength == 0 )
			I2C2->CR2 = 
				IIC_AutoEND | 
				IIC_NBytes(length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		else
			I2C2->CR2 = 
				IIC_Reload | 
				IIC_NBytes(length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		
		//等待传输完成
		uint32_t revents = xEventGroupWaitBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT );
		I2C2->CR1 &= ~( 1<<0 );
		//解锁
		Unlock_IntIIC();
		
		if( revents & (IIC_NACKF|IIC_ARLO) )
			return false;
		else if( revents & IIC_STOPF )
			return true;
		return false;	
	}
	return false;
}

/*7位地址发送并接收数据
	addr：7位从器件地址
	datas：要发送的数据指针
	length：数据长度
	Sync_waitTime：超时时间
*/
bool IntIIC_ReceiveAddr7( uint8_t addr, uint8_t* rx_datas, uint16_t rx_length, IICSpeed speed, double Sync_waitTime )
{
	if( rx_length==0 )
		return false;
	
#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)rx_datas ) )
	{	//缓冲区为非cache区域
	}
	else if( ((uint32_t)rx_datas & 0x1f) == 0 )
	{	//缓冲区32字节对齐
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)rx_datas, rx_length);
	}
	else	//无缓冲区返回错误
		return false;
#endif
	
	if( Lock_IntIIC(Sync_waitTime) )
	{
		//计算当次发送长度
		RMRxLength = rx_length;
		if( RMRxLength > 255 )
			rx_length = 255;
		else
			rx_length = RMRxLength;
		RMRxLength -= rx_length;
		RMTxLength = 0;

		//清空状态
		xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
		I2C2->CR1 &= ~( (1<<15) | (1<<14) );
		//IIC时间配置
		if( speed == IICSpeed_Fast400k )
			I2C2->TIMINGR = TIM400k;	//400k
		else if( speed == IICSpeed_Standard50k )
			I2C2->TIMINGR = TIM50k;	//35k
		else
			I2C2->TIMINGR = TIM100k;	//100k
		
		I2C2->CR1 |= (1<<0);	
		/*配置DMA*/
			//清空DMA状态
			DMA1_Stream4->CR &= ~(1<<0);
			DMA1->HIFCR = (1<<27) | (1<<26) | (1<<25)  | (1<<24)  | (1<<22);
			//设置DMA存储器地址
			DMA1_Stream4->M0AR = (uint32_t)rx_datas;
			//设置DMA传输数量
			DMA1_Stream4->NDTR = RMRxLength + rx_length;
			//使能DMA
			DMA1_Stream4->CR |= (1<<0);	
		/*配置DMA*/		
			
		//开启IIC RXDMA
		I2C2->CR1 |= (1<<15);
		
		//开始传输
		if( RMRxLength == 0 )
			I2C2->CR2 = 
				IIC_READ | 
				IIC_AutoEND | 
				IIC_NBytes(rx_length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		else
			I2C2->CR2 = 
				IIC_READ | 
				IIC_Reload | 
				IIC_NBytes(rx_length) | 
				IIC_ADDR7(addr) | 
				IIC_START;

		//等待传输完成
		uint32_t revents = xEventGroupWaitBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT );
		I2C2->CR1 &= ~( 1<<0 );
		//解锁
		Unlock_IntIIC();
		
		if( revents & (IIC_NACKF|IIC_ARLO) )
			return false;
		else if( revents & IIC_STOPF )
			return true;
		return false;	
	}
	return false;
}

/*7位地址发送并接收数据
	addr：7位从器件地址
	datas：要发送的数据指针
	length：数据长度
	Sync_waitTime：超时时间
*/
bool IntIIC_SendReceiveAddr7( uint8_t addr, const uint8_t* tx_datas, uint16_t tx_length, const uint8_t* rx_datas, uint16_t rx_length, IICSpeed speed, double Sync_waitTime )
{
	if( tx_length==0 || rx_length==0 )
		return false;
	
#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)tx_datas ) )
	{	//缓冲区为非cache区域
	}
	else if( ((uint32_t)tx_datas & 0x1f) == 0 )
	{	//缓冲区32字节对其
		SCB_CleanDCache_by_Addr((uint32_t*)tx_datas, tx_length);
	}
	else	//无缓冲区返回错误
		return false;
	if( isDMABuf( (uint32_t)rx_datas ) )
	{	//缓冲区为非cache区域
	}
	else if( ((uint32_t)rx_datas & 0x1f) == 0 )
	{	//缓冲区32字节对齐
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)rx_datas, rx_length);
	}
	else	//无缓冲区返回错误
		return false;
#endif
	
	if( Lock_IntIIC(Sync_waitTime) )
	{
		//计算当次发送长度
		RMTxLength = tx_length;
		if( RMTxLength > 255 )
			tx_length = 255;
		else
			tx_length = RMTxLength;
		RMTxLength -= tx_length;
		
		//计算当次接收长度
		RMRxLength = rx_length;
		
		//清空状态
		xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
		I2C2->CR1 &= ~( (1<<15) | (1<<14) );
		//IIC时间配置
		if( speed == IICSpeed_Fast400k )
			I2C2->TIMINGR = TIM400k;	//400k
		else if( speed == IICSpeed_Standard50k )
			I2C2->TIMINGR = TIM50k;	//35k
		else
			I2C2->TIMINGR = TIM100k;	//100k
		
		I2C2->CR1 |= (1<<0);	
		/*配置DMA*/
			//清空DMA状态
			DMA1_Stream2->CR &= ~(1<<0);
			DMA1->LIFCR = (1<<21) | (1<<20) | (1<<19)  | (1<<18)  | (1<<16);
			//设置DMA存储器地址
			DMA1_Stream2->M0AR = (uint32_t)tx_datas;
			//设置DMA传输数量
			DMA1_Stream2->NDTR = RMTxLength + tx_length;
			//使能DMA
			DMA1_Stream2->CR |= (1<<0);	
		
			//清空DMA状态
			DMA1_Stream4->CR &= ~(1<<0);
			DMA1->HIFCR = (1<<5) | (1<<4) | (1<<3)  | (1<<2)  | (1<<0);
			//设置DMA存储器地址
			DMA1_Stream4->M0AR = (uint32_t)rx_datas;
			//设置DMA传输数量
			DMA1_Stream4->NDTR = RMRxLength;
//			//使能DMA
//			DMA1_Stream4->CR |= (1<<0);	
		/*配置DMA*/				
		I2C2->CR1 |= (1<<14);
		
		//开始传输
		if( RMTxLength == 0 )
			I2C2->CR2 = 
				IIC_NBytes(tx_length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		else
			I2C2->CR2 = 
				IIC_Reload | 
				IIC_NBytes(tx_length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		
		//等待传输完成
		uint32_t revents = xEventGroupWaitBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT );
		I2C2->CR1 &= ~( 1<<0 );
		//解锁
		Unlock_IntIIC();
		
		if( revents & (IIC_NACKF|IIC_ARLO) )
			return false;
		else if( revents & IIC_STOPF )
			return true;
		return false;	
	}
	return false;
}

extern "C" void I2C2_EV_IRQHandler()
{	
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	uint8_t ISR = I2C2->ISR;
	
	if( ISR & IIC_TC )
	{	//需要发送Restart
		if( RMRxLength > 0 )
		{
			//计算当次发送长度
			uint8_t length;
			if( RMRxLength > 255 )
				length = 255;
			else
				length = RMRxLength;
			RMRxLength -= length;

			//使能DMA
			DMA1_Stream4->CR |= (1<<0);	
			//开启IIC RXDMA
			I2C2->CR1 |= (1<<15);
			
			uint32_t addr = I2C2->CR2 & 0x3ff;
			//开始传输
			if( RMRxLength == 0 )
				I2C2->CR2 = 
					IIC_READ | 
					IIC_AutoEND | 
					IIC_NBytes(length) | 
					addr | 
					IIC_START;
			else
				I2C2->CR2 = 
					IIC_READ | 
					IIC_Reload | 
					IIC_NBytes(length) | 
					addr | 
					IIC_START;
		}
		else
			I2C2->CR2 = IIC_STOP;
	}
	else if( ISR & IIC_TCR )
	{	//继续发送
		if( RMTxLength > 0 )
		{
			//计算当次发送长度
			uint8_t length;
			if( RMTxLength > 255 )
				length = 255;
			else
				length = RMTxLength;
			RMTxLength -= length;

			//继续发送
			uint32_t CR2 = I2C2->CR2 & (~(0xff<<16));
			I2C2->CR2 = CR2 | IIC_NBytes(length);
		}
		else if( RMRxLength > 0 )
		{
			//计算当次发送长度
			uint8_t length;
			if( RMRxLength > 255 )
				length = 255;
			else
				length = RMRxLength;
			RMRxLength -= length;

			//继续发送
			uint32_t CR2 = I2C2->CR2 & (~(0xff<<16));
			I2C2->CR2 = CR2 | IIC_NBytes(length);
		}
		else
			I2C2->CR2 = IIC_STOP;
	}
	else
		//置位完成标志	
		xEventGroupSetBitsFromISR( events, ISR, &HigherPriorityTaskWoken );
	
	//清空已记录的所有标志
	I2C2->ICR = ISR & ( (1<<5) | (1<<4) | (1<<3) );
	
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

extern "C" void I2C2_ER_IRQHandler()
{
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	uint32_t ISR = I2C2->ISR;
	
	//清空已记录的错误标志
	I2C2->ICR = ISR & ( (1<<13) | (1<<12) | (1<<11) | (1<<10) | (1<<9) | (1<<8) );
	//置位错误标志
	xEventGroupSetBitsFromISR( events, ISR, &HigherPriorityTaskWoken );	
	
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

void init_drv_IntIIC()
{
	/*配置IIC2*/
		//打开IIC2电源
		RCC->APB1LENR |= (1<<22);
		os_delay(0.01);
		
		//复位IIC
		I2C2->CR1 = 0;
		os_delay(0.005);
	
		//IIC时间配置
		#ifdef SYSFREQ480
			//I2C2->TIMINGR = 0xb03fdb;	//400k
			I2C2->TIMINGR = 0x307075B1;	//100k
		#else
			//I2C2->TIMINGR = 0x9034B6;//400K
			I2C2->TIMINGR = 0x10C0ECFF;//100K
		#endif

		//使能IIC
		I2C2->CR1 = 
			//(1<<15) | (1<<14) | 
			(1<<7) | (1<<6) | (1<<5) | (1<<4) | 
			1;
		NVIC_SetPriority(I2C2_EV_IRQn,5);
		NVIC_EnableIRQ(I2C2_EV_IRQn);
		NVIC_SetPriority(I2C2_ER_IRQn,5);
		NVIC_EnableIRQ(I2C2_ER_IRQn);
	/*配置IIC2*/
	
	/*DMA初始化*/
		//打开DMA1时钟
		RCC->AHB1ENR |= (1<<0);
		os_delay(0.005);
		
		//DMA1_Stream2 IIC2 Tx
		DMA1_Stream2->PAR = (uint32_t)&I2C2->TXDR;
		DMAMUX1_Channel2->CCR = (36<<0);
		DMA1_Stream2->CR = (1<<16) | (0<<13) | (1<<10) | (0<<9) | (0b01<<6);
		DMA1_Stream2->FCR = (1<<2) | (3<<0);
		
		//DMA1_Stream4 IIC2 Rx
		DMA1_Stream4->PAR = (uint32_t)&I2C2->RXDR;
		DMAMUX1_Channel4->CCR = (35<<0);
		DMA1_Stream4->CR = (1<<16) | (0<<13) | (1<<10) | (0<<9) | (0b00<<6);
		DMA1_Stream4->FCR = (1<<2) | (3<<0);
		
	/*DMA初始化*/
	
	/*配置GPIO PB10(IIC2 SCL) PB11(IIC2 SDA)*/
		//打开GPIOB电源
		RCC->AHB4ENR |= (1<<1);
		os_delay(0.01);
				
		//引脚开漏上拉
		set_register( GPIOB->OTYPER , 0 , 10*1 , 1 );	//PB10
		set_register( GPIOB->OTYPER , 1 , 11*1 , 1 );	//PB11
		set_register( GPIOB->PUPDR , 0b01 , 10*2 , 2 );	//PB10
		set_register( GPIOB->PUPDR , 0b01 , 11*2 , 2 );	//PB11
		//中速
		set_register( GPIOB->OSPEEDR , 0b10 , 10*2 , 2 );	//PB10
		set_register( GPIOB->OSPEEDR , 0b10 , 11*2 , 2 );	//PB11
		//复用IIC1
		set_register( GPIOB->AFR[1], 4, 10*4-32, 4 );	//PB10
		set_register( GPIOB->AFR[1], 4, 11*4-32, 4 );	//PB11
		//引脚复用功能
		set_register( GPIOB->MODER , 0b10 , 10*2 , 2 );	//PB10
		set_register( GPIOB->MODER , 0b10 , 11*2 , 2 );	//PB11
	/*配置GPIO PB10(IIC2 SCL) PB11(IIC2 SDA)*/
}