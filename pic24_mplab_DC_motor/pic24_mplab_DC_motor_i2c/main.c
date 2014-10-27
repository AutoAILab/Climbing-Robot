#include "p24fj48ga002.h"
#include "i2c.h"

#define uint8  unsigned char
#define uint16 unsigned int
#define uint32 unsigned long
#define int8   char
#define int16  int
#define int32  long

#define BIT0  1
#define BIT1  2
#define BIT2  4
#define BIT3  8
#define BIT4  16
#define BIT5  32
#define BIT6  64
#define BIT7  128
#define BIT8  256
#define BIT9  512
#define BIT10  1024
#define BIT11  2048
#define BIT12  4096
#define BIT13  8192
#define BIT14  16384
#define BIT15  32768

#define ISR __attribute__((interrupt, auto_psv))

#define Fsoc 11059200
#define PWM_Max 50
#define Key_Delay_Value 10000
#define Speed_Max 24
#define Speed_Min 100

#define I2C_Address 0x62

uint8 Flash_Time,StartUp;
uint16 Counter,SwitchTime,SwitchTime_Negative,Hall,Counter_Temp;
uint8  Current,PWM;
uint8  BeforeTime,CloseTime;
uint16 Speed,Speed_Target,Speed_Change_Count; 
uint16 Key_Delay=Key_Delay_Value;
uint16 Speed_Detect[16],Speed_Detect_Count,Speed_Summer,Speed_Average;
uint8  Receive[7],Send[7],Reci_Count,CheckSum,Receive_Flag,Send_Count;
uint8 i2c_i,rx_data[10],tx_data[10];


struct{
	uint8 Sender;	 	
	uint8 Receiver;
	uint8 Command;	 		
	uint8 Data1;	 	
	uint8 Data2;	 	
} Protocol;

//uint16 Speed_Tab[20]={240,120,80,60,48,40,34,30,27,24,22,20,18,17,16,15,14,13,13,12};

void Delay(uint16 d)
{
	uint16 i,j;
	for(i=0;i<d;i++)
		for(j=0;j<65535;j++);
}

void Delay_Short(uint16 d_s)
{
	uint16 i;
	for(i=0;i<d_s;i++);
}

void Unlock_IO(void)
{

asm volatile (  "MOV #OSCCON, w1 \n"
				"MOV #0x46, w2 \n"
				"MOV #0x57, w3 \n"
				"MOV.b w2, [w1] \n"
				"MOV.b w3, [w1] \n"
				"BCLR OSCCON,#6");

}

void Lock_IO(void)
{

asm volatile (  "MOV #OSCCON, w1 \n"
				"MOV #0x46, w2 \n"
				"MOV #0x57, w3 \n"
				"MOV.b w2, [w1] \n"
				"MOV.b w3, [w1] \n"
				"BSET OSCCON, #6" );
}

void Io_Map()
{

	Unlock_IO();
	
	RPOR1bits.RP2R = 18;//oc1
	RPOR6bits.RP13R = 19;//oc2
	
	// Assign U1RX To Pin RP6
	RPINR18bits.U1RXR = 6;
	// Assign U1TX To Pin RP2
	RPOR3bits.RP7R = 3;

	Lock_IO();
}
void Io_Init()
{
	AD1PCFG=0x9fff;
	TRISB&=~(BIT2|BIT3|BIT12|BIT13);//Mosfet Drive Pin
    TRISB|=BIT14;//Hall input
	TRISA|=BIT1;//Current input
    TRISA&=~BIT0;//Led
    TRISB|=(BIT11|BIT15);//Button input

	LATA&=~BIT0;//Turn on Led 
	LATB&=~(BIT2|BIT3|BIT12|BIT13);//Mosfet Drive Stop
}

void Close_Mosfet(void)
{
		LATB&=~BIT3;
		OC2RS=0;//BIT13
		LATB&=~BIT12;
		OC1RS=0;//BIT2
}

void Uart1_Init(uint32 Baud)
{
	U1BRG=((((Fsoc/2)/Baud)/16)-1);//
	U1STA=0;
	U1MODE=0x8000;
	U1STAbits.UTXEN=1;
	IEC0bits.U1TXIE=1;
	IEC0bits.U1RXIE=1;

	IPC3bits.U1TXIP=3;
	IPC2bits.U1RXIP=3;

}

//void U1Send(uint8 U1data)
//{
//	U1TXREG = U1data;// send back the character to TX buffer for transmission		
//	while ( !U1STAbits.TRMT );	// stay here while character at shift register is not yet transmitted
//}

void Uart_Send(uint8 S_Add,uint8 R_Add,uint8 P_C,uint8 D1,uint8 D2)
{
	Send[0]=0xe0;
	Send[1]=S_Add;
	Send[2]=R_Add;
	Send[3]=P_C;
	Send[4]=D1;
	Send[5]=D2;
	Send[6]=S_Add^R_Add^P_C^D1^D2;
	Send_Count=0;
	U1TXREG = Send[0];
}

void ISR _U1RXInterrupt()
{
	uint8 U1_Temp;
	U1_Temp=U1RXREG;
    if(Receive_Flag==0){
		if(U1_Temp==0xe0){
	      Reci_Count=0;
	      Receive[Reci_Count]=U1_Temp;
	      CheckSum=0;
	      Reci_Count++;
	    }  
	    else{  
	           Receive[Reci_Count]=U1_Temp;
	           if(Reci_Count<=5){
	            CheckSum^=Receive[Reci_Count];
	            Reci_Count++;
	           }
	           else{
				 Receive[Reci_Count]=U1_Temp;
	             if(Receive[6]==CheckSum){//checksum correct
					  Protocol.Sender=Receive[1];
					  Protocol.Receiver=Receive[2];	
					  Protocol.Command=Receive[3];	
					  Protocol.Data1=Receive[4];	
					  Protocol.Data2=Receive[5];		
	                  Receive_Flag=1;  
	             }    
	           }
	    }
	}
	_U1RXIF = 0;					// manually cleared U2RX Interrupt flag
}

void RX_Receive(void)
{
	if(Receive_Flag==1){
		Receive_Flag=0;
		switch(Protocol.Command){
			case 0x01://speed--
				if(Speed_Target<Speed_Min) 
					Speed_Target++;
				else
					Speed_Target=Speed_Min;
				break;
			case 0x02://speed +
				if(Speed_Target>Speed_Max)
					Speed_Target--;
				else 
					Speed_Target=Speed_Max;
				break;
			case 0x03:
				if((Protocol.Data2>=Speed_Max)&&(Protocol.Data2<=Speed_Min)){
					Speed_Target= Protocol.Data2;
				}
				break;
			case 0x10:
				Uart_Send(0x01,0x00,0x90,0x00,(uint8)Speed_Average);
				break;
			case 0x7e:
				StartUp=2;
				break;
			case 0x7f:
				StartUp=0;
				Close_Mosfet();
				break;
			default:
				break;
	
		}

	}
}

void ISR _U1TXInterrupt()
{
	if(Send_Count<=5){
		Send_Count++;
		U1TXREG = Send[Send_Count];				
	}
	_U1TXIF = 0;					// manually cleared Tx Interrupt flag
}


void Button(void)
{
	if((PORTB&BIT11)==0){
		Key_Delay--;
		 if(((PORTB&BIT11)==0)&&(Key_Delay<1)){
			//LATA|=BIT0;
			if(Speed_Target<Speed_Min) 
				Speed_Target++;
			else
				Speed_Target=Speed_Min;
			//U1Send(Speed);
			Key_Delay=Key_Delay_Value;
		}
    } 
	if((PORTB&BIT15)==0){
		Key_Delay--;
		if(((PORTB&BIT15)==0)&&(Key_Delay<1)){
			//LATA&=~BIT0;
			if(Speed_Target>Speed_Max)
				Speed_Target--;
			else 
				Speed_Target=Speed_Max;
			//U1Send(Speed);
			Key_Delay=Key_Delay_Value;		
		}
    } 
    if((PORTB&BIT14)==BIT14){
		LATA|=BIT0;
    }  
	else if((PORTB&BIT14)==0){
		LATA&=~BIT0;
    } 
}

void Timer2_Init(uint16 Time2_Value)
{
	T2CON=0;
    T2CONbits.TON=0;
	T2CONbits.TCKPS=0;//64 Pre-scale
	T2CONbits.TCS=0;
	TMR2=0;
	PR2=(float)Time2_Value*Fsoc/2000000+0.5-1;//1us-> pr2=5

	//IPC0bits.T2IP=0;
   // IFS0bits.T2IF=0;
	//IEC0bits.T2IE=0;

	T2CONbits.TON=1;

}

void Pwm_Init(void)
{
	OC1CON=0x0000;
	OC1R=0;
	OC1RS=0;
	OC1CON=0x0006;

	OC2CON=0x0000;
	OC2R=0;
	OC2RS=0;
	OC2CON=0x0006;

	Timer2_Init(20);//1us  PR2=5
}



void Timer1_Init(uint16 Time_Value)
{
	T1CON=0;
    T1CONbits.TON=0;
	T1CONbits.TCKPS=1;//8 Pre-scale
	T1CONbits.TCS=0;
	TMR1=0;
	PR1=(float)Time_Value*Fsoc/2000000/8+0.5-1; 

	IPC0bits.T1IP=4;
    IFS0bits.T1IF=0;
	IEC0bits.T1IE=1;

	T1CONbits.TON=1;
}

void ISR _T1Interrupt(void)
{
	Counter++;
	_T1IF=0;
}



void Init(void)
{
	uint8 i;
	Delay(5);
//	SwitchTime=40;
	Counter=0;
	PWM=3;
	Speed_Target=100;
	Speed_Detect_Count=0;
	Speed_Summer=0;
	Speed_Average=0;
	Receive_Flag=0;
	for(i=1;i<10;i++)
		Speed_Detect[i]=0;
}



void Open_Mosfet(uint8 R_L,uint8 Duty)//1--RT-LB  2--LB-RT  duty--%
{
	if((R_L==1)&&(Duty<=PWM_Max)){
		LATB&=~BIT3;
		OC2RS=0;//BIT13	
		LATB|=BIT12;
		OC1RS=(float)PR2*Duty*0.01;//BIT2
	}
	else if((R_L==2)&&(Duty<=PWM_Max)){
		LATB&=~BIT12;
		OC1RS=0;//BIT2
		LATB|=BIT3;
		OC2RS=(float)PR2*Duty*0.01;//BIT13
	}
	else
	   Close_Mosfet();	

}

void Continue_Current(void)
{
	LATB|=BIT3;
	OC2RS=0;//BIT13
	LATB|=BIT12;
	OC1RS=0;//BIT2

}


void Current_Detect(void)
{
	if((PORTA&BIT1)==BIT1){
		Current=1;
		LATB&=~(BIT3|BIT13);
		LATB&=~(BIT2|BIT12);
	}
	else{
		Current=0;
	}

}

void Led_Flash(void)
{
 	uint8 i;
	for(i=0;i<30;i++){
		LATA&=~BIT0;
		Delay(5);
		LATA|=BIT0;
		Delay(5);
	}	
}

void S_Average(uint16 S_D)
{
	Speed_Summer+=S_D;
	Speed_Summer-=Speed_Detect[Speed_Detect_Count];
	Speed_Detect[Speed_Detect_Count]=S_D;
	Speed_Detect_Count++;
	if(Speed_Detect_Count>=16)
		Speed_Detect_Count=0;
	Speed_Average=(Speed_Summer>>4);
	
	if(Speed_Average>35){
		BeforeTime=10;
		CloseTime=10;
	}
/*	else if(Speed_Average>=30){
		BeforeTime=12;
		CloseTime=9;
	}
*/
	else if(Speed_Average>=28){
		BeforeTime=0.4*Speed_Average;
		CloseTime=0.2*Speed_Average;
	}
	else if(Speed_Average>=24){
		BeforeTime=0.45*Speed_Average;
		CloseTime=0.2*Speed_Average;
	}
	else{
		BeforeTime=0.46*Speed_Average;
		CloseTime=0.2*Speed_Average;
	}
	SwitchTime=Speed_Average-BeforeTime-CloseTime;
	SwitchTime_Negative=SwitchTime;
}

void Motor_Start(void)
{
	uint8 i,Start_Count;
	Counter=0;
	Start_Count=0;
	PWM=0;

	for(i=0;i<100;i++){
		Hall=(PORTB&BIT14);		
		if(Hall==0){
			Open_Mosfet(2,PWM);					
		}
		else if(Hall==BIT14){
			Open_Mosfet(1,PWM);
	 	} 
		while((Hall==(PORTB&BIT14))&&(Counter<400)); 
		if(Counter>=400){
			if(PWM<15) 
				PWM++;
			else
				PWM=15;
		}
		else{
			S_Average(Counter);
		    Start_Count++;
		}
		Counter=0;
	}
		if(Start_Count<10){
			while(1){
				Close_Mosfet();
				StartUp=0;			
				Led_Flash();
			}
		}
		else{
			StartUp=1;
			Hall=(PORTB&BIT14);
			Counter=0;
			if(Hall==0){
				Open_Mosfet(2,PWM);					
			}
			else if(Hall==BIT14){
				Open_Mosfet(1,PWM);
	 		} 
		}

}


void Motor_Driver(void)
{
		if(Hall!=(PORTB&BIT14)){
			Counter_Temp=Counter;
			Counter=0;
			if((Counter_Temp<500)&&(Counter_Temp>8)){
				S_Average(Counter_Temp);
				//U1Send(SwitchTime);
				Hall=(PORTB&BIT14);	
			}

			if(Speed_Target>(Speed_Average+1)){
				if((Speed_Change_Count++)>80){
					if(PWM>1)	
						PWM--;
					else
						PWM=0;					
					Speed_Change_Count=0;
				}
			}
			if(Speed_Target<(Speed_Average-1)){
				if((Speed_Change_Count++)>80){
					if(PWM<PWM_Max-1)	
						PWM++;
					else
						PWM=PWM_Max;
					Speed_Change_Count=0;
				}
			}

		}
		if((Counter>=SwitchTime)&&(Counter<(SwitchTime+CloseTime))){
 				Close_Mosfet();
		}
		if(Counter>=(SwitchTime+CloseTime)){
			if(Hall==0)
				Open_Mosfet(1,PWM);	
			else if(Hall==BIT14)
				Open_Mosfet(2,PWM);
		}
/*
		if((Hall==0)&&(Counter>=(SwitchTime+CloseTime))){
				Open_Mosfet(1,PWM);					
		}
		if((Hall==BIT14)&&(Counter>=(SwitchTime+CloseTime+1))){
				Open_Mosfet(2,PWM);
		}
*/
		if(Counter>=500){
			while(1){
				Close_Mosfet();
				Led_Flash();
			}
		}
}


void I2C_Init(void)
{
    IFS1bits.SI2C1IF = 0;
	IEC1bits.SI2C1IE = 1;
	IPC4bits.SI2C1IP=2;

	I2C1CONbits.I2CEN=1;
	

	I2C1ADD = I2C_Address;	/* 0b1100001 Slave MCU address (customized)
					 * 4 msb is 1100, customized as slave MCU
					 * 3 lsb is 001, customized as the first slave MCU
					 */
}


void ISR _SI2C1Interrupt(void)
{
	
	if (I2C1STATbits.D_A==0) {
		// address byte
		i2c_i = 0;			// clear array to first
		rx_data[i2c_i] = SlavegetcI2C1();
		
		if (I2C1STATbits.R_W) {
			// read from slave
			SlaveputcI2C1(tx_data[i2c_i++]);	// put one char to i2c buffer
		}	
	//	AckI2C1();
	} else {
		// data byte
		if (I2C1STATbits.R_W) {
			// read from slave
			SlaveputcI2C1(tx_data[i2c_i++]);	// put one char to i2c buffer
		} else {
			// write to slave
			rx_data[i2c_i++] = I2C1RCV;;	// get one char from i2c buffer
			if(i2c_i==7){
				if((rx_data[0]==0xe0)&&(rx_data[6]==(rx_data[1]^rx_data[2]
				^rx_data[3]^rx_data[4]^rx_data[5]))){
					  Protocol.Sender=rx_data[1];
					  Protocol.Receiver=rx_data[2];	
					  Protocol.Command=rx_data[3];	
					  Protocol.Data1=rx_data[4];	
					  Protocol.Data2=rx_data[5];		
	                  Receive_Flag=1;
				} 
			}
		}
	}
	mI2C1_SlaveClearIntr();		// clears the I2C Slave interrupt flag
}


int main()
{
	Io_Init();
	Io_Map();
	Init();
    Timer1_Init(25);//us
 	Pwm_Init();
	Close_Mosfet();
	Delay(10);
	Uart1_Init(115200);
	//U1Send('O');
	//U1Send('K');
	//Motor_Start();

    I2C_Init();

	while(1){
		RX_Receive();
		if(StartUp==1){
			Button();
			Motor_Driver();
		}
		else if(StartUp==2){
			StartUp=0;
			Motor_Start();
		}
		else{
			Close_Mosfet();
		}
	}
}