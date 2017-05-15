////////// Includes ------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
//*Consola
#include "utils/uartstdio.h"
//*Libreria SD
#include "FaTFS/ff.h"
#include "FaTFS/diskio.h"
//*SPI
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
//*CAN
#include "inc/hw_can.h"
#include "driverlib/can.h"

////////// Defines -------------------------------------------------------------------------------------------

//*Para los WS(Front/Rear | Left/Right)
#define WS_PERIPH SYSCTL_PERIPH_GPIOD
#define WS_BASE GPIO_PORTD_BASE
#define ws_f_l GPIO_PIN_0
#define ws_f_r GPIO_PIN_1
#define ws_r_l GPIO_PIN_2
#define ws_r_r GPIO_PIN_3
#define ws_f_l_int GPIO_INT_PIN_0
#define ws_f_r_int GPIO_INT_PIN_1
#define ws_r_l_int GPIO_INT_PIN_2
#define ws_r_r_int GPIO_INT_PIN_3

//*Para el parametro de entrada de f_printf donde obtenemos cada canal de los ADC
#define	ADC1	Val_ADC[0],Val_ADC[1],Val_ADC[2],Val_ADC[3],Val_ADC[4],Val_ADC[5],Val_ADC[6],Val_ADC[7]
#define	ADC2	Val_ADC[8],Val_ADC[9],Val_ADC[10],Val_ADC[11],Val_ADC[12],Val_ADC[13],Val_ADC[14],Val_ADC[15]
#define	ADC3	Val_ADC[16],Val_ADC[17],Val_ADC[18],Val_ADC[19],Val_ADC[20],Val_ADC[21],Val_ADC[22],Val_ADC[23]
#define MsgRX1	MsgDataRX1[0],MsgDataRX1[1],MsgDataRX1[2],MsgDataRX1[3],MsgDataRX1[4],MsgDataRX1[5],MsgDataRX1[6],MsgDataRX1[7]
#define MsgRX2	MsgDataRX2[0],MsgDataRX2[1],MsgDataRX2[2],MsgDataRX2[3],MsgDataRX2[4],MsgDataRX2[5],MsgDataRX2[6],MsgDataRX2[7]
#define MsgRX3	MsgDataRX3[0],MsgDataRX3[1],MsgDataRX3[2],MsgDataRX3[3],MsgDataRX3[4],MsgDataRX3[5],MsgDataRX3[6],MsgDataRX3[7]
//*Etiquetas para las posiciones del vector de los ADC
#define Oil_T	Val_ADC[20]
#define Temp_CockPit	Val_ADC[22]
#define Fuel_Level		Val_ADC[23]
#define Oil_P			Val_ADC[21]
//#define Temp_Water_H	Val_ADC[]
//#define Temp_Water_L	Val_ADC[]

//{Temp_Water_H,Temp_Water_L,Temp_CockPit,Fuel_Level,Oil_P_L,Oil_P_H,Oil_T_L,Oil_T_H}


//*Para el boton de CLR de la Cuenta y el LED (Mismo Puerto)
#define Button_PERIPH SYSCTL_PERIPH_GPIOB
#define ButtonBase GPIO_PORTB_BASE
#define REC_But GPIO_PIN_2
#define REC_But_Int GPIO_INT_PIN_2

//#define	WS_INT	0	//Si WS_INT esta definido-> los WS sera controlados por las INT_GPIO_D
						//TAMBIEN HAY QUE DEFINIRLO EN LA TABLA DE VECTORES DE EXCEPCION
						//Se usa ensamblado condicional en muchas partes del codigo


////////// Variables Globales -----------------------------------------------------------------------------------------
FATFS FatFs;					//FatFs area de trabajo requerida para cada unidad montada
FIL logfile;					//Varibale requerida por la libreria para cada archivo abierto
uint32_t ui32MsgData;			//Dato a enviar
uint8_t *pui8MsgData;			//Puntero para apuntar al elemento del vector de n elementos
uint32_t data_ADC=0;			//Dato leido de un canal del ADC
uint32_t data_ADC_aux=0;		//Variable aux par adecuar dato
uint16_t ui16ws_f_l=0;			//Contador de pulsos WS_F_L
uint16_t ui16ws_f_r=0;			//Contador de pulsos WS_F_R
uint16_t ui16ws_r_l=0;			//Contador de pulsos WS_R_L
uint16_t ui16ws_r_r=0;			//Contador de pulsos WS_R_R
uint32_t g_ui32IntCount = 0; 	//Numeros de veces que hemos entrado en la interupcion
uint32_t g_ui32Msg1Count = 0;	//Numeros de veces que hemos enviado el Msg 1
uint32_t g_ui32Msg2Count = 0;	//Numeros de veces que hemos enviado el Msg 2
uint32_t g_ui32Msg3Count = 0;	//Numeros de veces que hemos enviado el Msg 3
bool g_bMsgObj3Sent = 0;		//Flag para indicar fin de la transmision de Msg3
bool g_bErrFlag = 0;			//Error en la transmision
tCANMsgObject CANMsgObject1;	//Objetos de Mensaje CAN variable tipo estructura propia del periferico
tCANMsgObject CANMsgObject2;	//...3 objetos diferentes
tCANMsgObject CANMsgObject3;	//1 por cada mensaje
tCANMsgObject CANMsgObject4;	//3 para Recepcion y 1 para Trasmision
bool RECORD=0;					//Nos indica si el boton de grabacion se ha pulsado
bool INI_OK=0;					//Inicializacion correcta
bool start_write=0;				//Se activa cada 1 ms por interrupcion del timer para la escritura en la SD y captura de datos
bool move=0;					//Variable que nos indica cuando tenemos que mover el log actual a la carpeta ya que el log actual esta en proceso
uint16_t Val_ADC [24];			//Vector donde se almacenan los valores leidos del ADC-> Se inicializara luego
uint32_t time_stamp=0;			//Sello de tiempo en la escritura
uint16_t resul=1;				//Retorno de las funciones de FatFS
uint16_t period=0;				//Valor de carga en el TIMER0B
uint16_t PeriodSysTimer=0;		//Valor de carga en el TIMER1A->Creacion del SysTimer
uint32_t SysTimer=0;			//SystemTimer de nuestra ADQ
TCHAR *file_name; 				//Apunta al primer valor de la cadena de texto que ponemos f_rename
char texto[20];					//Cadena de texto del nuevo nombre de archivo con sprintf
uint16_t numero=0;				//Contaremos las veces que ha escrito una fila de datos para llamar a f_sync
uint8_t nfile=0;				//Nombre dinamicos de los logs antiguos
uint32_t ndatos=0;				//numero de filas escritas en el archivo
void disk_timerproc (void);		//Prototipo de la funcion de ISR de la SD
uint8_t MsgDataTX[8]={0,10,20,30,40,50,60,70};			//Vector que contienes los 8 bytes del msg CAN TX
uint8_t MsgDataRX1[8];			//Vector que contienes los 8 bytes del msg CAN RX
uint8_t MsgDataRX2[8];			//Vector que contienes los 8 bytes del msg CAN RX
uint8_t MsgDataRX3[8];			//Vector que contienes los 8 bytes del msg CAN RX
uint8_t RXFlag,RX1Flag,RX2Flag,RX3Flag=0;



//////// Funciones propias ---------------------------------------------------------------------------------
void ConfigureUART(void){
	    //
	    // Habilita el periferico GPIO del puerto A
	    //
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	    //
	    // Habilita la UART0
	    //
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	    //
	    // Configura los pines para la UART0
	    //
	    GPIOPinConfigure(GPIO_PA0_U0RX);
	    GPIOPinConfigure(GPIO_PA1_U0TX);
	    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	    //
	    // Usamos un reloj interno de 16 Khz para la UART
	    //
	    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	    //
	    // Incializamos la consola 115200 bps
	    //
	    UARTStdioConfig(0, 115200, 16000000);
}

//---------------------------------------------------------------------------------------
//	El primer parametro de entrada es el CS del ADC a leer
//	EL segundo es el canal a leer del ADC
//	La primera trama a mandar esta definida en el datasheet
//	Recibiremos los bits MSB del ADC primero
//---------------------------------------------------------------------------------------
void lectura_ADC(uint8_t CS,uint8_t ch){

	uint8_t addr = 0b00000110 | ((ch & 0b00000100)>>2); //Preparamos trama para la recepcion de datos del ADC segun datasheet
	uint8_t addr_dat = (ch & 0b00000011)<<6;	//Bit de Start, Modo Single, Canal a leer
	uint32_t addr_comp=(addr_dat)|(addr<<8);	//Lectura de los 12 bits del ADC

	if(CS==0){//ADC0
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3,0);
		SSIDataPut(SSI0_BASE, addr_comp); 	//mandamos los primeros 16 datos
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, &data_ADC);	//..y recibimos los siguientes 16 bits
		data_ADC=(data_ADC & 0x000F)<<8;	//nos quedamos con la el nibble de menor peso
		SSIDataPut(SSI0_BASE, 0xFF); 		//mandamos trama vacia
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, & data_ADC_aux);//..y recibimos lo siguientes 16 bits
		data_ADC=data_ADC|(data_ADC_aux & 0xFF00)>>8; //montamos nuestro resultado de 12  bits
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	}
	else if(CS==3){	//ADC1
		GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0);
		SSIDataPut(SSI0_BASE, addr_comp); 	//mandamos los primeros 16 datos
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, &data_ADC);	//..y recibimos los siguientes 16 bits
		data_ADC=(data_ADC & 0x000F)<<8;	//nos quedamos con la el nibble de menor peso
		SSIDataPut(SSI0_BASE, 0xFF); 		//mandamos trama vacia
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, & data_ADC_aux);//..y recibimos lo siguientes 16 bits
		data_ADC=data_ADC|(data_ADC_aux & 0xFF00)>>8; //montamos nuestro resultado de 12  bits
		GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);
	}
	else if(CS==2){	//ADC2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0); //CAMBIAR SS CON UN NUEVO PIN---->!!!!!!!!!!!!!!!!!!!
		SSIDataPut(SSI0_BASE, addr_comp); 	//mandamos los primeros 16 datos
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, &data_ADC);	//..y recibimos los siguientes 16 bits
		data_ADC=(data_ADC & 0x000F)<<8;	//nos quedamos con la el nibble de menor peso
		SSIDataPut(SSI0_BASE, 0xFF); 		//mandamos trama vacia
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, & data_ADC_aux);//..y recibimos lo siguientes 16 bits
		data_ADC=data_ADC|(data_ADC_aux & 0xFF00)>>8; //montamos nuestro resultado de 12  bits
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
	}
	else if(CS==1){	//ADC3
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0); //CAMBIAR SS CON UN NUEVO PIN---->!!!!!!!!!!!!!!!!!!!
		SSIDataPut(SSI0_BASE, addr_comp); 	//mandamos los primeros 16 datos
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, &data_ADC);	//..y recibimos los siguientes 16 bits
		data_ADC=(data_ADC & 0x000F)<<8;	//nos quedamos con la el nibble de menor peso
		SSIDataPut(SSI0_BASE, 0xFF); 		//mandamos trama vacia
		while(SSIBusy(SSI0_BASE));
		SSIDataGet(SSI0_BASE, & data_ADC_aux);//..y recibimos lo siguientes 16 bits
		data_ADC=data_ADC|(data_ADC_aux & 0xFF00)>>8; //montamos nuestro resultado de 12  bits
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
	}
	else{//En el caso de que salte cualquier otro ADC informar
		UARTprintf("No hay ADC\n");
	}

}
//---------------------------------------------------------------------------------------
//	Si ocurre algun error en la SD, bloqueamos el programa.
//---------------------------------------------------------------------------------------
void fatalError(char errMessage[]){ // Quitar el While(1), Y PONER UNA COMPROBACION 10 SEGUNDOS, PARA QUE NO TENGAMOS QUE APAGAR Y ENCENDER EL COCHE PARA PONERLA BIEN 
	UARTprintf(errMessage);	
	//while(1);
}
//---------------------------------------------------------------------------------------
//	Inicializamos el periferico CAN
//	Configuramos la velocidad del CAN Bus y Timming
//	Habilitamos interrupciones del CAN Bus
//	Configuramos los mensajes objeto
//---------------------------------------------------------------------------------------
 uint8_t Ini_CAN(void){

	CANInit(CAN0_BASE);	//Inicializamos el controlador
	CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 250000); //500 kps CAN Bus
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);	//Config interrupciones del controlador
	IntEnable(INT_CAN0);	//Habilitamos las interrupciones del CAN
	CANEnable(CAN0_BASE);	//CAN Bus Operativo

	//Configuracion de los mensajes objeto del CAN Bus
    CANMsgObject1.ui32MsgID = 40;
    CANMsgObject1.ui32MsgIDMask = 40;
    CANMsgObject1.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO;
    CANMsgObject1.ui32MsgLen = 8;	//Longitud del mensaje de 8 bytes

    CANMsgObject2.ui32MsgID = 50;
    CANMsgObject2.ui32MsgIDMask = 50;
    CANMsgObject2.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO;
    CANMsgObject2.ui32MsgLen = 8;	//Longitud del mensaje de 8 bytes

    CANMsgObject3.ui32MsgID = 60;
    CANMsgObject3.ui32MsgIDMask = 60;
    CANMsgObject3.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    CANMsgObject3.ui32MsgLen = 8;	//Longitud del mensaje de 8 bytes
    //----------------------------------------------------------------------------
    CANMsgObject4.ui32MsgID = 20;
    CANMsgObject4.ui32MsgIDMask = 0;
    CANMsgObject4.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    CANMsgObject4.ui32MsgLen = 8;
    CANMsgObject4.pui8MsgData = pui8MsgData;

    //Configuramos el tipo de mensaje
    CANMessageSet(CAN0_BASE, 1, &CANMsgObject1, MSG_OBJ_TYPE_RX);	//Numero de Msg Objeto 1 y tipo RX
    CANMessageSet(CAN0_BASE, 2, &CANMsgObject2, MSG_OBJ_TYPE_RX);	//Numero de Msg Objeto 2 y tipo RX
    CANMessageSet(CAN0_BASE, 3, &CANMsgObject3, MSG_OBJ_TYPE_RX);	//Numero de Msg Objeto 3 y tipo RX-> FIFO???
    UARTprintf("CAN Bus OK\n");

    return 0;

}
 //---------------------------------------------------------------------------------------
 //	Configuramos los timers 1, 2 y 3
 //	Timer0 -> Timer B [Periodic] 16 bits
 //	Timer1 -> Timer A y B [One-Shot] 32 bits
 //	Timer2 -> Timer A [PD0] y Timer B [PD1] 16/16 bits
 //	Timer3 -> Timer A [PD2] y Timer B [PD3]	16/16 bits
 //---------------------------------------------------------------------------------------
 uint8_t Ini_Timer(void){

	 TimerClockSourceSet(TIMER0_BASE,TIMER_CLOCK_SYSTEM); //Fijamos reloj del timer como el Clk del micro
	 TimerClockSourceSet(TIMER1_BASE,TIMER_CLOCK_SYSTEM); //Fijamos reloj del timer como el Clk del micro
	 TimerClockSourceSet(WTIMER2_BASE,TIMER_CLOCK_SYSTEM); //Fijamos reloj del timer como el Clk del micro
	 TimerClockSourceSet(WTIMER3_BASE,TIMER_CLOCK_SYSTEM); //Fijamos reloj del timer como el Clk del micro

	 TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC));								//Half-Width Timers, 16 bits Periodico (Timer B)
	 TimerConfigure(TIMER1_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC));	
	 TimerConfigure(WTIMER2_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT_UP |TIMER_CFG_B_CAP_COUNT_UP));	//Half-Width Timers, Edge-Count(Timer A) y Edge-Count(Timer B)
	 TimerConfigure(WTIMER3_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT_UP |TIMER_CFG_B_CAP_COUNT_UP));	//Half-Width Timers, Edge-Count(Timer A) y Edge-Count(Timer B)

	 TimerControlEvent(WTIMER2_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);	//Se contaran los flancos de subida de los WS
	 TimerControlEvent(WTIMER3_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);	//Se contaran los flancos de subida de los WS

	 period = (SysCtlClockGet() / 1000);				//Interrupcion cada 1 ms
	 PeriodSysTimer=(SysCtlClockGet() / 10000);			//interrupcion cada 0.1 ms

	 TimerLoadSet(TIMER0_BASE, TIMER_B, period);			//Cargamos el valor de cuenta al timer
	 TimerLoadSet(TIMER1_BASE, TIMER_A, PeriodSysTimer);	//Cargamos el valor de cuenta al timer

	 return 0;

 }

//---------------------------------------------------------------------------------------
//	Inicializamos el periferico SPI0
//	Configuramos la velocidad SPI0
//	Ponemos todos los SS en alto(activos en bajo)
//---------------------------------------------------------------------------------------
uint8_t Ini_SPI(void){
	uint8_t i=0;
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 2000000, 16); //Modo 0 SPI, 1Mhz, 16 bits de trama->facilita la creacion de las tramas para ADC
	SSIEnable(SSI0_BASE); //acitvamos SPI
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);	//Ponemos en alto el SS0
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
    UARTprintf("SPI OK\n");

    for(i=0;i<24;i++){ //Inicializamos el vector a 0
    	Val_ADC[i]=0;
    }

    return 0;
}

//---------------------------------------------------------------------------------------
//	Inicializamos la tarjeta SD
//	Montamos la unidad virtual
//	Mostramos un mensaje de confirmacion
//---------------------------------------------------------------------------------------
uint8_t Ini_SD(void){
//Se podria hacer algo asi como que intentara tres veces montar la unidad..
//Si despues de los tres intentos, hay algun error, entonces informa del problema
	switch(f_mount(&FatFs, "", 1)){
			case FR_OK:
				UARTprintf("SD Card mounted successfully\n");
				break;
			case FR_INVALID_DRIVE:
				fatalError("ERROR: Invalid drive number\n");
				break;
			case FR_DISK_ERR:
				fatalError("ERROR: DiskIO error - Check hardware!\n");
				break;
			case FR_NOT_READY:
				fatalError("ERROR: Medium removal or disk_initialize\n");
				break;
			case FR_NO_FILESYSTEM:
				fatalError("ERROR: No valid FAT volume on drive\n");
				break;
			default:
				fatalError("ERROR: Something went wrong\n");
				break;
		}
	resul=f_mkdir("Old_Logs");	//Creamos el sub-directorio(carpeta)
	return 0;

}
//---------------------------------------------------------------------------------------
//	Leeremos siempre los WS obteniendo la cuenta de los timers para no perder ningun pulso
//---------------------------------------------------------------------------------------
void WS_Read(void){

	ui16ws_f_l=TimerValueGet(WTIMER2_BASE,TIMER_A);
	ui16ws_f_r=TimerValueGet(WTIMER2_BASE,TIMER_B);
	ui16ws_r_l=TimerValueGet(WTIMER3_BASE,TIMER_A);
	ui16ws_r_r=TimerValueGet(WTIMER3_BASE,TIMER_B);

}
//---------------------------------------------------------------------------------------
//	Hara los calculos para imprimir la cabecera final con distintos datos
//---------------------------------------------------------------------------------------
void Print_EndFile(void){
	uint8_t seg=0;
	uint8_t min=0;
	uint32_t bytes;
	//Obtenemos la ultima muestra del SysTimer almacenada en time_stamp y sacamos el tiempo de grabacion
	//Formato de tiempo MM:SS
	seg=time_stamp/10000; 	//Pasamos us a seg
	if(seg>=60){			//Pasamos del minuto de grabacion?
	min=seg/60;				//Pasamos a min
	seg=seg%60;				//El resto de la division son los segundos
	}
	f_printf(&logfile,"\n\n//*******************************************************//\n\n");
	f_printf(&logfile,"Quantity of Written Rows:\t%d\n",ndatos);
	f_printf(&logfile,"Record Time(MM:SS) :\t\t%02d:%02d\n",min,seg);	//Rellena con ceros, tamaÃ±o fijo 2
	bytes=f_size(&logfile);
	if(bytes>=1024){
		bytes=bytes/1024;
		f_printf(&logfile,"File size is %d KB(%d bytes)\n\n",bytes,f_size(&logfile));
	}
	else{
	f_printf(&logfile,"File size is %d bytes)\n\n",bytes);
	}
	f_printf(&logfile,"//*******************************************************//\n");
}
//---------------------------------------------------------------------------------------
//	Recibira las tramas de CAN Bus con las IDs correspondientes
//---------------------------------------------------------------------------------------
void CAN_Read(void){
	uint8_t uIdx=0;
	CANMsgObject1.pui8MsgData=MsgDataRX1;
	CANMessageGet(CAN0_BASE, 1, &CANMsgObject1, 0);
	CANMsgObject2.pui8MsgData=MsgDataRX2;
	CANMessageGet(CAN0_BASE, 2, &CANMsgObject2, 0);
	CANMsgObject3.pui8MsgData=MsgDataRX3;
	CANMessageGet(CAN0_BASE, 3, &CANMsgObject3, 0);

	//Procesar el contenido de las tramas
	if(RX1Flag==1){
    UARTprintf("Msg ID=%d\t len=%u\t Data=\t",
    		CANMsgObject1.ui32MsgID, CANMsgObject1.ui32MsgLen);
    for(uIdx = 0; uIdx < CANMsgObject1.ui32MsgLen; uIdx++)
    {
        UARTprintf("%d\t ", MsgDataRX1[uIdx]);
    }
    UARTprintf("\n");
    RX1Flag=0;
	}
	if(RX2Flag==1){
    UARTprintf("Msg ID=%d\t len=%u\t Data=\t",
    		CANMsgObject2.ui32MsgID, CANMsgObject2.ui32MsgLen);
    for(uIdx = 0; uIdx < CANMsgObject2.ui32MsgLen; uIdx++)
    {
        UARTprintf("%d\t", MsgDataRX2[uIdx]);
    }
    UARTprintf("\n");
    RX2Flag=0;
	}
	if(RX3Flag==1){
    UARTprintf("Msg ID=%d\t len=%u\t Data=\t",
    		CANMsgObject3.ui32MsgID, CANMsgObject3.ui32MsgLen);
    for(uIdx = 0; uIdx < CANMsgObject3.ui32MsgLen; uIdx++)
    {
        UARTprintf("%d\t", MsgDataRX3[uIdx]);
    }
    UARTprintf("\n");
    RX3Flag=0;
	}
	RXFlag=0;
}
//---------------------------------------------------------------------------------------
//	Escribira la trama de CAN Bus con la ID correspondiente
//---------------------------------------------------------------------------------------
void CAN_Write(void){
	//Configurar contenido del mensaje-> Ver ejemplo
	//MsgDataTX={Temp_Water_H,Temp_Water_L,Temp_CockPit,Fuel_Level,Oil_P_L,Oil_P_H,Oil_T_L,Oil_T_H};
	//MsgDataTX[8]={0,0,0,0,0,0,0,0};
	//Defines para asignar la etiqueta a la posición del vector de Val_ADC

	CANMsgObject4.pui8MsgData = MsgDataTX;
	CANMessageSet(CAN0_BASE, 4, &CANMsgObject4, MSG_OBJ_TYPE_TX);	//Numero de Msg Objeto 4 y tipo TX-> ENVIAMOS MENSAJE!!
}

//////// RUTINAS DE INTERRUPCION DEL SISTEMA (TABLA DE VECTORES DE EXCEPCION DINAMICA)--------------------------

//---------------------------------------------------------------------------------------
//	ISR de la pulsacion del boton (actuara como boton de grabacion). Ademas, inciara el SysTimer de la grabacion
//---------------------------------------------------------------------------------------
void botonIntHandler (void){
	uint32_t	PF_IFG=0; //Captura de los flags
	PF_IFG = GPIOIntStatus(ButtonBase,true);    //capturamos el valor de los flags de interrupciones al entrar en la int
	GPIOIntClear(ButtonBase,REC_But_Int); //limpiamos los flags de interrupcion

	if((PF_IFG & REC_But_Int)==REC_But_Int){
		if(RECORD==0){	//POR NIVEL Y NO POR FLANCO UNA POSICION DE SUBIDA(ON) Y BAJADA(OFF)
		RECORD=1;	//Ativacion de banderas para la grabacion de datos en la SD
		}
		else{
			RECORD=0;
		}

	}
	    //SysCtlDelay(7000000); //Debouncing boton
	    //Dejaremos listo el directorio raiz con un nuevo archivo a escribir
	    //Tambien reseteamos el Timer
	    if(RECORD==0){
	    	//"move" nos indica que tenemos que mover el log actual a  la carpeta de logs antiguos
	    	//Es decir que hemos hecho la secuencia con el Switch de Record	OFF(Inicio)-> ON-OFF-ON
	    	//Si solo hacemos OFF(Inicio)-> ON-OFF  dejariamos el log ultimo fuera de la carpeta
	    	//Facilitariamos el trabajo a la persona que examina la SD
	    	move=1;
			numero=0;
	    	UARTprintf("FIN GRABACION\n");
			Print_EndFile();	//Imprimimos info util al comienzo del archivo
			SysTimer=0;
			ndatos=0;
			f_close(&logfile);							// Cerramos archivo, guardando los datos hasta el momento;
	    	TimerDisable(TIMER1_BASE,TIMER_A);
	    	TimerLoadSet(TIMER1_BASE, TIMER_A, PeriodSysTimer);	//Cargamos el valor de cuenta al timer

	    }
	    else if(RECORD==1 & move==1){
	    	nfile++;
	    	numero=0;
	    	sprintf(texto,"Old_Logs/log%d.txt", nfile);
	    	file_name = (TCHAR *)texto;
	    	f_rename("RECORD_LOG.txt", file_name);
	    	resul=f_open(&logfile, "RECORD_LOG.txt", FA_WRITE | FA_OPEN_ALWAYS);	//Abrimos el archivo - Si no exite, lo crea
	    	//if(resul!=FR_OK) while(1);
	    	TimerDisable(TIMER1_BASE,TIMER_A);
	    	TimerLoadSet(TIMER1_BASE, TIMER_A, PeriodSysTimer);	//Cargamos el valor de cuenta al timer
	    	move=0;	//Limpiamos bandera de meter en carpeta el log reciente
	    	UARTprintf("Moviendo log mas reciente...\n");
	    	UARTprintf("Grabando...\n");
		    TimerEnable(TIMER1_BASE,TIMER_A);	//Comienza la cuenta
		    f_printf(&logfile,"TIME\tEXT_FL\tEXT_FR\tEXT_RL\tEXT_RR\tAX\tAY\tAZ\tFRE_F\tFRE_R\tGI_V\tG1_P\tG2_P\tG3_P\tG4_P\t"
		    				"G1_T\tG2_T\tG3_T\tG4_T\tG5_T\tG6_T\tTEMP_OIL\tPRES_OIL\tT_COCKPIT\t"
		    				"FU_LV\tWS_FL\tWS_FR\tWS_RL\tWS_RR\tTIMF\n"); //Imprimimos leyenda de las columnas
	    }
	    else if(RECORD==1){
	    	resul=f_open(&logfile, "RECORD_LOG.txt", FA_WRITE | FA_OPEN_ALWAYS);	//Abrimos el archivo - Si no exite, lo crea
	    	//if(resul!=FR_OK) while(1);
	    	UARTprintf("Grabando...\n");
		    TimerEnable(TIMER1_BASE,TIMER_A);	//Comienza la cuenta
		    f_printf(&logfile,"TIME\tEXT_FL\tEXT_FR\tEXT_RL\tEXT_RR\tAX\tAY\tAZ\tFREN_F\tFREN_R\tG_V\tG1_P\tG2_P\tG3_P\tG4_P\t"
		    				"G1_T\tG2_T\tG3_T\tG4_T\tG5_T\tG6_T\tTEMP_OIL\tPRES_OIL\tT_COCKPIT\t"
		    				"FU_LV\tWS_FL\tWS_FR\tWS_RL\tWS_RR\tTIMF\n"); //Imprimimos leyenda de las columnas
	    }
	    else{
	    	UARTprintf("Algo fue mal!!\n");
	    }
}
//---------------------------------------------------------------------------------------
//	ISR para la cuenta de los WS (cada vez que se produzca un flanco de subida en algunos de los puertos)
//---------------------------------------------------------------------------------------
void wsIntHandler (void){
	uint32_t	PD_IFG=0; //Captura de los flags
	PD_IFG = GPIOIntStatus(WS_BASE,ws_f_l_int|ws_f_r_int|ws_r_l_int|ws_r_r_int);    //capturamos el valor de los flags de interrupciones al entrar en la int
	GPIOIntClear(WS_BASE,ws_f_l_int|ws_f_r_int|ws_r_l_int|ws_r_r_int); //limpiamos los flags de interrupcion

	if((PD_IFG & ws_f_l_int)==ws_f_l_int){
		ui16ws_f_l++;
	}
	if((PD_IFG & ws_f_r_int)==ws_f_r_int){
		ui16ws_f_r++;
	}
	if((PD_IFG & ws_r_l_int)==ws_r_l_int){
		ui16ws_r_l++;
	}
	if((PD_IFG & ws_r_r_int)==ws_r_r_int){
		ui16ws_r_r++;
	}
}
//---------------------------------------------------------------------------------------
//	ISR para la SD
//---------------------------------------------------------------------------------------
void SysTickHandler(void)
{
	disk_timerproc();
}

//---------------------------------------------------------------------------------------
//	ISR del Timer 0
//	Activamos bandera de escritura y/o mandar mensajes CAN Bus
//---------------------------------------------------------------------------------------
void Timer0IntHandler (void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT); //Liampiamos la bandera de interrupcion
	start_write=1;	//Habilitamos la escritura/envio
}
//---------------------------------------------------------------------------------------
//	ISR del Timer 1
//	Generara nuestra base de tiempos
//---------------------------------------------------------------------------------------
void Timer1IntHandler (void){
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //Liampiamos la bandera de interrupcion
	SysTimer++;	//Incrementamos la base de tiempos del sistema
}

//---------------------------------------------------------------------------------------
//	ISR CAN BUS (Debug de erorres posibles y activacion de flags)
//---------------------------------------------------------------------------------------
void CANIntHandler(void){
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }
    //
    // Check if the cause is message object 1, which is used for sending
    // message 1.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg1Count++;
        RXFlag=1;
        RX1Flag=1;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 2, which is used for sending
    // message 2.
    //
    else if(ui32Status == 2)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 2, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg2Count++;
        RXFlag=1;
        RX2Flag=1;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    //
    // Check if the cause is message object 2, which is used for sending
    // message 2.
    //
    else if(ui32Status == 3)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 2, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 3);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg2Count++;
        RXFlag=1;
        RX3Flag=1;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 3, which is used for sending
    // messages 3 and 4.
    //
    else if(ui32Status == 4)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 3, and a message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 4);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg3Count++;

        //
        // Set the flag indicating that a message was sent using message
        // object 3.  The program main loop uses this to know when to send
        // another message using message object 3.
        //
        g_bMsgObj3Sent = 1;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}



//////// Programa principal ----------------------------------------------------------------------------------------------
 int main(void){

	int8_t i=0;
	int8_t j=0;
	int8_t z=0;
	FPULazyStackingEnable();
	//Reloj a 80Mhz-->PLL(200Mhz) a traves osc de 16Mhz con un divisor de /5=>(SYSCTL_SYSDIV_5)
	  SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	//Inicializamos las UART
	  ConfigureUART();
	  UARTprintf("//**********************************************//\n");
	  UARTprintf("//                  ADQ_1.0                     //\n");
	  UARTprintf("//**********************************************//\n");
	  UARTprintf("\n");


	//Habilitamos los perifericos
	  SysCtlPeripheralEnable(WS_PERIPH);			//Puerto donde estan conectados los WS
	  SysCtlPeripheralEnable(Button_PERIPH);		//Puerto donde esta mapeado fisicamente el boton
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);	//Habilitamos el periferico del CAN0
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);	//Habilitamos el periferico de SPI0
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	//Puerto donde estan conectados los SS de los ADC y seÃ±ales SPI
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	//Puertos del CAN0 para RX y TX
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	//Habilitamos el perifÃ©rico del Timer 1 (Periodico)
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	//Habilitamos el perifÃ©rico del Timer 1 (Sello de tiempo grabacion)
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);	//Habilitamos el perifÃ©rico del Timer 1 (Sello de tiempo grabacion)
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);	//Habilitamos el perifÃ©rico del Timer 2	(Contador WS)
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);	//Habilitamos el perifÃ©rico del Timer 3	(Contador WS)
	  SysCtlDelay(5);								//espera recomendada de 5 ciclos

	  SysTickPeriodSet(SysCtlClockGet() / 100); //Interrupcion cada 10ms como dice la liberia
	  SysTickEnable();
	  SysTickIntEnable();

	//Configuramos los pines

	  //*****WS*****
#ifndef	WS_INT
	  GPIOPinConfigure(GPIO_PD0_WT2CCP0);//Configuramos los pines de los WS como entrada para el edge count de los timer
	  GPIOPinConfigure(GPIO_PD1_WT2CCP1);
	  GPIOPinConfigure(GPIO_PD2_WT3CCP0);
	  GPIOPinConfigure(GPIO_PD3_WT3CCP1);
	  GPIOPinTypeTimer(WS_BASE,ws_f_l|ws_f_r|ws_r_l|ws_r_r);	//Habilitamos que funcionen para los timers
#else
	  GPIOPinTypeGPIOInput(WS_BASE,ws_f_l|ws_f_r|ws_r_l|ws_r_r);
#endif
//

	  //*****ADC*****
	  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6); //Habilitar el resto de pines de SS segun TABLA
	  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_3);
	  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);
	  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_2);
	  GPIOPinConfigure(GPIO_PA4_SSI0RX);  	//Señal MISO
	  GPIOPinConfigure(GPIO_PA2_SSI0CLK); 	//Señal SCLK
	  GPIOPinConfigure(GPIO_PA5_SSI0TX);	//Señal MOSI
	  GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_4| GPIO_PIN_2); //habilitamos la configuracion para los puertos SPI resistencia Pull_up control por HW
	  	  //Configuramos los SS de la SPI0
	  GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);//habiltamos como pull-up el SS0
	  GPIODirModeSet(GPIO_PORTA_BASE,GPIO_PIN_3, GPIO_DIR_MODE_OUT);//Decimos que el control lo llevaremos por SW en modo salida

	  GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);//habiltamos como pull-up el SS1
	  GPIODirModeSet(GPIO_PORTA_BASE,GPIO_PIN_6, GPIO_DIR_MODE_OUT);//Decimos que el control lo llevaremos por SW en modo salida

	  GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);//habiltamos como pull-up el SS2
	  GPIODirModeSet(GPIO_PORTE_BASE,GPIO_PIN_1, GPIO_DIR_MODE_OUT);//Decimos que el control lo llevaremos por SW en modo salida

	  GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);//habiltamos como pull-up el SS3
	  GPIODirModeSet(GPIO_PORTE_BASE,GPIO_PIN_2, GPIO_DIR_MODE_OUT);//Decimos que el control lo llevaremos por SW en modo salida

	  //*****CAN BUS*****
	  GPIOPinConfigure(GPIO_PE4_CAN0RX); 	//PE4 como CAN RX
	  GPIOPinConfigure(GPIO_PE5_CAN0TX);	//PE5 como CAN TX
	  GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5); //Habilitamos los puertos para CAN

	  //*****SD*****
	  //¡¡¡¡¡Lo perifericos y recursos que necesita el modulo y la libreria SD se encuentran en diskio.c!!!!!!
	  //¡¡¡¡¡ HE CAMBIADO LA LIBRERIA PARA UTILIZAR LA SSI2 en vez de la SSI0----> Mirar TABLA DE PINES !!!!!!

	  //*****RECORD BUTTON Y LUZ DE FRENO (AVERIAS)*****
	  GPIOPinTypeGPIOInput(ButtonBase,REC_But);		//Habilitamos PB2 como entrada del boton de grabacion del salpicadero
	  //GPIOPadConfigSet(ButtonBase ,REC_But,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);
//	  GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_3);	//Habilitamos PB3 como salida para activar la luz de freno en caso de averÃ­as
															//Para valores >400 activamos luz de freno


	//Config de Interrupciones
#ifdef WS_INT
	  IntEnable(INT_GPIOD);
#endif
	  IntEnable(INT_GPIOB);		//Habilitamos Interrupcion del periferico
	  IntEnable(INT_TIMER0B);
	  IntEnable(INT_TIMER1A);

	  IntPrioritySet(FAULT_SYSTICK,0);	//Prioridad en la interrupciones
	  IntPrioritySet(INT_TIMER1A,0x2);
	  IntPrioritySet(INT_GPIOB,0x5);

	//Configuramos los pines de interrupciones
	  GPIOIntDisable(ButtonBase,REC_But_Int);
#ifdef	WS_INT
	  GPIOIntDisable(WS_BASE,ws_f_l_int|ws_f_r_int|ws_r_l_int|ws_r_r_int);
#endif
	  //Limpiamos flags de int
	  GPIOIntClear(ButtonBase,REC_But_Int);
#ifdef	WS_INT
	  GPIOIntClear(WS_BASE,ws_f_l_int|ws_f_r_int|ws_r_l_int|ws_r_r_int);
#endif
	  //Configuracion de la interrupcion
#ifdef	WS_INT
	  GPIOIntTypeSet(WS_BASE,ws_f_l|ws_f_r|ws_r_l|ws_r_r,GPIO_RISING_EDGE); //Flanco de bajada
#endif
	  GPIOIntTypeSet(ButtonBase,REC_But_Int,GPIO_BOTH_EDGES); 				//Flanco de subida y bajada
	  //IE flags
	  GPIOIntEnable(ButtonBase, REC_But_Int); //IE en el pin4 (boton)
#ifdef	WS_INT
	  GPIOIntEnable(WS_BASE, ws_f_l_int|ws_f_r_int|ws_r_l_int|ws_r_r_int); //IE de los pines del WS
#endif
	  TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);	//Int Time-out Timer B del Timer 0
	  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);	//Int Time-out Timer B del Timer 0
	  //Habilitamos interrupciones globales
	  IntMasterEnable();

	// Configuramos los perifericos y recursos
	  Ini_CAN();
	  //Ini_SPI();
	  Ini_Timer();	//Configuramos los timers que necesitemos
	  Ini_SD();
	  //Las funciones ini devuelvan 0 si se ha configurado correctamente y un valor distinto de 0 si no
	  //Asi podemos poner una condicion final que active un bandera de que toda la incializacion fue exitosa(CODIGO DE COLORES)
	  //Si no es asi, escribe un texto en la SD y envia a la telemetria (CAN BUS) un mensaje de error->Entra en un bucle infinito que no se sale hasta que se resetea el micro
	  /*if(Ini_CAN!=0 || Ini_SPI!=0 || Ini_SD!=0 || Ini_TImer!=0){
		  //Mensajes de errores
		  //while(1); //Entramos en un bucle infinito
	  }
	  else{*/
	  	  //start_write=1;
		  INI_OK=1;
		  TimerEnable(TIMER0_BASE,TIMER_B);
#ifndef	WS_INT
		  TimerEnable(WTIMER2_BASE,TIMER_BOTH);
		  TimerEnable(WTIMER3_BASE,TIMER_BOTH);
#endif

	while(INI_OK){

		if(start_write==1){	//En la int del TIMER A, capturaremos el valor de los 4 WS que se hayan registrado hasta ese momento.
			numero++;
		#ifndef	WS_INT
			WS_Read();
		#endif

			//Leemos el ADC, guardamos los valores que nos interesan y preparamos el mensaje a enviar por CAN BUS cada X tiempo (Interrupcion de timer)
			//Si RECORD=1, escribiremos todos lo valores leidos del ADC y WS en la SD
			//Si la variable start_write=1 (int TIMER B), capturaremos los datos de los ADC en sendos vectores aux para cada ADC
				for(j=0;j<3;j++){				//A traves de dos bucles anidados
					for(i=0;i<8;i++){
						//lectura_ADC(j,i);
						Val_ADC[z++]=(uint16_t)data_ADC;	//Ir metiendo los valores de data_ADC en un vector int16 [23]
						if(z==24){
							z=0;
						}
					}
				}
			//UARTprintf("%d\t%d\t%d\t%d\n",ui16ws_f_l,ui16ws_f_r,ui16ws_r_l,ui16ws_r_r);

			//Leemos CAN Bus
				if(RXFlag==1){
				CAN_Read();
				}
			//Mandar mensajes por CAN Bus
				if(numero==2000){
				CAN_Write();
				numero=0;
				}

				if(RECORD==1){	//Escribiremos en la SD el valor del SysTimer, los 4 WS + valores ADC
					//Capturamos valores de los timers
						numero++;	//variable se incrementa para saber cuandi hacer el guardado de datos
						ndatos++;
						time_stamp=SysTimer;//Capturamos valor del systimer
						//LIBRERIA SD->PARA EL USO DE PRINTF Y EN GENERAL FUNCIONES DE STRING SE HA CAMBIADO EL PARAMETRO _USE_STRFUNC A UN VALOR DE 2
						//Es 2 para que convierta el \n en \r+\n
						//resul=f_open(&logfile, "RECORD_LOG.txt", FA_WRITE | FA_OPEN_ALWAYS);	//Abrimos el archivo - Si no exite, lo crea
						//if(resul== FR_OK){
							f_lseek(&logfile, f_size(&logfile));		// Move forward by filesize; logfile.fsize+1 is not needed in this application
							//f_printf(&logfile, "%d\t",numero);	//Imprimimos el sello de tiempo de la escritura
							f_printf(&logfile, "%u\t",time_stamp);	//Imprimimos el sello de tiempo de la escritura
							f_printf(&logfile,"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",ADC1);
							f_printf(&logfile,"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",ADC2);
							f_printf(&logfile,"%d\t%d\t%d\t%d\t%d\t\t%d\t\t%d\t\t%d\t",ADC3);
							f_printf(&logfile,"%u\t %u\t %u\t %u\t",ui16ws_f_l,ui16ws_f_r,ui16ws_r_l,ui16ws_r_r);	//Imprimimos el sello de tiempo de la escritura
							f_printf(&logfile,"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",MsgRX1);
							f_printf(&logfile,"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",MsgRX2);
							f_printf(&logfile,"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",MsgRX3);
							time_stamp=SysTimer;	//Capturamos valor final para saber tiempo de escritura
							f_printf(&logfile, "%u\t",time_stamp);	//Imprimimos el sello de tiempo de la escritura
							f_printf(&logfile, "\n");				//Acabamos la escritura de la fila de datos
							/*if(RECORD==0){
								//Aqui iria la cabecera final
								//que sera una funcion que llamaremos
						    	//Seria interesante imprimir una cabezera final con el tamaÃ±o del archivo grabado y el tiempo del mismo pasado a min y segundos
						    	//La fecha estara en la cabecera, al igual que el lugar y la hora de comience de test
								Print_EndFile();	//Imprimimos info util al comienzo del archivo
								SysTimer=0;
								f_close(&logfile);							// Cerramos archivo, guardando los datos hasta el momento;
							}*/
							if(numero==30000){
								//Por seguridad salvaremos los datos cada 3000 filas escritas en el log
								//la diferencia entre f_sync y f_close es que el primero salva los datos y abre de nuevo el archivo para seguir escribiendo
								//f_close cierra el archivo y lo deshabilita hasta que no se usa de nuevo f_open
								resul=f_sync(&logfile);
								//if(resul!=FR_OK) while(1);	//imprimir mensaje de error en la SD ya que si 3 segundos no se graba en una endurance no pasa na
								numero=0;	//reseteamos nuetro testigo
							}
						}
					}
		start_write=0; 			//Limpiamos la bandera de escritura que solo la activa la int de time-out
	}
}
