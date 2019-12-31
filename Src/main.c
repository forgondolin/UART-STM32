/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_VARRE 5             // inc varredura a cada 5 ms (~200 Hz)
#define DIGITO_APAGADO 0x10    // kte valor p/ apagar um dígito no display
#define DT_RESPOSTA 100        // delay do pisca para os leds, aguardando resposta na UART
#define DT_BUZZER 200          // delay do buzzer
#define DT_LOC 250          	// Intervalo de tempo entre as leituras locais
#define DT_REQ 200				// Intervalo de tempo entre as leituras remotas
#define DT_MIN 1000

#define BYTES_RX 4 // RX tem 4 bytes
#define BYTES_TX 4 // TX tem 4 bytes
#define BYTES_REQ 4 // Bytes de informação
#define BYTES_DADOS 4 //

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t buffer_rx[BYTES_RX]; // Armazena os bytes vindos remotamente
uint8_t buffer_tx[BYTES_TX]; // Armazena os bytes da leitura analogica a serem transmitidos
uint8_t buffer_request[BYTES_REQ]; // Envia um frame de requisição
uint8_t buffer_dados[BYTES_DADOS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void set_modo_oper(int);             // seta modo_oper (no stm32f1xx_it.c)
void set_modo_relogio(int m);
void set_horas(int m);
void set_minutos(int m);
int get_modo_oper(void);             // obtém modo_oper (stm32f1xx_it.c)
int get_modo_relogio(void);
int get_horas(void);
int get_minutos(void);

// funcoes do arquivo prat_05_funcoes.c
void reset_pin_GPIOs(void);         // reset pinos da SPI
void serializar(int ser_data);       // prot fn serializa dados p/ 74HC595
int16_t conv_7_seg(int NumHex);      // prot fn conv valor --> 7-seg

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t val_adc = 0;                 // var global: valor lido no ADC
int16_t val_adc_remoto = 0;

// Variáveis de controle da transmissão/recepção...
uint8_t flagTransmit = 0;
uint8_t flagResposta = 0;
uint8_t flagReceived = 0;
uint8_t flagDisparaRx = 0;
uint8_t flagFalha = 0;
uint8_t flagErro = 0;
uint8_t adcRdy = 0;

static enum {
	INIT_ADC, CONVERTE_ADC
} sttADC;

static enum {
	REQ_INIT, VERIF_RECV, CONVERTE_RQ, REQ_ERRO, REQ_DEINIT
} sttReq;

static enum {
	LEDS_IDLE, INIT_LEDS, PISCA_LEDS
} sttLEDS_UART;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// vars e flags de controle do programa no superloop...
	int milADC = 0,                      // ini decimo de seg
			cenADC = 0,                      // ini unidade de seg
			dezADC = 0,                      // ini dezena de seg
			uniADC = 0,                      // ini unidade de minuto
			relogMin = 0,					// ini relogio min
			relogHrs = 0;					// ini relog seg

	int16_t val7seg = 0x00FF,            // inicia 7-seg com 0xF (tudo apagado)
			serial_data = 0x01FF;           // dado a serializar (dig | val7seg)
	uint32_t miliVolt = 0x0,             // val adc convertido p/ miliVolts
			tIN_varre = 0, 				  // registra tempo última varredura
			tIN_leitura = 0, //armazena o instante que a leitura ADC foi disparada
			tIN_leds = 0, // armazena o instante em que os leds começaram a piscar
			tIN_buzzer = 0, //armazena o instante em que o buzzer começou a tocar
			tIN_Relog = 0,		//armazena o instante em que um segundo passou.
			tIN_Leitura_Remota = 0, //armazena o instante em que o REQ da UART foi disparado (Tx)
			tIN_Press_Btn = 0, //armazena o instante em que o botão de reset foi pressionado.
			tIN_Busy_Rx = 0;

	static uint16_t checksum_rx = 0;
	static uint16_t checksum_tx = 0;
	static int modo_op;
	static int modo_relog;
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
	MX_ADC1_Init();
	MX_USART1_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	// Reseta todos os LEDs e o Buzzer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

	// inicializa a SPI (pinos 6,9,10 da GPIOB)
	reset_pin_GPIOs();

	//Carrega o buffer_request com o frame de requisição
	buffer_request[0] = 'r';
	buffer_request[1] = 'r';
	buffer_request[2] = 'r';
	buffer_request[3] = 'r';	// Envia o frame p/ servir de referencia

	// Pontapé inicial para ativar a interrupt da UART.
	HAL_UART_Receive_IT(&huart1, buffer_rx, BYTES_RX);

	// var de estado que controla a varredura (qual display é mostrado)
	static enum {
		DIG_UNI, DIG_DEC, DIG_CENS, DIG_MILS
	} sttVARRE = DIG_UNI;

	static enum {
		RESET_IDLE, RESET_WAIT
	} sttRST;

	static enum {
		RX_TIMEOUT_IDLE, RX_TIMEOUT_WAIT, AGUARDA_ERRO_SAIR
	} sttRXTIMEOUT;

	// var de estado dos piscas e do buzzer
	static enum {
		DEINIT_ALARME, //estado referente à desinicialização do alarme
		INIT_BUZZER,   //inicialização do buzzer
		PISCA_BUZZER   //controle do pisca do buzzer
	} sttALARME;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// Pega o modo de operação e o modo de relógio pra usar uma vez no loop
		modo_op = get_modo_oper();
		modo_relog = get_modo_relogio();

		// tarefa #0: Resetar o programa se o botão for pressionado por 3s
		switch (sttRST) {
		case RESET_IDLE:
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0) {
				tIN_Press_Btn = HAL_GetTick();
				sttRST = RESET_WAIT;
			}
			break;
		case RESET_WAIT:
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 1) {
				sttRST = RESET_IDLE;
			}
			if (HAL_GetTick() - tIN_Press_Btn >= 3000) {
				NVIC_SystemReset();
			}
		}

		// tarefa #1: Controle simples do relógio
		// Relogio não pára mesmo em modo de setup
		// Em modo simples, apenas incrementa as variáveis
		if (HAL_GetTick() - tIN_Relog >= DT_MIN) {
			tIN_Relog = HAL_GetTick();
			relogMin++;

			if (relogMin == 60) {
				relogHrs++;
				relogMin = 0;
			}

			if (relogHrs == 24) {
				relogHrs = 0;
			}
		}

		// Tarefa #2: Atualização dos valores setados no relógio
		// Quando o modo do relógio chega a 3, pega os valores e zera o modo do relógio
		if (modo_relog == 3) {
			relogMin = get_minutos();
			relogHrs = get_horas();

			set_modo_relogio(0);
		}

		// Tarefa #3.1: Responde requisição da leitura analógica
		// Se o flagTransmit for true, transmite o valor no ADC na UART
		if (flagDisparaRx) {
			// Reseta o flag transmit

			//Transmite o valor ADC para quem pediu
			if (huart1.RxState != HAL_UART_STATE_BUSY_RX) { // Não está busy, mas vai ficar...
				flagDisparaRx = 0;
				HAL_UART_Receive_IT(&huart1, buffer_rx, sizeof(buffer_rx));
				tIN_Busy_Rx = HAL_GetTick();
			}
		}

		// Tarefa #3.2: Responde requisição da leitura analógica
		// Se o flagTransmit for true, transmite o valor no ADC na UART
		if (flagTransmit && adcRdy) {
			//Transmite o valor ADC para quem pediu
			if (huart1.gState != HAL_UART_STATE_BUSY_TX) {
				// Reseta o flag transmit e envia o dado
				flagTransmit = 0;
				HAL_UART_Transmit_IT(&huart1, buffer_tx, sizeof(buffer_tx));
			}
		}

		// Tarefa #3.3: Verifica os dados que chegaram na UART
		if (flagResposta) {
			flagResposta = 0;

			checksum_rx = buffer_dados[0] + buffer_dados[1]; //checksum recebido
			// verif. o checksum transmitido
			checksum_tx = (uint16_t) (buffer_dados[2] << 8)
					| (uint16_t) buffer_dados[3];

			if (checksum_rx == checksum_tx) { // Se for igual, valida os dados e coloca no adc
				val_adc_remoto = (buffer_dados[0] << 8) | buffer_dados[1];
				flagReceived = 1;
			} else { // Se não for igual, pede novamente os dados (vai disparar sozinho)
				flagErro = 1;
			}
		}

		// Tarefa #4: Controle do ADC
		switch (sttADC) {
		case INIT_ADC: // dispara por software uma conversão ADC a cada 250ms
			if (HAL_GetTick() - tIN_leitura >= DT_LOC) {
				HAL_ADC_Start_IT(&hadc1); // dispara ADC p/ conversão por IRQ
				tIN_leitura = HAL_GetTick();
			}
		case CONVERTE_ADC:
			// converter o valor lido em decimais p/ display se o modo de operação for 1
			if (modo_op == 1) {
				miliVolt = val_adc * 3300 / 4095;
				uniADC = miliVolt / 1000;
				dezADC = (miliVolt - (uniADC * 1000)) / 100;
				cenADC = (miliVolt - (uniADC * 1000) - (dezADC * 100)) / 10;
				milADC = miliVolt - (uniADC * 1000) - (dezADC * 100)
						- (cenADC * 10);
			}
			sttADC = INIT_ADC; //retorna ao modo de op. 1 para disparar o adc
		}

		// tarefa #5: Se (modo_oper=2) pede o valor analógico na serial
		if (modo_op == 2) {
			// Verifica se deu timeout no Rx
			/*switch (sttRXTIMEOUT) {
			case RX_TIMEOUT_IDLE: // Aguarda o strobe do Rx e renova
				if (HAL_GetTick() - tIN_Busy_Rx >= 5 * DT_REQ) { // Dispara o erro
					sttReq = REQ_ERRO;
					sttRXTIMEOUT = AGUARDA_ERRO_SAIR;
				}
				break;
			case AGUARDA_ERRO_SAIR:
				if (HAL_GetTick() - tIN_Busy_Rx < 5 * DT_REQ) { // Aguarda o erro sair
					sttReq = REQ_INIT;
					sttRXTIMEOUT = RX_TIMEOUT_IDLE;
				}
				break;
			}*/

			switch (sttReq) { // Requisita o t_req uma vez a cada 200ms
			case REQ_INIT:
				if (HAL_GetTick() - tIN_Leitura_Remota >= DT_REQ) {
					// Envia requisição p/ UART se tiver livre
					if (huart1.gState != HAL_UART_STATE_BUSY_TX) {
						tIN_Leitura_Remota = HAL_GetTick();
						HAL_UART_Transmit_IT(&huart1, buffer_request,
								sizeof(buffer_request));
						// Inicializa os LEDs de indicação p/ avisar que está aguardando
						//sttLEDS_UART = INIT_LEDS;

						// Para para o passo de verificação da comunicação
						sttReq = VERIF_RECV;
					}
				}
				break;
			case VERIF_RECV:
				// Se recebeu, passa para o próximo passo...
				if (flagReceived) {
					flagReceived = 0;
					sttReq = CONVERTE_RQ;
					sttLEDS_UART = LEDS_IDLE;
					flagFalha = 0;
				}

				if (flagErro) { // Se deu erro no checksum, pede de novo o pacote
					flagErro = 0;
					sttReq = REQ_INIT;
					//sttLEDS_UART = LEDS_IDLE;
					//flagFalha = 0;
				}
				// Verifica continuamente se houve timeout
				if ((HAL_GetTick() - tIN_Leitura_Remota) > 5 * DT_REQ) {
					sttReq = REQ_ERRO;
				}
				break;
			case CONVERTE_RQ:
				// converter o valor lido em decimais p/ display
				//val_adc_remoto = buffer_rx[1] * buffer_rx[0];

				miliVolt = val_adc_remoto * 3300 / 4095;
				uniADC = miliVolt / 1000;
				dezADC = (miliVolt - (uniADC * 1000)) / 100;
				cenADC = (miliVolt - (uniADC * 1000) - (dezADC * 100)) / 10;
				milADC = miliVolt - (uniADC * 1000) - (dezADC * 100)
						- (cenADC * 10);

				sttReq = REQ_INIT; //retorna ao modo de op. 1 para disparar o adc

				break;
			case REQ_ERRO:
				// Tratar o erro... (pisca os LEDs e toca o buzzer)
				flagFalha = 1;
				sttALARME = INIT_BUZZER;
				sttLEDS_UART = INIT_LEDS;
				// Volta para o req init até alguém enviar um pacote de requisição válido
				sttReq = REQ_INIT;

				break;
			case REQ_DEINIT:
				break;
			}
		}

		// tarefa #6: qdo milis() > DELAY_VARRE ms, desde a última mudança
		// Imprime o valor analógico no display de 7 segmentos. (local ou remoto), sem erro
		if (modo_op > 0 && modo_relog == 0
				&& (HAL_GetTick() - tIN_varre) > DT_VARRE && !flagFalha)// se ++0,1s atualiza o display
				{
			tIN_varre = HAL_GetTick(); // salva tIN p/ prox tempo varredura
			switch (sttVARRE) // teste e escolha de qual DIG vai varrer
			{
			case DIG_MILS: {
				sttVARRE = DIG_CENS;           // ajusta p/ prox digito
				serial_data = 0x0008;           // display #1
				val7seg = conv_7_seg(milADC);
				break;
			}
			case DIG_CENS: {
				sttVARRE = DIG_DEC;            // ajusta p/ prox digito
				serial_data = 0x0004;            // display #2
				if (cenADC > 0 || dezADC > 0 || uniADC > 0) {
					val7seg = conv_7_seg(cenADC);
				} else {
					val7seg = conv_7_seg(DIGITO_APAGADO);
				}
				break;
			}
			case DIG_DEC: {
				sttVARRE = DIG_UNI;            // ajusta p/ prox digito
				serial_data = 0x0002;            // display #3
				if (dezADC > 0 || uniADC > 0) {
					val7seg = conv_7_seg(dezADC);
				} else {
					val7seg = conv_7_seg(DIGITO_APAGADO);
				}
				break;
			}
			case DIG_UNI: {
				sttVARRE = DIG_MILS;           // ajusta p/ prox digito
				serial_data = 0x0001;           // display #4
				if (uniADC > 0) {
					val7seg = conv_7_seg(uniADC);
					val7seg &= 0x7FFF;           // liga o ponto decimal
				} else {
					val7seg = conv_7_seg(DIGITO_APAGADO);
				}
				break;
			}
			}  // fim case
			tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
			serial_data |= val7seg; // OR com val7seg = dado a serializar
			serializar(serial_data); // serializa dado p/74HC595 (shift reg)
		}  // -- fim da tarefa #6

		// tarefa #6.5: qdo milis() > DELAY_VARRE ms, desde a última mudança
		// Imprime o valor analógico no display de 7 segmentos. (local ou remoto), COM ERRO
		if (modo_op > 0 && modo_relog == 0
				&& (HAL_GetTick() - tIN_varre) > DT_VARRE && flagFalha)	// se ++0,1s atualiza o display
				{
			tIN_varre = HAL_GetTick(); // salva tIN p/ prox tempo varredura
			switch (sttVARRE) // teste e escolha de qual DIG vai varrer
			{
			case DIG_MILS: {
				sttVARRE = DIG_CENS;           // ajusta p/ prox digito
				serial_data = 0x0008;           // display #1
				val7seg = conv_7_seg('o');
				break;
			}
			case DIG_CENS: {
				sttVARRE = DIG_DEC;            // ajusta p/ prox digito
				serial_data = 0x0004;            // display #2
				val7seg = conv_7_seg('r');
				break;
			}
			case DIG_DEC: {
				sttVARRE = DIG_UNI;            // ajusta p/ prox digito
				serial_data = 0x0002;            // display #3
				val7seg = conv_7_seg('r');
				break;
			}
			case DIG_UNI: {
				sttVARRE = DIG_MILS;           // ajusta p/ prox digito
				serial_data = 0x0001;           // display #4
				val7seg = conv_7_seg('E');
				break;
			}
			}  // fim case
			tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
			serial_data |= val7seg; // OR com val7seg = dado a serializar
			serializar(serial_data); // serializa dado p/74HC595 (shift reg)
		}  // -- fim da tarefa #6

		//tarefa #7: Atualiza a mostragem do relógio. Modo de operação do relógio 0
		// Obs: DIG_MILS -> 4º
		if (modo_op
				== 0&& modo_relog == 0 && (HAL_GetTick() - tIN_varre) > DT_VARRE) // se ++0,1s atualiza o display
				{
			tIN_varre = HAL_GetTick(); // salva tIN p/ prox tempo varredura
			switch (sttVARRE)  // teste e escolha de qual DIG vai varrer
			{
			case DIG_MILS: {
				sttVARRE = DIG_CENS;           // ajusta p/ prox digito
				serial_data = 0x0008;          // display #1
				val7seg = conv_7_seg(relogMin % 10);
				break;
			}
			case DIG_CENS: {
				sttVARRE = DIG_DEC;            // ajusta p/ prox digito
				serial_data = 0x0004;         // display #2
				val7seg = conv_7_seg(relogMin / 10);
				break;
			}
			case DIG_DEC: {
				sttVARRE = DIG_UNI;            // ajusta p/ prox digito
				serial_data = 0x0002;          // display #3
				val7seg = conv_7_seg(relogHrs % 10);
				val7seg &= 0x7FFF;           // liga o ponto decimal
				break;
			}
			case DIG_UNI: {
				sttVARRE = DIG_MILS;           // ajusta p/ prox digito
				serial_data = 0x0001;          // display #4
				val7seg = conv_7_seg(relogHrs / 10);
				break;
			}
			}  // fim case
			tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
			serial_data |= val7seg; // OR com val7seg = dado a serializar
			serializar(serial_data); // serializa dado p/74HC595 (shift reg)
		}  // -- fim da tarefa #7

		//tarefa #8: Atualiza a mostragem do relógio. Modo de operação do relógio 1
		// Mostra apenas os dígitos das horas
		// Obs: DIG_MILS -> 4º Digito
		if (modo_op
				== 0&& modo_relog == 1 && (HAL_GetTick() - tIN_varre) > DT_VARRE) // se ++0,1s atualiza o display
				{
			tIN_varre = HAL_GetTick(); // salva tIN p/ prox tempo varredura
			switch (sttVARRE)  // teste e escolha de qual DIG vai varrer
			{
			case DIG_MILS: {
				sttVARRE = DIG_CENS;           // ajusta p/ prox digito
				serial_data = 0x0008;          // display #1
				val7seg = conv_7_seg(DIGITO_APAGADO);
				break;
			}
			case DIG_CENS: {
				sttVARRE = DIG_DEC;            // ajusta p/ prox digito
				serial_data = 0x0004;         // display #2
				val7seg = conv_7_seg(DIGITO_APAGADO);
				break;
			}
			case DIG_DEC: {
				sttVARRE = DIG_UNI;            // ajusta p/ prox digito
				serial_data = 0x0002;          // display #3
				val7seg = conv_7_seg(get_horas() % 10);
				val7seg &= 0x7FFF;           // liga o ponto decimal
				break;
			}
			case DIG_UNI: {
				sttVARRE = DIG_MILS;           // ajusta p/ prox digito
				serial_data = 0x0001;          // display #4
				val7seg = conv_7_seg(get_horas() / 10);
				break;
			}
			}  // fim case
			tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
			serial_data |= val7seg; // OR com val7seg = dado a serializar
			serializar(serial_data); // serializa dado p/74HC595 (shift reg)
		}  // -- fim da tarefa #8

		//tarefa #9: Atualiza a mostragem do relógio. Modo de operação do relógio 2
		// Mostra todos os dígitos
		// Obs: DIG_MILS -> 4º Digito
		if (modo_op
				== 0&& modo_relog == 2 && (HAL_GetTick() - tIN_varre) > DT_VARRE) // se ++0,1s atualiza o display
				{
			tIN_varre = HAL_GetTick(); // salva tIN p/ prox tempo varredura
			switch (sttVARRE)  // teste e escolha de qual DIG vai varrer
			{
			case DIG_MILS: {
				sttVARRE = DIG_CENS;           // ajusta p/ prox digito
				serial_data = 0x0008;          // display #1
				val7seg = conv_7_seg(get_minutos() % 10);
				break;
			}
			case DIG_CENS: {
				sttVARRE = DIG_DEC;            // ajusta p/ prox digito
				serial_data = 0x0004;         // display #2
				val7seg = conv_7_seg(get_minutos() / 10);
				break;
			}
			case DIG_DEC: {
				sttVARRE = DIG_UNI;            // ajusta p/ prox digito
				serial_data = 0x0002;          // display #3
				val7seg = conv_7_seg(get_horas() % 10);
				val7seg &= 0x7FFF;           // liga o ponto decimal
				break;
			}
			case DIG_UNI: {
				sttVARRE = DIG_MILS;           // ajusta p/ prox digito
				serial_data = 0x0001;          // display #4
				val7seg = conv_7_seg(get_horas() / 10);
				break;
			}
			}  // fim case
			tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
			serial_data |= val7seg; // OR com val7seg = dado a serializar
			serializar(serial_data); // serializa dado p/74HC595 (shift reg)
		}  // -- fim da tarefa #9

		//tarefa #10: atualiza a situação dos LEDs de acordo com o modo de operação
		if (modo_op == 0 && modo_relog == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		} else if (modo_op == 1) { // Modo de operação leitura ADC LOCAL
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		} else if (modo_op == 2) {
			switch (sttLEDS_UART) {
			case LEDS_IDLE:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				break;
			case INIT_LEDS:
				//
				// Garante que todos estão desligados
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				sttLEDS_UART = PISCA_LEDS;
				break;
			case PISCA_LEDS:
				// Garante que todos estão desligados
				if (HAL_GetTick() - tIN_leds >= DT_RESPOSTA) {
					tIN_leds = HAL_GetTick();
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
				}
				break;

			}
		} else if (modo_relog == 1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		} else if (modo_relog == 2) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}

		switch (sttALARME) {
		case DEINIT_ALARME: //Desliga o buzzer
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			break;

		case INIT_BUZZER:
			//Armazena o instante em que o buzzer foi inicializado
			tIN_buzzer = HAL_GetTick();

			//Liga o buzzer
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

			//Desliga os leds
			sttALARME = PISCA_BUZZER;
			break;

		case PISCA_BUZZER:
			// Aguarda o tempo do buzzer para desligá-lo e retornar ao início
			if (HAL_GetTick() - tIN_buzzer >= DT_BUZZER) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			}

			if (modo_op < 2) { // Se o modo
				sttALARME = DEINIT_ALARME;
			}
			break;
		}

	}    // -- fim do loop infinito

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* EXTI1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	/* ADC1_2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	/* EXTI2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	/* EXTI3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
					| GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA1 PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB12 PB13 PB14
	 PB15 PB5 PB6 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
			| GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// fn que atende ao callback da ISR do conversor ADC1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		adcRdy = 0;
		static uint16_t checksum = 0;

		val_adc = HAL_ADC_GetValue(&hadc1); // Armazena o valor adc LOCAL

		if (get_modo_oper() == 1) {
			sttADC = CONVERTE_ADC; // alterou valor lido
		}

		// Copia o adc p/ o buffer_tx
		__disable_irq();
		buffer_tx[0] = (val_adc & 0x0F00) >> 8;
		buffer_tx[1] = val_adc & 0x00FF;

		// Calcula o checksum_tx (falta implementar)
		checksum = (uint16_t) (buffer_tx[0] + buffer_tx[1]);

		// Coloca o checksum_tx no buffer p transmitir
		buffer_tx[2] = (uint8_t) (checksum >> 8);
		buffer_tx[3] = (uint8_t) 0xFF & checksum;
		__enable_irq();

		// Dispara outra leitura
		//HAL_ADC_Start_IT(&hadc1);
		adcRdy = 1; // Esse aqui dispara a outra...
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static uint8_t i = 0;

// Creio que seja necessário fazer um __noInterrupts() pra tornar
// a interrupção thread safe...
	if (huart->Instance == USART1) {
		// Verifica o que tiver que verificar... [CRC, etc..]
		if (buffer_rx[0] == 'r' && buffer_rx[1] == 'r' && buffer_rx[2] == 'r'
				&& buffer_rx[3] == 'r') {
			// Atualiza as flags que precisar atualizar
			flagTransmit = 1;
		} else {
			// se for uma resposta, copia o buffer e habilita o flag
			__disable_irq();
			for (i = 0; i < sizeof(buffer_rx); i++) {
				buffer_dados[i] = buffer_rx[i];
			}
			__enable_irq();
			flagResposta = 1;
		}

		// Dispara outra recepção
		//__HAL_UART_FLUSH_DRREGISTER(&huart1);

		//HAL_UART_Receive_IT(&huart1, buffer_rx, sizeof(buffer_rx));
		flagDisparaRx = 1;
	}
// Desfazer o __nointerrupts()
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
