/* USER CODE BEGIN Header */
/*VILLETTE Lou-Anne & CALDEIRA Quentin*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACC_ADR 		0x32 //Adresse i2c de l'acceloremetre
#define MAG_ADR 		0x3C //Adresse i2c du magnetometre
#define WHO_AM_I_A 		0x0F
#define WHO_AM_I_M		0x4F


#define CTRL_REG1_A 	0x20
#define CTRL_REG5_A 	0x24
#define OUT_X_L_A		0x28

#define CFG_REG_A_M 	0x60
#define	OUTX_L_REG_M	0x68

#define SUB_INCREMENT   0x80//Masque pour autoriser l'incrément des sous registres

#define CTRL_REG_A_M 	0x60

#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct data_meas {
  int16_t X;
  int16_t Y;
  int16_t Z;
};

struct data_real {
  float X;
  float Y;
  float Z;
};

struct angle {
  float theta;
  float psi;
  float phi;
  float delta;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*------------Fonction permettant de relier le printf et l'uart---------------*/
int __io_putchar(int ch) {
	uint8_t c = ch & 0x00FF;
	HAL_UART_Transmit(&huart2, &c, 1, 10);
	return ch;
}

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&huart2);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/*----------------------------------------------------------------------------*/

/*---Fonction permettant d'attester la présence du magneto et de l'accelero---*/
int who_am_i_sensors(){
	uint8_t buf[1];                                                              //Buffer de 1 octet car on ne lit que une case mémoire
	HAL_StatusTypeDef ret;                                                       //Variable HAL permettant de voir l'état de la transmission I2C
	buf[0] = WHO_AM_I_A;                                                         //On affecte l'adresse de la case mémoire stockant WHOAMI pour l'accelero
	ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_ADR, buf, 1, HAL_MAX_DELAY);       //On effectue la transmission sur l'accelero
	if ( ret != HAL_OK ) {                                                       //Si la transmission s'est mal passée, on affiche une erreur
		printf("Error Tx\r\n");
	}
	else {
	    ret = HAL_I2C_Master_Receive(&hi2c1, ACC_ADR, buf, 1, HAL_MAX_DELAY);     //Sinon, on recupere la valeur dans la case memoire
	    if ( ret != HAL_OK ) {                                                    //Si la réception s'est mal passée, on affiche une erreur
	      printf("Error Rx\r\n");
	    }
	    else {
	    	if ( buf[0]==0x33 ) {                                                 //On teste si la valeur présente dans la case memoire est identique à celle indiquée dans la doc
	        printf("Detection de accelerometre \n\r");
	    	}
	    }
  }
  buf[0] = WHO_AM_I_M;                                                             //On affecte l'adresse de la case mémoire du WHOAMI pour le magneto
  ret = HAL_I2C_Master_Transmit(&hi2c1, MAG_ADR, buf, 1, HAL_MAX_DELAY);           //On effectue la transmission sur le magneto
  if ( ret != HAL_OK ) {                                                           //Si la transmission s'est mal passée, on affiche une erreur
	  printf("Error Tx\r\n");
  }
  else {
  	ret = HAL_I2C_Master_Receive(&hi2c1, MAG_ADR, buf, 1, HAL_MAX_DELAY);          //Sinon, on recupere la valeur dans la case memoire
  	if ( ret != HAL_OK ) {                                                          //Si la réception s'est mal passée, on affiche une erreur
  	  printf("Error Rx\r\n");
  	}
    else {
    	if ( buf[0]==0x40 ) {                                                        //On teste si la valeur présente dans la case memoire est identique à celle indiquée dans la doc
    		printf("Detection de magneto \n\r");
    	}
    }
  }
}

int reset_acc(){
	uint8_t buf[2];																	//Buffer de 2 octets car on dirige la mémoire puis on écrit une valeur
	HAL_StatusTypeDef ret;
	buf[0] = CTRL_REG5_A;  															//L'adresse où écrire sera CTRL_REG_5
 	buf[1]=0x80;  																	//Data de reset
	ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_ADR, buf, 2, HAL_MAX_DELAY);			//On teste si la transmission se passe bien
	if ( ret != HAL_OK ) {
		printf("Error Tx\r\n");														//Si elle se passe mal, on met une erreur
	}
}

int config_acc(){
	HAL_StatusTypeDef ret;
	uint8_t buf[6] ;																//Buffer de 6 car on va écrire sur 6 reg d'un coup
	uint8_t res[6] ;																//Buffer permettant de vérifier si les valeurs ont bien été écrites
	buf[0]=0x27;//Valeur a mettre dans ctrm_reg_1
	buf[1]=0x00;//Valeur a mettre dans ctrm_reg_2
	buf[2]=0x00;//Valeur a mettre dans ctrm_reg_3
	buf[3]=0x00;//Valeur a mettre dans ctrm_reg_4
	buf[4]=0x00;//Valeur a mettre dans ctrm_reg_5
	buf[5]=0x00;//Valeur a mettre dans ctrm_reg_6
	ret = HAL_I2C_Mem_Write(&hi2c1, ACC_ADR, CTRL_REG1_A|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		printf("Error Tx\r\n");
	}
	ret = HAL_I2C_Mem_Read(&hi2c1, ACC_ADR, CTRL_REG1_A|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, res, 6, HAL_MAX_DELAY);
	uint8_t i=0;
	//Ici on relit les 6 registres pour vérifier que la valeur souhaitée à bien été inscrite
	for(i=0;i<6;i++){
		if(buf[i]==res[i]){
			printf("0x%02x mis dans le registre CTRL_REG_%d_A \n\r", buf[i], i+1);
		}
		else{
			printf("Valeur mise dans le registre CTRL_REG_%d_A erronee \n\r",i+1);
		}
	}
}

int reset_mag(){
	uint8_t buf[1];
	HAL_StatusTypeDef ret;
	buf[0] = 0x60;//1 sur reboot
	ret = HAL_I2C_Mem_Write(&hi2c1, MAG_ADR, CTRL_REG_A_M, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		printf("Error Tx\r\n");
	}
}

int config_mag(){
	HAL_StatusTypeDef ret;
	uint8_t buf[3] ;															//Buffer de 3 car on va écrire sur 3 reg d'un coup
	uint8_t res[3] ;															//Buffer permettant de vérifier si les valeurs ont bien été écrites
	buf[0]=0x80;//Valeur a mettre dans ctrm_reg_a
	buf[1]=0x02;//Valeur a mettre dans ctrm_reg_b
	buf[2]=0x00;//Valeur a mettre dans ctrm_reg_c
	ret = HAL_I2C_Mem_Write(&hi2c1, MAG_ADR, CFG_REG_A_M|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		printf("Error Tx\r\n");
	}
	ret = HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, CFG_REG_A_M|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, res, 3, HAL_MAX_DELAY);
	uint8_t i=0;
	//Ici on relit les 3 registres pour vérifier que la valeur souhaitée à bien été inscrite
	for(i=0;i<3;i++){
		if(buf[i]==res[i]){
			printf("0x%02x mis dans le registre CTRL_REG_%d_A \n\r", buf[i], i+1);
		}
		else{
			printf("Valeur mise dans le registre CTRL_REG_%d_A erronee \n\r",i+1);
		}
	}
}

void get_data(struct data_meas* acc, struct data_meas* mag){
		HAL_StatusTypeDef ret;
		uint8_t buf[6] ;
		ret = HAL_I2C_Mem_Write(&hi2c1, ACC_ADR, OUT_X_L_A, I2C_MEMADD_SIZE_8BIT, 0, 0, HAL_MAX_DELAY);
		if ( ret != HAL_OK ) {
			printf("Error Tx\r\n");
		}
		ret = HAL_I2C_Mem_Read(&hi2c1, ACC_ADR, OUT_X_L_A|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
		acc->X=(buf[1]<<8)|(buf[0]);
		acc->Y=(buf[3]<<8)|(buf[2]);
		acc->Z=(buf[5]<<8)|(buf[4]);

		ret = HAL_I2C_Mem_Write(&hi2c1, MAG_ADR, OUTX_L_REG_M, I2C_MEMADD_SIZE_8BIT, 0, 0, HAL_MAX_DELAY);
		if ( ret != HAL_OK ) {
			printf("Error Tx\r\n");
		}
		ret = HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUTX_L_REG_M|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
		mag->X=(buf[1]<<8)|(buf[0]);
		mag->Y=(buf[3]<<8)|(buf[2]);
		mag->Z=(buf[5]<<8)|(buf[4]);
}

void acc_calibration(struct data_meas* acc, struct data_real* real_acc){
	real_acc->X=-6.025*pow(10,-5)*(acc->X+1152)+1.075*pow(10,-7)*(acc->Y-1055)+1.387*pow(10,-6)*(acc->Z-1504);
	real_acc->Y=1.299*pow(10,-6)*(acc->X+1152)-6.14*pow(10,-5)*(acc->Y-1055)+8.793*pow(10,-8)*(acc->Z-1504);
	real_acc->Z=-1.378*pow(10,-6)*(acc->X+1152)+4.689*pow(10,-7)*(acc->Y-1055)-6.001*pow(10,-5)*(acc->Z-1504);
}

void mag_calibration(struct data_meas* mag){
	  mag->X=mag->X-46;
	  mag->Y=mag->Y-88;
	  mag->Z=mag->Z-12;
}

void calcul_angle(struct angle* angle, struct data_real* acc, struct data_meas* mag){
	angle->theta=atan((acc->Y)/(acc->X));
	angle->psi=atan((-acc->Z)/(sqrt(acc->Y*acc->Y+acc->X*acc->X)));
	angle->delta=acos(sqrt(pow((mag->Y*acc->Z-mag->Z*acc->Y),2)+pow((mag->Z*acc->X-mag->X*acc->Z),2)+pow((mag->X*acc->Y-mag->Y*acc->X),2)+(mag->X*acc->Y-mag->Y*acc->Z))/(sqrt(mag->X*mag->X+mag->Y*mag->Y+mag->Z*mag->Z)*sqrt(acc->Z*acc->X+acc->Y*acc->Y+acc->Z*acc->Z)));
	//Conversion en radians
	//NOTE : 57.3 = (360)/(2*pi)
	angle->theta=angle->theta*57.3;
	angle->psi=angle->psi*57.3;
	angle->delta=angle->delta*57.3;
}
void affich_meas(struct data_meas* acc,struct data_real* real_acc,struct data_meas* mag,struct angle* angle){
	//Affichage des toutes les valeurs
	printf("accX=%5d\t accY=%5d\t accZ=%5d\t |\t magX=%5d\t magY=%5d\t magZ=%5d",acc->X,acc->Y,acc->Z,mag->X,mag->Y,mag->Z);
	printf("\t|||||\t");
	printf("accX=%.2f\t accY=%.2f\t accZ=%.2f",real_acc->X,real_acc->Y,real_acc->Z);
	printf("\t|||\t");
	printf("theta=%.2f\t psi=%.2f\t delta=%.2f",angle->theta,angle->psi,angle->delta);
	printf("\r");
}

/*---------------------------------------------------------------------------------*/
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  who_am_i_sensors();//Fonction qui permet dé vérifier la communication avec les capteurs
  reset_acc();//Fonction qui permet de reset les registres de Acc
  config_acc();//Fonction qui permet de régler les registres de config de Mag
  reset_mag();//Fonction qui permet de reset les registres de Acc
  config_mag();//Fonction qui permet de régler les registres de config de Mag
  //Définition des structures contenant les 3 axes
  struct data_meas acc;
  struct data_meas mag;
  struct data_real real_acc;
  struct angle angle;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  get_data(&acc,&mag);
	  acc_calibration(&acc,&real_acc);
	  mag_calibration(&mag);
	  calcul_angle(&angle,&real_acc,&mag);
	  affich_meas(&acc,&real_acc,&mag,&angle);
  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
	  printf("error i2c");
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
