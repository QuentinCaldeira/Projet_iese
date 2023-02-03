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
#define WHO_AM_I_A 		0x0F//Adresse contenant le WHOAMI de ACC
#define WHO_AM_I_M		0x4F//Adresse contenant le WHOAMI de MAG

#define CTRL_REG1_A 	0x20//Valeur contenant le premier registre de controle de ACC. Les autres ne sont pas nécessaires car nous allons faire une écriture multiple
#define CTRL_REG5_A 	0x24//Valeur contenant le registre 5 de controle de ACC, permettant le REBOOT
#define OUT_X_L_A		0x28//Valeur contenant le registre de sortie des valeurs de ACC

#define CTRL_REG_A_M 	0x60//Valeur contenant le premier registre de controle de MAG. Les autres ne sont pas nécessaires car nous allons faire une écriture multiple
#define	OUTX_L_REG_M	0x68//Valeur contenant registre de sortie des valeurs de MAG

#define SUB_INCREMENT   0x80//Masque pour autoriser l'incrément des sous registres (écriture multiple)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//structure contenant les axes en INT
struct data_meas {
  int16_t X;
  int16_t Y;
  int16_t Z;
};

//structure contenant les axes en FLOAT
struct data_real {
  float X;
  float Y;
  float Z;
};

//structure contenant les angles en FLOAT
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
 	buf[1]=0x80;  																	//Data de reset, 1 logique sur BOOT
	ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_ADR, buf, 2, HAL_MAX_DELAY);			//On teste si la transmission se passe bien
	if ( ret != HAL_OK ) {
		printf("Error Tx\r\n");														//Si elle se passe mal, on met une erreur
	}
}

int config_acc(){
	HAL_StatusTypeDef ret;
	uint8_t buf[6] ;																//Buffer de 6 car on va écrire sur 6 reg d'un coup
	uint8_t res[6] ;																//Buffer permettant de vérifier si les valeurs ont bien été écrites
	buf[0]=0x27;//ODR=10Hz, Axe X Y et Z actifs
	buf[1]=0x00;//Pas de filtres
	buf[2]=0x00;//Pas d'interruptions
	buf[3]=0x00;//FS=+/-2g, SPI disabled
	buf[4]=0x00;//Valeur a mettre dans ctrm_reg_5
	buf[5]=0x00;//Pas d'interruptions
	ret = HAL_I2C_Mem_Write(&hi2c1, ACC_ADR, CTRL_REG1_A|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
	//NOTE : SUB_INCREMENT permet d'autoriser l'incrémentation des registres en cas d'écriture multiple
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
	buf[0] = 0x60;//1 sur reboot et SOFT_RST
	ret = HAL_I2C_Mem_Write(&hi2c1, MAG_ADR, CTRL_REG_A_M, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		printf("Error Tx\r\n");
	}
}

int config_mag(){
	HAL_StatusTypeDef ret;
	uint8_t buf[3] ;															//Buffer de 3 car on va écrire sur 3 reg d'un coup
	uint8_t res[3] ;															//Buffer permettant de vérifier si les valeurs ont bien été écrites
	buf[0]=0x80;//Activation de la compensation de température, ODR=10Hz
	buf[1]=0x02;//Activation de offset cancellation
	buf[2]=0x00;//Pas de désactivation de I2C, pas d'interruption
	ret = HAL_I2C_Mem_Write(&hi2c1, MAG_ADR, CTRL_REG_A_M|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		printf("Error Tx\r\n");
	}
	ret = HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, CTRL_REG_A_M|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, res, 3, HAL_MAX_DELAY);
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
		ret = HAL_I2C_Mem_Write(&hi2c1, ACC_ADR, OUT_X_L_A, I2C_MEMADD_SIZE_8BIT, 0, 0, HAL_MAX_DELAY);//On va dans le registre contenant la sortie de ACC
		if ( ret != HAL_OK ) {
			printf("Error Tx\r\n");
		}
		ret = HAL_I2C_Mem_Read(&hi2c1, ACC_ADR, OUT_X_L_A|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);//On lit les 6 registres contenant X,Y et Z
		acc->X=(buf[1]<<8)|(buf[0]);//Les deux premiers registres lus sont l'axe X. Le 1er reg est celui de poids faible, le 2nd est celui de poids fort
		acc->Y=(buf[3]<<8)|(buf[2]);//Les deux suivants sont l'axe Y. Le 1er reg est celui de poids faible, le 2nd est celui de poids fort
		acc->Z=(buf[5]<<8)|(buf[4]);//Les deux suivants sont l'axe Z. Le 1er reg est celui de poids faible, le 2nd est celui de poids fort

		ret = HAL_I2C_Mem_Write(&hi2c1, MAG_ADR, OUTX_L_REG_M, I2C_MEMADD_SIZE_8BIT, 0, 0, HAL_MAX_DELAY);
		if ( ret != HAL_OK ) {
			printf("Error Tx\r\n");
		}
		ret = HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUTX_L_REG_M|SUB_INCREMENT, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);//On va dans le registre contenant la sortie de MAG
		mag->X=(buf[1]<<8)|(buf[0]);//Les deux premiers registres lus sont l'axe X. Le 1er reg est celui de poids faible, le 2nd est celui de poids fort
		mag->Y=(buf[3]<<8)|(buf[2]);//Les deux suivants sont l'axe Y. Le 1er reg est celui de poids faible, le 2nd est celui de poids fort
		mag->Z=(buf[5]<<8)|(buf[4]);//Les deux suivants sont l'axe Z. Le 1er reg est celui de poids faible, le 2nd est celui de poids fort
}

void acc_calibration(struct data_meas* acc, struct data_real* real_acc){
	//Cette fonction permet de corriger les valeurs de l'ACC. Les coefficients écrits ont été calculés à partir du tableur Excel, voir PJ
	real_acc->X=-6.025*pow(10,-5)*(acc->X+1152)+1.075*pow(10,-7)*(acc->Y-1055)+1.387*pow(10,-6)*(acc->Z-1504);
	real_acc->Y=1.299*pow(10,-6)*(acc->X+1152)-6.14*pow(10,-5)*(acc->Y-1055)+8.793*pow(10,-8)*(acc->Z-1504);
	real_acc->Z=-1.378*pow(10,-6)*(acc->X+1152)+4.689*pow(10,-7)*(acc->Y-1055)-6.001*pow(10,-5)*(acc->Z-1504);
}

void mag_calibration(struct data_meas* mag){
	//Cette fonction permet de corriger les valeurs de MAG. Les coefficients écrits ont été calculés à partir d'un tableau Excel
	  mag->X=mag->X-46;
	  mag->Y=mag->Y-88;
	  mag->Z=mag->Z-12;
}

void calcul_angle(struct angle* angle, struct data_real* acc, struct data_meas* mag){
	//Cette fonction calcule les 3 angles selon les formules données dans le poly
	//ATTENTION, atan retourne une valeur en RADIANS
	angle->theta=atan((acc->Y)/(acc->X));
	angle->psi=atan((-acc->Z)/(sqrt(acc->Y*acc->Y+acc->X*acc->X)));
	angle->delta=acos(sqrt(pow((mag->Y*acc->Z-mag->Z*acc->Y),2)+pow((mag->Z*acc->X-mag->X*acc->Z),2)+pow((mag->X*acc->Y-mag->Y*acc->X),2))/(sqrt(mag->X*mag->X+mag->Y*mag->Y+mag->Z*mag->Z)*sqrt(acc->X*acc->X+acc->Y*acc->Y+acc->Z*acc->Z)));
	angle->phi=atan((mag->X*sin(angle->theta)-mag->Y*cos(angle->theta))/(mag->Z*cos(angle->psi)+mag->Y*sin(angle->theta)*sin(angle->psi)+mag->X*cos(angle->theta)*sin(angle->psi)));
	//Conversion en degrés
	//NOTE : 57.3 = (360)/(2*pi)
	angle->theta=angle->theta*57.3;
	angle->psi=angle->psi*57.3;
	angle->delta=angle->delta*57.3;
	angle->phi=angle->phi*57.3;
}
void affich_meas(struct data_meas* acc,struct data_real* real_acc,struct data_meas* mag,struct angle* angle){
	//Affichage de toutes les valeurs
	printf("Acceleration (reelles)\r\n");
	printf("accX=%.2f\t accY=%.2f\t accZ=%.2f\r\n",real_acc->X,real_acc->Y,real_acc->Z);
	printf("Magnetometre (brutes calibrees)\r\n");
	printf("magX=%5d\t magY=%5d\t magZ=%5d\r\n",mag->X,mag->Y,mag->Z);
	printf("Angles\r\n");
	printf("theta=%.2f\t psi=%.2f\t delta=%.2f\t phi=%.2f\r \033[0;0H",angle->theta,angle->psi,angle->delta,angle->phi);
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

  struct data_meas acc;//Structure pour la valeur brute (int) de l'ACC
  struct data_meas mag;//Structure pour la valeur brute (int) et corrigée (int) de MAG
  struct data_real real_acc;//Structure pour la valeur corrigée de ACC (float)
  struct angle angle;//Structure pour la valeur des angles (float)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\033[2J");//CLEAR TERMINAL SCREEN
  while (1){
	  get_data(&acc,&mag);//Cette fonction permet d'acquérir les données
	  acc_calibration(&acc,&real_acc);//Cette fonction corrige les valeurs de ACC
	  mag_calibration(&mag);//Cette fonction corrige les valeurs de MAG
	  calcul_angle(&angle,&real_acc,&mag);//Cette fonction retourne les angles calculés
	  affich_meas(&acc,&real_acc,&mag,&angle);//Cette fonction affiche les angles
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
