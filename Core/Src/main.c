/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TRUE 1
#define FALSE 0

#define LEFT 0
#define RIGHT 1

#define BACKWARDS 0
#define FORWARDS 1

// max speed
#define PWM_MAX 65535
// Wheel & encoder config
#define LOW_THRESHOLD 2000
#define HIGH_THRESHOLD 3000
#define PULSES_PER_REV 12
#define WHEEL_DIAMETER_MM 40.0f
#define WHEEL_BASE_MM 80.0f
#define M_PI 3.14159265358979323846
#define WHEEL_CIRCUM_MM (M_PI * WHEEL_DIAMETER_MM)
#define MM_PER_PULSE (WHEEL_CIRCUM_MM / PULSES_PER_REV)

#define MAX_ENCODER_DIFF 30
#define MOTOR_BASE_SPEED 0.5f
#define LINE_KP 0.5f
#define LINE_HIGH_THRESHOLD 2000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// state control variables

// LED states
enum LED_STATE {
    LED_ON,
    LED_OFF
};
enum LED_STATE taskLED_state = LED_ON;
uint32_t taskLED_timeout = 0;

typedef enum { MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, DONE } StepType;

typedef struct {
    StepType type;
    float value;  // mm for forward, degrees for turn
} TrajectoryStep;

TrajectoryStep path[] = {
    { MOVE_FORWARD, 500 },
    { TURN_RIGHT,    30 },
    { MOVE_FORWARD, 433 },
    { TURN_LEFT,    90  },
    { MOVE_FORWARD, 245 },
    { DONE, 0 }
};
int current_step = 0;

typedef enum {
    SEARCH_INIT,
    SEARCH_ROTATE_RIGHT,
    SEARCH_ROTATE_RIGHT_BACK,
    SEARCH_ROTATE_LEFT,
    SEARCH_ROTATE_LEFT_BACK,
    SEARCH_DONE
} SearchState;

typedef enum {
    FOLLOW_TRAJECTORY,
    FOLLOW_LINE,
    SEARCH_LINE,
    OVERCOME_GAP,
    AVOID_OBSTACLE,
    COURSE_DONE
} CourseState;

// adc variables
volatile uint16_t adc[6];
volatile uint16_t buffer[6];    // DMA target
volatile uint8_t conversion_done_flag = 1;
char string_buf[100];

uint8_t current_channel_index = 0;
const uint32_t adc_channels[6] = {
    ADC_CHANNEL_5,   // PA0 – Line sensor mid
    ADC_CHANNEL_6,   // PA1 – Encoder left
    ADC_CHANNEL_8,   // PA3 – Line sensor right
    ADC_CHANNEL_9,   // PA4 – Battery
    ADC_CHANNEL_10,  // PA5 – Encoder right
    ADC_CHANNEL_12   // PA7 – Line sensor left
};


// Encoder state
volatile int left_prev_state = 0;
volatile int right_prev_state = 0;
volatile int left_total_count = 0;
volatile int right_total_count = 0;

// variables for driving control
uint32_t start_left = 0;
uint32_t start_right = 0;
volatile int target_pulses = 0;
static uint8_t driving = 0;

// variables for course state
CourseState course_state = FOLLOW_TRAJECTORY;
SearchState search_state = SEARCH_INIT;
int avoidObstacleState = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float getVel(float, int);
void setMotors(int, int, float, float);
void moveRoboboy(int, int, float, float);
void driveMotorsCooperative(int, int);
char driveMMStraightCooperative(int, float);
char spinDegreesCooperative(int, float);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
void readSensorsAndPrint(void);
void process_encoder_signals();
void taskLED(void);
int mm_to_pulses(float);
int degrees_to_pulses(float);
void task_followTrajectory(void);
void reset_encoder_counters(void);
char line_detected(void);
char collision_detected(void);
float* get_pwm_values(void);
void task_followLine();
void task_searchLine(void);
void task_avoidObstacle();
void task_overcomeGap();
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
  /* INITIALIZE PWM */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer, 6);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  TIM1 -> CCR2 = 0;
  TIM1 -> CCR3 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (conversion_done_flag) {
		  readSensorsAndPrint();

		  process_encoder_signals();
		  taskLED();

		  switch (course_state) {
		  	  case FOLLOW_TRAJECTORY:
		  		  task_followTrajectory();
		  		  break;
		  	  case FOLLOW_LINE:
		  		  task_followLine();
		  		  break;
		  	  case SEARCH_LINE:
		  		  task_searchLine();
		  		  break;
		  	  case OVERCOME_GAP:
		  		  task_overcomeGap();
		  		  break;
		  	  case AVOID_OBSTACLE:
		  		  task_avoidObstacle();
		  		  break;
		  	  case COURSE_DONE:
		  		  moveRoboboy(1, 1, 0, 0);
		  	  default:
		  		  moveRoboboy(1, 1, 0, 0);
		  		  break;
		  	  }
	  }


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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

// <------ Code for motor controls ------>

/*
 * @brief Returns fraction of maximum velocity, either inverted or not
 *
 * @param vel must be in range [0,1],
 * @param dir must be 1 - forwards or 0 - backwards, otherwise return -1
 */
float getVel(float vel, int dir) {
	if ((dir != 1 && dir != 0) || vel < 0 || vel > 1) return -1;

	if (dir) {
		return PWM_MAX * (1 - vel);
	} else {
		return PWM_MAX * vel;
	}
}

/*
 * @brief Sets the velocity of the motors and sets the h-bridges according to the wanted direction
 *
 * @param velL velocity of left motor, must be in range [0,65535],
 * @param velR velocity of right motor, must be in range [0,65535],
 * @param leftF bit indicating left motor spin direction, must be 1 or 0,
 * @param rightF bit indicating right motor spin direction, must be 1 or 0
 */
void setMotors(int leftF, int rightF, float velL, float velR) {
	// ==== left motor h-bridges ====
	if (leftF == 0) {
		// case: forwards
		HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, GPIO_PIN_RESET);
	} else {
	    // case: backwards
		HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, GPIO_PIN_SET);
	}

	// ==== right motor h-bridges ====
	if (rightF == 0) {
		// case: forwards
	    HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, GPIO_PIN_RESET);
	} else {
    	// case: backwards
	    HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, GPIO_PIN_SET);
	}

	// set Motor velocities
	TIM1->CCR2 = velL;
	TIM1->CCR3 = velR;
}

/*
 * @brief Function which sets the motor speeds according to the given velocity and direction
 *
 * @param leftF bit indicating left motor spin direction, must be 1 or 0,
 * @param rightF bit indicating right motor spin direction, must be 1 or 0
 * @param velL velocity of right motor, must be in range [0,1],
 * @param velR velocity of right motor, must be in range [0,1],
 */
void moveRoboboy(int leftF, int rightF, float velL, float velR) {
	if((leftF != 1 && leftF != 0) || (rightF != 1 && rightF != 0)) return;

	//right motor runs "backwards"
	rightF = !rightF;

	if (velL < 0) {
		leftF = !leftF;
		velL = -velL;
	}
	if (velR < 0) {
		rightF = !rightF;
		velR = -velR;
	}

	//potentially flip velocity due to h-bridge
	velL = getVel(velL, leftF);
	velR = getVel(velR, rightF);

	setMotors(leftF, rightF, velL, velR);
}

/*
 * @brief Drives both motors with adjusted speeds to minimize encoder difference.
 *
 * Uses encoder feedback to compensate for mechanical asymmetries between left and right motors.
 * Adjusts motor speeds proportionally to keep the robot moving in a straight line.
 * This function should be called repeatedly in a cooperative main loop.
 *
 * @param leftF  Direction for the left motor (0 = forward, 1 = backward)
 * @param rightF Direction for the right motor (0 = forward, 1 = backward)
 */
void driveMotorsCooperative(int leftF, int rightF) {
	if((leftF != 1 && leftF != 0) || (rightF != 1 && rightF != 0)) return;

	int encoderDiff = left_total_count - right_total_count;

	float newSpeedRight;
	float newSpeedLeft;

	if (encoderDiff > MAX_ENCODER_DIFF) {
		newSpeedRight = 0.0f;
		newSpeedLeft = 0.75f;
	} else if (encoderDiff < -MAX_ENCODER_DIFF) {
		newSpeedRight = 0.75f;
		newSpeedLeft = 0.0f;
	} else {
		newSpeedRight = MOTOR_BASE_SPEED - (encoderDiff / 3 * MAX_ENCODER_DIFF);
		newSpeedLeft = MOTOR_BASE_SPEED + (encoderDiff / 3 * MAX_ENCODER_DIFF);
	}

	// Clamp to 0–1
	if (newSpeedLeft > 1.0f) newSpeedLeft = 1.0f;
	if (newSpeedLeft > 1.0f) newSpeedLeft = 1.0f;
	if (newSpeedRight < 0.0f) newSpeedRight = 0.0f;
	if (newSpeedRight < 0.0f) newSpeedRight = 0.0f;

	moveRoboboy(leftF, rightF, newSpeedLeft, newSpeedRight);
}

/*
 * @brief Drives the robot straight for a given distance in millimeters.
 *
 * Starts a straight-line movement in the specified direction and tracks the distance
 * traveled using encoder counts. Must be called repeatedly until it returns 1.
 *
 * @param forward Movement direction (1 = forward, 0 = backward)
 * @param mm      Distance to travel in millimeters
 * @return        1 when the distance is completed, 0 otherwise
 */
char driveMMStraightCooperative(int forward, float mm) {
	if (driving == 0) {
		target_pulses = mm_to_pulses(mm);
		start_left = left_total_count;
		start_right = right_total_count;
		driving = 1;
	}
	if (forward == 1) {
		driveMotorsCooperative(1, 1);
	} else {
		driveMotorsCooperative(0, 0);
	}

	if ((left_total_count - start_left) >= target_pulses &&
	        (right_total_count - start_right) >= target_pulses) {
	        moveRoboboy(1, 1, 0, 0);  // stop
	        driving = 0;
	        return 1;
	    }
	return 0;
}

/*
 * @brief Rotates the robot in place for a specified angle.
 *
 * Initiates a point-turn and tracks the rotation using encoder pulses.
 * Must be called repeatedly until the turn is complete and it return 1.
 *
 * @param direction Direction of rotation (LEFT = 0, RIGHT = 1)
 * @param degrees   Angle to rotate in degrees
 * @return          1 when the rotation is completed, 0 otherwise
 */
char spinDegreesCooperative(int direction, float degrees) {
    if (driving == 0) {
        target_pulses = degrees_to_pulses(degrees);
        start_left = left_total_count;
        start_right = right_total_count;
        driving = 1;
    } else if ((left_total_count - start_left) >= target_pulses && (right_total_count - start_right) >= target_pulses) {
    	moveRoboboy(1, 1, 0, 0);
    	driving = 0;
    	return 1;
    }

    if (direction == RIGHT) {
         // left forward, right backward
        driveMotorsCooperative(0, 1);
    } else {
    	// left backwards, right forwards
    	driveMotorsCooperative(1, 0);
    }
    return 0;
}



// <------ Code for processing encoder signals ------>

/*
 * @brief Callback function for ADC conversion complete interrupt.
 *
 * Stores the read ADC value into the adc[] buffer and starts the next conversion cycle.
 * This function is automatically called when the ADC finishes a conversion.
 *
 * @param hadc Pointer to the ADC handler (should be ADC1)
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
	        for (int i = 0; i < 6; i++) {
	            adc[i] = buffer[i];
	        }
	        conversion_done_flag = 1;
	    }
}

void readSensorsAndPrint(void) {
	if (conversion_done_flag) {
		conversion_done_flag = 0;
	    HAL_ADC_Start_DMA(&hadc1, buffer, 6);
	}

    int len = sprintf(string_buf,
    		"ADC: [%4d %4d %4d %4d %4d %4d] ENC: L=%4d R=%4d\r\n",
			adc[0], adc[1], adc[2], adc[3], adc[4], adc[5],
			left_total_count, right_total_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)string_buf, len, 100);
}

/*
 * @brief Interprets encoder signals from ADC values and counts rising edges.
 *
 * Increments the global encoder tick counters for the left and right wheel
 * when a rising edge is detected in the analog signal.
 * This function should be called regularly (e.g. in main loop).
 */
void process_encoder_signals() {
    // Left encoder
    uint16_t val_l = adc[1];
    uint8_t state_l = (val_l > HIGH_THRESHOLD) ? 1 : (val_l < LOW_THRESHOLD) ? 0 : left_prev_state;
    if (state_l != left_prev_state && state_l == 1) {
        left_total_count++;
    }
    left_prev_state = state_l;

    // Right encoder
    uint16_t val_r = adc[4];
    uint8_t state_r = (val_r > HIGH_THRESHOLD) ? 1 : (val_r < LOW_THRESHOLD) ? 0 : right_prev_state;
    if (state_r != right_prev_state && state_r == 1) {
        right_total_count++;
    }
    right_prev_state = state_r;
}


// <------ Code for LED control ------>
/*
 * @brief Blinks the onboard LED in a cooperative, non-blocking manner.
 *
 * Uses a state machine and HAL_GetTick() to toggle the LED every 500 ms.
 * Should be called regularly in the main loop.
 */
void taskLED(void) {
    uint32_t current_time = HAL_GetTick();

    if (current_time < taskLED_timeout) return;

    switch (taskLED_state) {
        case LED_ON:
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            taskLED_timeout = current_time + 500;
            taskLED_state = LED_OFF;
            break;

        case LED_OFF:
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
            taskLED_timeout = current_time + 500;
            taskLED_state = LED_ON;
            break;
    }
}


// <------ Code for trajectory following ------>

/*
 * @brief Converts a linear distance in millimeters to encoder pulses.
 *
 * @param mm Distance in millimeters
 * @return Number of encoder pulses corresponding to the distance
 */
int mm_to_pulses(float mm) {
    return (int)(mm / MM_PER_PULSE);
}

/*
 * @brief Converts a rotation angle (degrees) to encoder pulses based on robot geometry.
 *
 * Assumes the robot rotates around its center (point turn).
 *
 * @param degrees Angle in degrees
 * @return Number of pulses required for the given rotation
 */
int degrees_to_pulses(float degrees) {
    float turn_circumference = M_PI * WHEEL_BASE_MM;
    float arc_length = (degrees / 360.0f) * turn_circumference;
    return (int) mm_to_pulses(arc_length);
}

/*
 * @brief Drives a predefined path made of trajectory steps (forward/turn).
 *
 * Executes each step from the global path[] array cooperatively. Advances to the next step
 * only after the current movement or turn completes. Transitions to FOLLOW_LINE state once done.
 */
void task_followTrajectory(void) {
    TrajectoryStep* step = &path[current_step];

    switch (step->type) {
        case MOVE_FORWARD:
            if (driveMMStraightCooperative(1, step->value)) {
            	moveRoboboy(1, 1, 0, 0);
            	current_step++;
            }
            break;

        case TURN_LEFT:
            if (spinDegreesCooperative(LEFT, step->value)) {
            	moveRoboboy(1, 1, 0, 0);
            	current_step++;
                reset_encoder_counters();
            }
            break;

        case TURN_RIGHT:
        	if (spinDegreesCooperative(RIGHT, step->value)) {
        		moveRoboboy(1, 1, 0, 0);
        		current_step++;
        		reset_encoder_counters();
        	}
            break;

        case DONE:
            moveRoboboy(1, 1, 0, 0);
            break;
    }
}

/*
 * @brief Resets encoder pulse counts to 0.
 */
void reset_encoder_counters(void) {
    left_total_count = 0;
    right_total_count = 0;
}


// <------ Code for following line ------>

/*
 * @brief Checks if any line sensor detects a dark surface.
 *
 * @return true if any sensor value is above the line detection threshold
 */
char line_detected(void) {
    return (adc[0] > LINE_HIGH_THRESHOLD || adc[2] > LINE_HIGH_THRESHOLD || adc[5] > LINE_HIGH_THRESHOLD);
}

/*
 * @brief Checks if any line sensor detects a dark surface.
 *
 * @return true if any sensor value is above the line detection threshold
 */
char collision_detected(void) {
	return (!HAL_GPIO_ReadPin(GPIOA, switch_left_Pin) || !HAL_GPIO_ReadPin(GPIOA, switch_middle_Pin) || !HAL_GPIO_ReadPin(GPIOA, switch_right_Pin));
}

/*
 * @brief Computes PWM values for left and right motors based on line sensor values.
 *
 * Uses a weighted sum of the left, middle, and right line sensors to calculate
 * position error and applies a P-controller to derive speed correction.
 *
 * @return Pointer to float[2] array: {left PWM, right PWM}
 * @note Dynamically allocates memory — caller is responsible for freeing it if needed.
 */
float* get_pwm_values(void) {
    // All sensors see white → line lost
	static float pwm_values[2];
    if (line_detected() == FALSE) {
    	pwm_values[0] = 0;
    	pwm_values[1] = 0;
    	return pwm_values;
    }

    int sum = adc[0] + adc[2] + adc[5];
    if (sum == 0) {
    	// case: weird stuff and we dont want division by zero errors
    	pwm_values[0] = 0;
    	pwm_values[1] = 0;
    	return pwm_values;
    }
    // Weighted sum for error calculations: left = -1, middle = 0, right = +1
    int error = (-1 * adc[5] + 0 * adc[0] + 1 * adc[2]) * 1000 / sum;

    float correction = LINE_KP * (float)error / 1000.0f;

    float speed_left = MOTOR_BASE_SPEED - correction;
    float speed_right = MOTOR_BASE_SPEED + correction;

    // Clamp
    if (speed_left > 1.0f) speed_left = 1.0f;
    if (speed_left < 0.0f) speed_left = 0.0f;
    if (speed_right > 1.0f) speed_right = 1.0f;
    if (speed_right < 0.0f) speed_right = 0.0f;

    pwm_values[0] = getVel(speed_left, FORWARDS);
    pwm_values[1]= getVel(speed_right, FORWARDS);

    return pwm_values;
}

/*
 * @brief Cooperative state machine to search for a lost line.
 *
 * Performs a left-right sweep using one-sided turns to relocate the line.
 * If no line is found, transitions to OVERCOME_GAP state.
 * If the line is found during the search, resumes line following.
 */
void task_followLine() {
	float* pwm = get_pwm_values();

	if (line_detected()) {
		setMotors(0, 0, pwm[0], pwm[1]);
	} else if (collision_detected()){
		avoidObstacleState = 0;
		reset_encoder_counters();
		course_state = AVOID_OBSTACLE;
	} else {
		course_state = SEARCH_LINE;
	}
}


// <------ Code for search line ------>

/*
 * @brief Cooperative state machine to search for a lost line.
 *
 * Performs a left-right sweep using one-sided turns to relocate the line.
 * If no line is found, transitions to OVERCOME_GAP state.
 * If the line is found during the search, resumes line following.
 */
void task_searchLine(void) {
    switch (search_state) {
        case SEARCH_INIT:
            moveRoboboy(1, 1, 0, 0);
            search_state = SEARCH_ROTATE_RIGHT;
            break;

        case SEARCH_ROTATE_RIGHT:
            if (spinDegreesCooperative(RIGHT, 100.0f)) {
            	moveRoboboy(1, 1, 0, 0);
                search_state = SEARCH_ROTATE_RIGHT_BACK;
            } else if (line_detected()) {
                moveRoboboy(1, 1, 0, 0);
                search_state = SEARCH_DONE;
            }
            break;

        case SEARCH_ROTATE_RIGHT_BACK:
        	if (spinDegreesCooperative(LEFT, 100.0f)) {
        		moveRoboboy(1, 1, 0, 0);
                search_state = SEARCH_ROTATE_LEFT;
        	}
            break;

        case SEARCH_ROTATE_LEFT:
        	if (spinDegreesCooperative(LEFT, 100.0f)) {
        		// case: no line found
        		moveRoboboy(1, 1, 0, 0);
        		search_state = SEARCH_ROTATE_LEFT_BACK;
        	} else if (line_detected()) {
                moveRoboboy(1, 1, 0, 0);
                search_state = SEARCH_DONE;
            }
            break;

        case SEARCH_ROTATE_LEFT_BACK:
        	if (spinDegreesCooperative(RIGHT, 100.0f)) {
        		moveRoboboy(1, 1, 0, 0);
        		reset_encoder_counters();
        		search_state = SEARCH_INIT;
                course_state = OVERCOME_GAP;
        	}
            break;
        case SEARCH_DONE:
            course_state = FOLLOW_LINE;
            reset_encoder_counters();
            search_state = SEARCH_INIT;
            break;
    }
}


// <------ Code for avoid obstacle------>

/*
 * @brief Obstacle avoidance maneuver using a fixed path around the obstacle.
 *
 * The robot first drives backward, turns, then bypasses the obstacle
 * using a sequence of trajectory steps.
 * Each step is executed cooperatively via driveMMStraightCooperative() and spinDegreesCooperative().
 */

void task_avoidObstacle() {
	switch(avoidObstacleState) {
		case 0:
			if (driveMMStraightCooperative(0, 50)) {
				avoidObstacleState++;
			}
			break;
		case 1:
			if (spinDegreesCooperative(RIGHT, 90)) {
				avoidObstacleState++;
			}
			break;
		case 2:
			if (driveMMStraightCooperative(1, 150)) {
			    avoidObstacleState++;
			}
			break;
		case 3:
			if (spinDegreesCooperative(LEFT, 90)) {
				avoidObstacleState++;
			}
			break;
		case 4:
			if (driveMMStraightCooperative(1, 200)) {
				avoidObstacleState++;
			}
			break;
		case 5:
			if (spinDegreesCooperative(LEFT, 90)) {
				avoidObstacleState++;
			}
			break;
		case 6:
			if (driveMMStraightCooperative(1, 150)) {
				avoidObstacleState++;
			}
			break;
		case 7:
			if (spinDegreesCooperative(RIGHT, 90)) {
				avoidObstacleState++;
			}
			break;
		default:
			course_state = FOLLOW_LINE;
	}
}

/*
 * @brief Handles the case when the robot lost the line due to a gap.
 *
 * Continues driving straight while checking for the line.
 * Once the line is detected again, switches back to FOLLOW_LINE state.
 */
void task_overcomeGap() {
	if (line_detected()) {
		course_state = FOLLOW_LINE;
	} else {
		driveMMStraightCooperative(1, 100);
	}
}




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
