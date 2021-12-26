
#ifndef DISTANCESENSORMANAGER_H_
#define DISTANCESENSORMANAGER_H_

/* Initial ACD Module */
void ADC_init(void);

/* Initial ADC for front sensor */
void ADCFront_Init(void);

/* Initial for right sensor */
void ADCRight_Init(void);

/* REad the ADC data from front sensor */
uint32_t ADC_DataGetFront(void);

/* Read the ADC data from right sensor */
uint32_t ADC_DataGetRight(void);

/* Function to convert ADC data to distance front [cm] */
float distanceFront(void);

/* Function to convert ADC data to distance right [cm] */
float distanceRight(void);

void distanceFrontPrint(void);

void distanceRightPrint(void);

/* Initial reflection sensor */
void RS_init(void); //Initialize Reflection Sensor

/* Function to read the tape from sensor - True = Black; False = White */
bool RS_Read(void); //Read the tape

#endif /* DISTANCESENSORMANAGER_H_ */
