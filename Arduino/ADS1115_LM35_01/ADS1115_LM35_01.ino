/********************************************************************************
 * ADS1115_LM35_01 - Medicao de temperatura com LM35 e ADS1115 (16 bit)
 * ------------------------------------------------------------------------------
 * https://github.com/agaelema/ADS1115_and_LM35
 * developed by: Haroldo Amaral - agaelema@gmail.com
 * 2017/11/18 - v 1.0
 ********************************************************************************/
#include <Wire.h>
#include <Adafruit_ADS1015.h>

/* possibilidades/niveis de acordo com a resolução */
#define   ADC_16BIT_MAX   65536

/* cria instância do ADC */
Adafruit_ADS1115 ads(0x48);

/*
 * Variaveis
 */
float ads_bit_Voltage;
float lm35_constant;


void setup(void) 
{
  /* inicializa a serial */
  Serial.begin(9600);

  /* aguarda a serial estar disponível */
  while (!Serial);

  /* imprime o nome do arquivo com data e hora */
  Serial.println(F("\r\n"__FILE__"\t"__DATE__" "__TIME__));

  /*
   * configura o ganho do PGA interno do ADS1115
   * - Sem configurar ele inicia automaticamente na escala de +/- 6.144V
   * - lembre-se de não exceder os limites de tensao nas entradas
   * - - VDD+0.3v ou GND-0.3v
   */
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
//  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
//  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
//  ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
//  ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
//  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  /* inicializa o ADC */
  ads.begin();

  /* modifique este valor de acordo com o ganho selecionado */
  float ads_InputRange = 6.144f;

  /* no range de +-6.144V, 187.502uV/bit */
  ads_bit_Voltage = (ads_InputRange * 2) / (ADC_16BIT_MAX - 1);

  /* LM35 - tensão por grau Celsius - 10mV/oC */
  lm35_constant = 10.0f / 1000;


  /*
   * imprime o valor da tensao por bit
   * - ADS1115 (de acordo com o ganho do PGA)
   * - Arduino Nano/Uno com referencia em Vcc
   * - Arduino Nano/Uno com referencia interna de 1.1V
   */
  Serial.println();
  Serial.print("ADS volt/bit: ");   Serial.print(ads_bit_Voltage * 1000, 4);     Serial.println(" mV/bit");
  Serial.println();
  Serial.println();

  /* imprime a primeira linha com identificacao dos dados */
  Serial.println("ADS RAW \tADS Temp.");

  /* seta a referencia interna */
  analogReference(INTERNAL);
}

void loop(void) 
{
  /* variaveis apra armazenar o valor RAW do adc */
  int16_t ads_ch0 = 0;
  int16_t nano_ch0_0 = 0;           // usando referencia de Vcc (5V)
  int16_t nano_ch0_1 = 0;           // usando referencia interna (1.1V)
  /* variaveis para armazenar o resultado em tensao */
  float ads_Voltage_ch0 = 0.0f;
  /* variaveis para armazenar a temperatura */
  float ads_Temperature_ch0 = 0.0f;

  /********************************************
   * ADS1115 - 16bit ADC
   * - le o ADC
   * - converter o valor RAW em tensao
   * - calcula a temperatura
   ********************************************/
  ads_ch0 = ads.readADC_SingleEnded(0);
  ads_Voltage_ch0 = ads_ch0 * ads_bit_Voltage;
  ads_Temperature_ch0 = ads_Voltage_ch0 / lm35_constant;

  /* imprime os resultados */
  Serial.print(ads_ch0);    Serial.print("\t\t");   Serial.print(ads_Temperature_ch0, 3);       Serial.print("\t\t");
  Serial.println();
  
  delay(200);
}
