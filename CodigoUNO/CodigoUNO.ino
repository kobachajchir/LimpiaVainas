#include <Arduino.h>
#include <LiquidCrystal_I2C_Koba.h>
#include <RotaryEncoder.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args) write(args);
#else
#define printByte(args) print(args, BYTE);
#endif

uint8_t barra0[] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}; //CONJUNTO DE BARRAS A MOSTRAR
uint8_t barra1[] = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
uint8_t barra2[] = {0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c};
uint8_t barra3[] = {0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e};
uint8_t barra4[] = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};

#define INITIALIZING 0
#define READY 1
#define WORKING 2
#define PAUSED 3
#define FINISHING 4
#define FINISH 5
#define CANCELLED 6

#define MIN_LOOP_MS_TIME 10 // Tiempo base
#define MIN_MS_BUTTON_TIME 5 // Valor * tiempo base, esta en milisegundos
#define MIN_MS_BUTTON_LONGTIME 150 // Valor * tiempo base, esta en milisegundos
#define BUZZER_INITIALIZING_TIME 2 // Valor * tiempo base, esta en milisegundos
#define BUZZER_FINISHING_TIME 10 // Valor * tiempo base, esta en milisegundos
#define BUZZER_FINISH_TIME 100 // Valor * tiempo base, esta en milisegundos
#define BUZZER_CANCELLED_TIME 50 // Valor * tiempo base, esta en milisegundos
#define BUZZER_INITIALIZING_FC 1000 // Frecuencia en HZ
#define BUZZER_FINISHING_FC 1500 // Frecuencia en HZ
#define BUZZER_FINISH_FC 1500 // Frecuencia en HZ
#define BUZZER_CANCELLED_FC 250 // Frecuencia en HZ
#define LCD_ON_TIME 500 // Valor * tiempo base, esta en milisegundos
#define LCD_UPDATE_TIME 500 // Valor * tiempo base, esta en milisegundos
#define TIME_TO_RESET 5 // Valor * tiempo base, esta en milisegundos

#define PIN_ENCODER_DT 2
#define PIN_ENCODER_CLK 3
#define PIN_ENCODER_SW 4

#define MOTOR_ON bandera.unionBit.bit0
#define BUZZER_ON bandera.unionBit.bit1
#define CHANGING_PARAMS bandera.unionBit.bit2
#define DISPLAY_DATA bandera.unionBit.bit3
#define TIME_SET bandera.unionBit.bit4
#define SEC_PASSED bandera.unionBit.bit5
#define ENCODER_TURN_CCW bandera.unionBit.bit6 
#define ENCODER_TURN_CW bandera.unionBit.bit7

#define BUTTON_SW_PRESS bandera2.unionBit.bit0
#define BUTTON_SW_LONG_PRESS bandera2.unionBit.bit1
#define BUTTON_SW_RELEASED bandera2.unionBit.bit2
#define LCD_ON bandera2.unionBit.bit3
#define UPDATE_LCD bandera2.unionBit.bit4
#define CHANGING_VEL bandera2.unionBit.bit5
#define CHANGING_MIN bandera2.unionBit.bit6
#define RESETTING bandera2.unionBit.bit7

#define BUZZERTIMEPASSED bandera3.unionBit.bit0
//#define bandera3.unionBit.bit1
//#define bandera3.unionBit.bit2
//#define bandera3.unionBit.bit3
//#define bandera3.unionBit.bit4
//#define bandera3.unionBit.bit5
//#define bandera3.unionBit.bit6
//#define bandera3.unionBit.bit7

typedef union
{ //Byte con bits individiales
  struct
  {
    uint8_t bit0 : 1;
    uint8_t bit1 : 1;
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
  } unionBit;
  uint8_t unionByte;
} myByte;

myByte bandera;
myByte bandera2;
myByte bandera3;

typedef struct
  {
    uint16_t sec;
    uint8_t min;
} timeStruct;

timeStruct timeToRun;
timeStruct lastTimeRun;

const uint32_t PIN_MOTOR = 5;
const uint32_t PIN_BUZZER = 6;
const uint32_t PIN_LED_TESTIGO = 7;

uint8_t mode = 0;
uint8_t secCounter = 0;
uint8_t timerEncoderSW = 0;
uint8_t timerUpdateLCD = 0;
uint16_t secsToGo;
uint16_t timerLCD;
uint16_t frecuencyBuzzer;
uint32_t lastTime;
char carac;
char linea[16];
uint8_t velocidad;
uint8_t timeToReset;
uint8_t buzzerTime;
const char clientName[] PROGMEM = "NOMBRE CLIENTE"; //16 CARACTERES MAXIMO!!

//FUNCIONES PROTOTIPADAS
void mostrarBarras(uint8_t valor);
void imprimirLCD(char *aImprimir, char altura);
int my_ceil(float num);
char my_strlen(const char *p);
void updateTimeOnLCD();
void updateTimeStruct();
void RotaryChanged(); //we need to declare the func above so Rotary goes to the one below

LiquidCrystal_I2C_Koba lcd(0x27, 16, 2);
RotaryEncoder Rotary(&RotaryChanged, PIN_ENCODER_DT, PIN_ENCODER_CLK, PIN_ENCODER_SW); // Pins 2 (DT), 3 (CLK), 4 (SW)

//FUNCIONES IMPLEMENTADAS
void RotaryChanged()
{
  const unsigned int state = Rotary.GetState();
  
  if (state & DIR_CW){
    ENCODER_TURN_CCW = 1;
  }
    
  if (state & DIR_CCW){
    ENCODER_TURN_CW = 1;
  }
     
}

void updateTimeStruct() {
  // Verificar si ha pasado un segundo (esto depende de tu implementación específica)
  if ((mode == WORKING || mode == FINISHING) && SEC_PASSED) {
    if(secsToGo > 0){
      secsToGo--;
      if(BUZZERTIMEPASSED && mode == FINISHING){
        buzzerTime = BUZZER_FINISHING_TIME;
        frecuencyBuzzer = BUZZER_FINISHING_FC;
        BUZZERTIMEPASSED = 0;
      }
    }
    if (timeToRun.sec > 0) {
      timeToRun.sec--;
    } else {
      if (timeToRun.min > 0) {
        timeToRun.min--;
        timeToRun.sec = 59;
      }
    }
    // Verificar el estado de los últimos 5 segundos
    if(mode == WORKING){
      if (secsToGo <= 5) {
        mode = FINISHING;
        Serial.print("Finishing");
      }
    }
    // Verificar si el tiempo ha llegado a 0:0:0
    if(mode == FINISHING){
      if (secsToGo == 0) {
        mode = FINISH;
        Serial.print("Finish");
        buzzerTime = BUZZER_FINISH_TIME;
        frecuencyBuzzer = BUZZER_FINISH_FC;
        BUZZERTIMEPASSED = 0;
        BUZZER_ON = 0;
      }
    }
  }
}

void updateTimeOnLCD() {  
  // Formatear la cadena a imprimir
  if(mode == WORKING || mode == FINISHING){
    if(mode == FINISHING){
      strcpy(linea, "TERMINANDO");
      imprimirLCD(linea, 0);
    }else if(!CHANGING_PARAMS && !CHANGING_VEL){
      strcpy(linea, "TIEMPO RESTANTE");
      imprimirLCD(linea, 0);
    }
  }else if(mode == READY || mode == WORKING){
    if (CHANGING_PARAMS) {
      if (CHANGING_MIN) {
        strcpy(linea, "MODIF. MINS.");
        imprimirLCD(linea, 0);
      } else if (CHANGING_VEL) {
        sprintf(linea, "MODIF VEL %02d", map(velocidad, 0, 255, 0, 100));
        imprimirLCD(linea, 0);
      }
    }
  }else if(mode == FINISH){
    strcpy(linea, "TERMINO");
    imprimirLCD(linea, 0);
  }else if(mode == PAUSED){
    lcd.limpiar();
    strcpy(linea, "PAUSADO");
    imprimirLCD(linea, 0);
    sprintf(linea, "%02d:%02d", timeToRun.min, timeToRun.sec);
    imprimirLCD(linea, 1);
  }
  
  if(CHANGING_PARAMS){
    if(CHANGING_MIN){
      sprintf(linea, "%02d:%02d", timeToRun.min, timeToRun.sec);
      imprimirLCD(linea, 1);
    }else if(CHANGING_VEL && mode != PAUSED){
      mostrarBarras(velocidad);
    }
  }else {
    sprintf(linea, "%02d:%02d", timeToRun.min, timeToRun.sec);
    imprimirLCD(linea, 1);
  }
  // Imprimir la cadena en la primera línea del LCD
}

char my_strlen(const char *p)
{
  const char *start = p;
  while (*p)
    p++;
  return (char)(p - start);
}

void imprimirLCD(char *aImprimir, char altura)
{
  uint8_t largo = my_strlen(aImprimir);
  uint8_t inicio = 8 - (largo / 2);
  uint8_t i;
  uint8_t count = 0;
  lcd.posCursor(0, altura);
  for (i = 0; i < inicio; i++)
  {
    lcd.print(' ');
    count++;
  }
  for (i = 0; i < largo; i++)
  {
    carac = aImprimir[i];
    lcd.print(carac);
  }
  for (i = 0; i < (16 - largo - count); i++)
  {
    lcd.print(' ');
  }
}

int my_ceil(float num)
{
  int a = num;
  if ((float)a != num)
    return num + 1;

  return num;
}

void mostrarBarras(uint8_t valor)
{
  char j = 0;
  lcd.posCursor(0, 1);           //Posiciono el cursor en la segunda linea
  lcd.print("                "); //Imprimo 16 espacios para limpiar la linea
  char bloques = (valor / 16);
  if (bloques == 0)
  {
    lcd.posCursor(0, 1);
  }
  else
  {
    for (char i = 0; i < bloques; i++)
    {                      //Mueve los espacios del cursor la cantidad que sea, se divide en 80 por 16 espacios con 5 subdivisiones
      lcd.posCursor(i, 1); //Muevo el cursor
      lcd.printByte(4);
    }
  }
  //Ultimo movimiento de cursor
  //Empieza en 3 porque esa es la primer posicion de barra en la romchar, multiplica el resto, por la candidad de subdivisiones, y redondea hacia arriba
  j = my_ceil((((float)valor / 16) - (bloques)) * 5);
  if (j <= 4)
  {
    lcd.printByte(j); //Imprimo la barra correspondiente
  }
  else
  {
    lcd.printByte(4);
  }
  lcd.encenderDisplay(); //Hasta aca abajo el display estaba apagado, ahora lo enciendo y muestro lo que guarde en memoria
}

void setup() {
  Serial.begin(9600);
  Serial.print("Iniciando");
  bandera.unionByte = 0;
  bandera2.unionByte = 0;
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_ENCODER_DT, INPUT);
  pinMode(PIN_ENCODER_CLK, INPUT);
  pinMode(PIN_ENCODER_SW, INPUT);
  pinMode(PIN_LED_TESTIGO, OUTPUT);
  tone(PIN_BUZZER, BUZZER_INITIALIZING_FC);
  buzzerTime = BUZZER_INITIALIZING_TIME;
  frecuencyBuzzer = BUZZER_INITIALIZING_FC;
  BUZZER_ON = 1;
  BUZZERTIMEPASSED = 0;
  lcd.init();
  lcd.limpiar();
  lcd.caracterPersonalizado(0, barra0);
  lcd.caracterPersonalizado(1, barra1);
  lcd.caracterPersonalizado(2, barra2);
  lcd.caracterPersonalizado(3, barra3);
  lcd.caracterPersonalizado(4, barra4);
  strcpy(linea, "INICIANDO");
  imprimirLCD(linea, 0);
  LCD_ON = 1;
  timerLCD = 0;
  lcd.encenderLuz();
  lastTime = millis();
  mode = INITIALIZING;
  DISPLAY_DATA = 1;
  TIME_SET = 0;
  timeToRun.sec = timeToRun.min = 0;
  strcpy(linea, "RADAL");
  imprimirLCD(linea, 0);
  strcpy_P(linea, clientName);
  imprimirLCD(linea, 1);
  UPDATE_LCD = 1;
  timerUpdateLCD = LCD_UPDATE_TIME;
  velocidad = 255;
  timeToReset = TIME_TO_RESET;
  MOTOR_ON = 0;
  analogWrite(PIN_MOTOR, 0);
}


void loop() {
  if ((millis() - lastTime) >= MIN_LOOP_MS_TIME) {
    lastTime = millis();
    if (secCounter < 100) { //100 * 10ms del min loop ms time dan aprox 1s de delay
      secCounter++;
    } else {
      SEC_PASSED = 1;
      DISPLAY_DATA = 1;
      secCounter = 0;
    }
    if (LCD_ON) {
      if (timerLCD < LCD_ON_TIME) {
        timerLCD++;
      } else {
        LCD_ON = 0;
        timerLCD = 0;
        lcd.apagarLuz();
        Serial.println("LCD off");
      }
    }
    if(UPDATE_LCD){
      if(timerUpdateLCD > 0){
        timerUpdateLCD--;
      }
    }
    if(buzzerTime > 0){
      buzzerTime--;
    }else{
      BUZZERTIMEPASSED = 1;
    }
    if (digitalRead(PIN_ENCODER_SW) == LOW) {
      timerEncoderSW++;
      if(LCD_ON){
        timerLCD = 0;
      }
    } else {
      if (timerEncoderSW >= MIN_MS_BUTTON_TIME && timerEncoderSW < MIN_MS_BUTTON_LONGTIME) {
        Serial.println("Enter button pressed");
        timerEncoderSW = 0;
        if (!LCD_ON) {
          LCD_ON = 1;
          timerLCD = 0;
          lcd.encenderLuz();
          Serial.println("LCD on");
        } else {
          if (!BUTTON_SW_PRESS) {
            BUTTON_SW_PRESS = 1;
            BUTTON_SW_LONG_PRESS = 0;
            BUTTON_SW_RELEASED = 0;
          }
        }
      }else if(timerEncoderSW >= MIN_MS_BUTTON_LONGTIME){
        Serial.println("Enter button longpressed");
        timerEncoderSW = 0;
        if (!LCD_ON) {
          LCD_ON = 1;
          timerLCD = 0;
          lcd.encenderLuz();
          Serial.println("LCD on");
        } else {
          if (!BUTTON_SW_LONG_PRESS) {
            BUTTON_SW_PRESS = 0;
            BUTTON_SW_LONG_PRESS = 1;
            BUTTON_SW_RELEASED = 0;
          }
        }
      }
      timerEncoderSW = 0;
      if (BUTTON_SW_PRESS == 1 || BUTTON_SW_LONG_PRESS == 1) {
        Serial.println("Enter button released");
        BUTTON_SW_RELEASED = 1;
      }
    }

  }

  // Manejo del botón ENTER
  if (BUTTON_SW_RELEASED && mode != INITIALIZING) {
    if(LCD_ON){
      timerLCD = 0;
    }
    Serial.println("SW PRESS");
    if(BUTTON_SW_PRESS){
      if(mode == PAUSED && RESETTING){
        strcpy(linea, "PRESIONE ENTER");
        imprimirLCD(linea, 0);
        Serial.print("Cancelled resetting");
        RESETTING = 0;
        if(secCounter > 5){
          mode == WORKING;
          Serial.print("Back to working");
        }else{
          mode == FINISHING;
          Serial.print("Back to finishing");
        }
        timeToReset = TIME_TO_RESET;
      }else if (mode == READY) {
        if (CHANGING_PARAMS) {
          if (CHANGING_MIN) {
            if(timeToRun.min != 0){
              CHANGING_MIN = 0;
              CHANGING_VEL = 1;
              sprintf(linea, "MODIF VEL %02d", map(velocidad, 0, 255, 0, 100));
              Serial.println("Changing velocity");
            }
          } else if (CHANGING_VEL) {
            CHANGING_MIN = 0;
            CHANGING_VEL = 0;
            CHANGING_PARAMS = 0;
            strcpy(linea, "DATOS GUARDADOS");
            imprimirLCD(linea, 0);
            timerUpdateLCD = LCD_UPDATE_TIME;
            UPDATE_LCD = 1;
            Serial.println("Params saved");
            secsToGo = timeToRun.min*60;
          }
        } else if (TIME_SET && timeToRun.min != 0) {
          mode = WORKING;
          lastTimeRun.min = timeToRun.min;
          lastTimeRun.sec = 0;
          Serial.println("Mode changed to WORKING");
        }
      } else if (mode == WORKING || mode == FINISHING) {
        mode = PAUSED;
        lcd.limpiar();
        strcpy(linea, "PAUSADO");
        updateTimeOnLCD();
        Serial.println("Mode changed to PAUSED");
      } else if (mode == PAUSED) {
        mode = WORKING;
        if(CHANGING_PARAMS && CHANGING_VEL){
          CHANGING_VEL = 0;
          CHANGING_PARAMS = 0;
          UPDATE_LCD = 0;
          timerUpdateLCD = 0;
          updateTimeOnLCD();
        }
        Serial.println("Mode changed to WORKING");
      }else if(mode == FINISH || mode == CANCELLED){
        mode = READY;
        strcpy(linea, "LISTO");
        Serial.println("lastTime min");
        Serial.println(lastTimeRun.min);
        TIME_SET = 1;
        timeToRun.min = lastTimeRun.min;
        timeToRun.sec = 0;
        secsToGo = lastTimeRun.min*60;
        Serial.println("READY again");
      }
      updateTimeOnLCD();
      BUTTON_SW_PRESS = 0;
    }else if(BUTTON_SW_LONG_PRESS){
      if(mode == PAUSED){
        UPDATE_LCD = 1;
        timerUpdateLCD = LCD_UPDATE_TIME;
        strcpy(linea, "RESETEANDO EN:");
        imprimirLCD(linea, 0);
        sprintf(linea, "%02d", timeToReset);
        imprimirLCD(linea, 1);
        RESETTING = 1;
      }
      BUTTON_SW_LONG_PRESS = 0;
    }
    BUTTON_SW_RELEASED = 0;
  }else if(mode == INITIALIZING){
    BUTTON_SW_RELEASED = 0;
    BUTTON_SW_PRESS = 0;
    BUTTON_SW_LONG_PRESS = 0;
  }

  // Manejo del encoder CW
  if (ENCODER_TURN_CW) {
    if(LCD_ON){
      timerLCD = 0;
    }else{
      LCD_ON = 1;
      lcd.encenderLuz();
    }
    Serial.println("CW MOVE");
    if(mode == READY){
      if (CHANGING_PARAMS) {
        if (CHANGING_MIN) {
          if (timeToRun.min < 59) {
            timeToRun.min++;
            Serial.print("Minutes incremented: ");
            Serial.println(timeToRun.min);
          }
        } else if (CHANGING_VEL) {
          if (velocidad < 255) {
            velocidad+=5;
            Serial.print("Velocity incremented: ");
            Serial.println(velocidad);
          }
        }
      } else {
        TIME_SET = 0;
        CHANGING_PARAMS = 1;
        CHANGING_MIN = 1;
        CHANGING_VEL = 0;
        Serial.println("Started changing seconds");
      }
      updateTimeOnLCD();
    }else if(mode == WORKING){
      UPDATE_LCD = 1;
      timerUpdateLCD = LCD_UPDATE_TIME;
      CHANGING_PARAMS = 1;
      CHANGING_VEL = 1;
      if (velocidad < 255) {
        velocidad+=5;
        Serial.print("Velocity incremented: ");
        Serial.println(velocidad);
      }
      sprintf(linea, "MODIF VEL %02d", map(velocidad, 0, 255, 0, 100));
      imprimirLCD(linea, 0);
      analogWrite(PIN_MOTOR, velocidad);
    }
    ENCODER_TURN_CW = 0;
  }
  // Manejo del encoder CCW
  if (ENCODER_TURN_CCW) {
    if(LCD_ON){
      timerLCD = 0;
    }else{
      LCD_ON = 1;
      lcd.encenderLuz();
    }
    Serial.println("CCW MOVE");
    if(mode == READY){
      if (CHANGING_PARAMS) {
        if (CHANGING_MIN) {
          if (timeToRun.min > 0) {
            timeToRun.min--;
            Serial.print("Minutes decremented: ");
            Serial.println(timeToRun.min);
          }
        } else if (CHANGING_VEL) {
          if (velocidad > 0) {
            velocidad-=5;
            Serial.print("Velocity decremented: ");
            Serial.println(velocidad);
          }
        }
      } else {
        TIME_SET = 0;
        CHANGING_PARAMS = 1;
        CHANGING_MIN = 1;
        CHANGING_VEL = 0;
        Serial.println("Started changing seconds");
      }
      updateTimeOnLCD();
    }else if(mode == WORKING){
      UPDATE_LCD = 1;
      timerUpdateLCD = LCD_UPDATE_TIME;
      CHANGING_PARAMS = 1;
      CHANGING_VEL = 1;
      if (velocidad > 0) {
        velocidad-=5;
        Serial.print("Velocity decremented: ");
        Serial.println(velocidad);
      }
      sprintf(linea, "MODIF VEL %02d", map(velocidad, 0, 255, 0, 100));
      imprimirLCD(linea, 0);
      analogWrite(PIN_MOTOR, velocidad);
    }
    ENCODER_TURN_CCW = 0;
  }
  if(DISPLAY_DATA){
    DISPLAY_DATA = 0;
    if(SEC_PASSED && mode == PAUSED && RESETTING){
      if(timeToReset > 0){
        timeToReset--;
        sprintf(linea, "%02d", timeToReset);
        imprimirLCD(linea, 1);
        LCD_ON = 1;
        timerLCD = 0;
      }else{
        mode = CANCELLED;
        buzzerTime = BUZZER_CANCELLED_TIME;
        frecuencyBuzzer = BUZZER_CANCELLED_FC;
        BUZZERTIMEPASSED = 0;
        lcd.limpiar();
        LCD_ON = 1;
        timerLCD = 0;
        strcpy(linea, "CANCELADO");
        imprimirLCD(linea, 0);
        strcpy(linea, "PRESIONE ENTER");
        imprimirLCD(linea, 1);
        Serial.println("Cancelled time");
        RESETTING = 0;
        timeToReset = TIME_TO_RESET;
      }
    }
    if(SEC_PASSED && TIME_SET && (mode == WORKING || mode == FINISHING)){
      Serial.println("Second passed");
      updateTimeStruct();
      updateTimeOnLCD();
      SEC_PASSED = 0;
    }
  }
  if(UPDATE_LCD){
    if(timerUpdateLCD == 0){
      UPDATE_LCD = 0;
      if(mode == READY && !CHANGING_PARAMS && !TIME_SET){
        Serial.print("Updating");  
        TIME_SET = 1;
        strcpy(linea, "LISTO");
        imprimirLCD(linea, 0);
      }else if(mode == INITIALIZING){
        mode = READY;
        buzzerTime = BUZZER_FINISHING_TIME;
        updateTimeOnLCD();
        strcpy(linea, "CONFIG. TIEMPO");
        imprimirLCD(linea, 0);
      }else if(mode == WORKING && CHANGING_PARAMS && CHANGING_VEL){
        CHANGING_VEL = 0;
        CHANGING_PARAMS = 0;
        updateTimeOnLCD();
      }
      timerUpdateLCD = LCD_UPDATE_TIME;
    }
  }
  if(mode == WORKING && MOTOR_ON == 0){
    analogWrite(PIN_MOTOR, velocidad);
    digitalWrite(PIN_LED_TESTIGO, HIGH);
    MOTOR_ON = 1;
    Serial.print("Motor girando a");
    Serial.println(velocidad);
  }
  if(mode == PAUSED && MOTOR_ON == 1){
    analogWrite(PIN_MOTOR, 0);
    digitalWrite(PIN_LED_TESTIGO, LOW);
    MOTOR_ON = 0;
    Serial.print("Pauso motor");
  }
  if((mode == FINISH || mode == CANCELLED) && MOTOR_ON == 1){
    analogWrite(PIN_MOTOR, 0);
    digitalWrite(PIN_LED_TESTIGO, LOW);
    MOTOR_ON = 0;
    Serial.print("Motor sin tiempo");
  }
  if(buzzerTime > 0 && !BUZZERTIMEPASSED && !BUZZER_ON){
    tone(PIN_BUZZER, frecuencyBuzzer);
    Serial.print("buzz time on");
    BUZZER_ON = 1;
  }else if(BUZZERTIMEPASSED && BUZZER_ON){
    noTone(PIN_BUZZER);
    BUZZER_ON = 0;
    BUZZERTIMEPASSED = 0;
    Serial.print("buzz time passed");
    if(mode == FINISHING && SEC_PASSED){
      buzzerTime = BUZZER_FINISHING_TIME;
    }
  }
}
