#include <QTRSensors.h>


#define rightMaxSpeed  125                   // Máxima velocidade do motor direito (20  a mais q o esquerdo) //105
#define leftMaxSpeed    105                  // Máxima velocidade do motor esquerdo //85
#define rightBaseSpeed  85                  // Velocidade Base do motor direito; Velocidade quando o robô está na linha (20  a mais q o esquerdo)//55
#define leftBaseSpeed   55                 // Velocidade Base do motor esquerdo; Velocidade quando o robô está na linha//35
#define NUM_SENSORS    6                     // Número de sensores usados
#define NUM_SAMPLES_PER_SENSOR  4            // O valor retornado é uma média de 4 leituras
#define EMITTER_PIN    QTR_EMITTERS_ON       // Controle dos emissores dos sensores
////***/
#define NUM_SENSORS1             2  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR1  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN1             2  // emitter is controlled by digital pin 2

// Motor da esquerda
#define leftMotorPWM 9
#define leftMotor 6

// Motor da direita
#define rightMotorPWM 5
#define rightMotor 3


// Botão
#define ButtonPin 13


QTRSensorsAnalog qtra((unsigned char []) {
  1, 2, 3, 4, 5, 6
} , NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN); // Inicialização dos sensores
QTRSensorsAnalog qtra1((unsigned char[]) { //Não usa
  0, 7
},  NUM_SENSORS1, NUM_SAMPLES_PER_SENSOR1, EMITTER_PIN1);

unsigned int sensorValues1[NUM_SENSORS1];
unsigned int sensorValues[NUM_SENSORS];

// Declaração de variáveis gerais utilizadas no código

int ldrPin1 = 0;
int ldrval1 = 0;
int boolval1 = 0;

int ldrPin2 = 7;
int ldrval2 = 0;
int boolval2 = 0;

int currentState = 0;
int previousState = 0;



float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float P = 0.0;
float I = 0.0;
float D = 0.0;

int position;
char readMode;
char whiteLine;



int buttonCounter;

int countDir;

bool buttonState = 0;


double lastError = 0;
long lastProcess = 0;

unsigned long startime;
unsigned long stoptime;
unsigned long desire;
void setup()
{

  pinMode(rightMotor, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  int i;

startime = 0;
stoptime = 0;
desire = 5000;
  
  for (i = 0; i < 250; i++) // Calibragem dos sensores
  {
    qtra.calibrate();
  }

  Serial.begin (9600);

  delay (10);


  ///****////

  {
    delay(500);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
      qtra1.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    }
    digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

    // print the calibration minimum values measured when emitters were on
    Serial.begin(9600);
   /* for (int i = 0; i < NUM_SENSORS1; i++)
    {
      Serial.print(qtra1.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();*/

    // print the calibration maximum values measured when emitters were on
   /* for (int i = 0; i < NUM_SENSORS1; i++)
    {
      Serial.print(qtra1.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);*/
  }
}
  void loop() {
    ////****////

    qtra1.read(sensorValues);// instead of unsigned int position = qtra.readLine(sensorValues);
    unsigned int position = qtra1.readLine(sensorValues);

    // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
    // 1000 means minimum reflectance, followed by the line position
  /*  for (unsigned char i = 0; i < NUM_SENSORS1; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(); // uncomment this line if you are using raw values
    // Serial.println(position); // comment this line out if you are using raw values

    delay(250);*/

    ///****///
    unsigned long currentMillis = millis(); // Contagem de tempo desde o início do programa

      buttonCounter = 0;

      digitalRead(ButtonPin); // Leitura do estado do botão

      if (digitalRead (ButtonPin) == HIGH) // Condição para iniciar ou não a volta
      
      {
        buttonState = 1;
        buttonCounter = buttonCounter + 1;
        int countDir = 0;
      }
      else {
        buttonState = 0;
      }
  
      switch (buttonCounter) { // Quando o botão for apertado seu valor será 1 e o case abaixo será executado

        case 1:
          while (countDir < 8) { // Quantidade de vezes que o sensor direito passa por uma linha branca


            // Regulagem das constantes do PID
            Kp = 0.05   ;  // ajuste na reta ?//DG: 0.4
            Ki = 0.002 ;       // ajuste a longo prazo nas curvas //DG: 0.9
            Kd = 0.08     ;  // ajuste nas curvas? //DG: 7.0


            position = qtra.readLine(sensorValues, readMode = QTR_EMITTERS_ON, whiteLine = 1); //  Leitura da posição do robô na pista; Biblioteca QTR

            float error =  2500.0 - position;
            float deltaTime = (millis() - lastProcess) / 100.0;
            lastProcess = millis();

            //Serial.println(position);

            /*Serial.print("error= ");
              Serial.print(error );
              Serial.print ('\t');
              Serial.print("deltaTime= ");
              Serial.print( deltaTime );*/
            //P
            P = error * Kp;

            //I
            I = ((error + lastError) * Ki) / deltaTime;
            //D
            D  = ((error - lastError) * Kd) / deltaTime;

            lastError = error ;
            double motorSpeed = P + I + D ;
            /*
              Serial.print ('\t');
              Serial.print("P ");
              Serial.print( P );
              Serial.print ('\t');
              Serial.print("I ");
              Serial.print(I );
              Serial.print ('\t');
              Serial.print("D");
              Serial.print(D);
              Serial.print ('\t');
            */
            /*Serial.print("motorSpeed ");
              Serial.print( motorSpeed );*/

            double rightMotorSpeed = rightBaseSpeed + motorSpeed;
            double leftMotorSpeed = leftBaseSpeed - motorSpeed;

            /* Serial.print('\t');
              Serial.print("ESQM");
              Serial.print(leftMotorSpeed);
              Serial.print('\t');
              Serial.print("DIRM");
              Serial.print(rightMotorSpeed);*/

            digitalWrite(rightMotor, LOW);
            digitalWrite(leftMotor, LOW);


            if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // Evita que o motor ultrapasse a velocidade máxima
            if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;     // Evita que o motor ultrapasse a velocidade máxima



            if (rightMotorSpeed < 0)
            {

              digitalWrite(rightMotorPWM, HIGH);
              rightMotorSpeed = - rightMotorSpeed;
              if (rightMotorSpeed > 0)
              {
                rightMotorSpeed = 100;//20

                analogWrite(rightMotor, rightMotorSpeed);
              }

            }

            if (leftMotorSpeed < 0)
            {
              digitalWrite(leftMotorPWM, HIGH);
              leftMotorSpeed = - leftMotorSpeed;
              if (leftMotorSpeed > 0)
              {
                leftMotorSpeed = 100;//20
                analogWrite(leftMotor, leftMotorSpeed);
              }
            }

            analogWrite(rightMotorPWM, rightMotorSpeed);
            analogWrite(leftMotorPWM, leftMotorSpeed);

            /*Serial.print('\t');
              Serial.print("ESQ");
              Serial.print(leftMotorSpeed);
              Serial.print('\t');
              Serial.print("DIR");
              Serial.println (rightMotorSpeed);*/
              
           ldrval1 = analogRead(ldrPin1);
           /* ldrval2 = analogRead(ldrPin2);
            Serial.print("dir");
            Serial.print(ldrval1);
            Serial.print('\t');
            Serial.print("esq");
            Serial.println(ldrval2);*/

            if (ldrval1 >= 1000 ) //QUANTO MAIS BRANCO, MENOR O VALOR (200 MIN)
            {
              boolval1 = HIGH;
            }
            else
            {
              if (ldrval1 <= 880)
              {
                boolval1 = LOW;
              }
            }
            if (boolval1 == LOW)
            {
              currentState = 0;
            }
            else
            {
              currentState = 1;
            }
            if (currentState != previousState)
            {
              if (currentState == 0) //  && boolval2 == LOW condicional para ignorar cruzamentos
              {
                countDir = countDir + 1;
              }
            }
            previousState = currentState;
          }
          previousState = currentState;

          if (countDir >= 8) // Após o robô terminar a volta, diminui a sua velocidade e o faz parar na Área de partida e chegada
          {

            digitalWrite(leftMotor,  LOW);
            digitalWrite(rightMotor, LOW) ;
            analogWrite(rightMotorPWM, 100);
            analogWrite(leftMotorPWM, 105);

            delay(200);

            digitalWrite(leftMotor, LOW);
            digitalWrite(rightMotor, LOW);
            analogWrite(rightMotorPWM, 100);
            analogWrite(leftMotorPWM, 105);

            delay(100);

            digitalWrite(leftMotor, LOW);
            digitalWrite(rightMotor, LOW);
            analogWrite(rightMotorPWM, 0);
            analogWrite(leftMotorPWM, 0);

          }

          buttonCounter = buttonCounter++;
          break; // Encerra o código
      }
   }
 
   
