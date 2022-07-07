
/* Começamos definindo as portas do driver (é utilizado DEFINE ao invés de int ou float para economizar memória do arduino.
 O compilador substitui todas as variaveis pelo valor que definimos durante a compilação)*/
#define PININ1 2
#define PININ2 4
#define PININ3 5
#define PININ4 7
#define PINENA 3
#define PINENB 6

// Portas dos sensores 
#define SENSOR1 A0
#define SENSOR2 A1
#define SENSOR3 A2
#define SENSOR4 A3
#define SENSOR5 A4
#define SENSOR6 A5

// Valores de ajustes para o seguidor de linha, os valores foram e devem ser ajustados conforme a necessidade do robô
#define TRESHOLD 750                       // Valor de referencia para cor da linha branca
#define SPEED0 200                       // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 0 0)
#define SPEED1 170                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 1 0)

#define SPEED2 180                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 0 0)
#define SPEED3 150                        // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 0)
#define SPEED4 100                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 1)

#define SPEED5 0                           // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 0)
#define SPEED6 0                         // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 1)
#define SPEED7 120                       // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 0 1)

#define RUNTIME 18500                      // Valor em segundos no qual o robô vai executar o percurso

int PWM_A = 9;
int PWM_B = 8;


int vel_A = 200;
int vel_B = 255;

//Dados do PID são definidos 
int Ki = 0;
int Kp = 42;
int Kd = 21;
int I = 0, P = 0, D = 0, PID = 0;
int velesq = 0, veldir = 0;
int erro = 0, erro_anterior =0;

void setup() {
  

pinMode(PWM_A,OUTPUT);
pinMode(PWM_B,OUTPUT);
}

void loop() {   //Função de seguir a linha é chamada em loop
  segueLinha();
}

void motorControl(int speedLeft, int speedRight){
    if(PID >= 0){
    velesq = vel_B;
    veldir = vel_A - PID;
}else{
    velesq = vel_B + PID;
    veldir = vel_A;
}


// Função para controle do driver de motor

  // Definições das portas digitais
  pinMode(PININ1, OUTPUT);
  pinMode(PININ2, OUTPUT);
  pinMode(PININ3, OUTPUT);
  pinMode(PININ4, OUTPUT);
  pinMode(PINENA, OUTPUT);
  pinMode(PINENB, OUTPUT);

  // Ajustes motor da esquerda
  if (speedLeft < 0) {
    speedLeft = -speedLeft;
    digitalWrite (PININ3, HIGH);
    digitalWrite (PININ4, LOW);
  } else {
    digitalWrite (PININ3, LOW);
    digitalWrite (PININ4, HIGH);
  }

  // Ajustes motor da direita
  if (speedRight < 0) {
    speedRight = -speedRight;
    digitalWrite (PININ1, LOW);
    digitalWrite (PININ2, HIGH);
  } else {
    digitalWrite (PININ1, HIGH);
    digitalWrite (PININ2, LOW);
  }

  analogWrite (PINENA, speedLeft);
  analogWrite (PINENB, speedRight);
  analogWrite (PWM_A, veldir);
  analogWrite (PWM_B, velesq);
}

void motorOption(char option, int speedLeft, int speedRight) {
  // Função para controle de motor com pre definições para testes.
  switch (option) {
    case '6': // Caso 6 seja selecionado, o robô vira para a esquerda.
      motorControl(-speedLeft, speedRight);
      break;
    case '4': //  Caso 6 seja selecionado, o robô vira para a direita.
      motorControl(speedLeft, -speedRight);
      break;
    case '2': //  Caso 6 seja selecionado, o robô vai para trás.
      motorControl(-speedLeft, -speedRight);
      break;
    case '8': //  Caso 6 seja selecionado, o robô vira para frente.
      motorControl(speedLeft, speedRight);
      break;
    case '0': //  Caso 6 seja selecionado, o robô para.
      motorControl(0, 0);
      break;
  }
}

bool motorStop(long runtime, long currentTime) {
  // Função de parada do robô
  if (millis() >= (runtime + currentTime)) {
    motorOption('0', 0, 0); //Caso 0 é selecionado
    while (true) {
    }
    return false;
  }
  return true;
}

void readSensors(void) {
  // Função para leitura dos sensores
  Serial.print(analogRead(SENSOR1));
  Serial.print(' ');
  Serial.print(analogRead(SENSOR2));
  Serial.print(' ');
  Serial.print(analogRead(SENSOR3));
  Serial.print(' ');
  Serial.print(analogRead(SENSOR4));
  Serial.print(' ');
  Serial.print(analogRead(SENSOR5));
  Serial.print(' ');
  Serial.println(analogRead(SENSOR6));
  Serial.print(' ');
}

void segueLinha(void) {
  // Função para controle do seguidor de linha em modo de maquina de estado finita
  bool flag = true;
  long currentTime = millis();

  while (flag) {
    flag = motorStop(RUNTIME, currentTime);

    // leitura do sensor (1 1 1 1 1 1)
    if (analogRead(A0) <= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) <= TRESHOLD) {
      motorOption('8', SPEED0, SPEED0);
      // leitura do sensor (0 1 1 1 1 0)
    } else if ( analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED0);
      // leitura do sensor (0 0 1 1 0 0)
    } else if ( analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED0);

      // leitura do sensor (0 1 1 1 0 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED1);

      // leitura do sensor (0 0 1 1 1 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD ) {
      motorOption('8', SPEED1, SPEED0);

      // leitura do sensor (0 0 1 0 0 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED2);
      // leitura do sensor (0 0 0 1 0 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD ) {
      motorOption('8', SPEED2, SPEED0);

      // leitura do sensor (0 1 1 0 0 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED3);

      // leitura do sensor (0 0 0 1 1 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED3, SPEED0);

      // leitura do sensor (1 1 1 0 0 0)
    } else if (analogRead(A0) <= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED4);
      // leitura do sensor (0 0 0 1 1 1)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) <= TRESHOLD) {
      motorOption('8', SPEED4, SPEED0);

      // leitura do sensor (0 1 0 0 0 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED5);

      // leitura do sensor (0 0 0 0 1 0)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED5, SPEED0);

      // leitura do sensor (1 1 0 0 0 0)
    } else if (analogRead(A0) <= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('8', SPEED0, SPEED6);

      // leitura do sensor (0 0 0 0 1 1)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) <= TRESHOLD) {
      motorOption('8', SPEED6, SPEED0);

      // leitura do sensor (1 0 0 0 0 0)
    } else if (analogRead(A0) <= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD) {
      motorOption('6', SPEED7, SPEED7);
      // leitura do sensor (0 0 0 0 0 1)
    } else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) <= TRESHOLD) {
      motorOption('4', SPEED7, SPEED7);

    }
  }
  motorOption('0', 0, 0);
}

void calcula_PID(){  //Função para calcular o PID conforme o erro.
    if(erro == 0){
    I = 0;
}
    P = erro;
    I = I + erro;
    if(I > 255){
    I=255;
}else if(I < -255){
    I= -255;
}
    D =erro - erro_anterior;
    PID = (Kp*P) + (Ki*I) + (Kd*D);
    erro_anterior = erro;
}