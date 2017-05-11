#include "sensors.h"
#include "motors.h"
#include "timer.h"

#define NUMSENSORS 7
#define BRANCO 700
#define VEL_INI 150

int main() {
    float kd, kp, ki; //Constantes PID
    float p = 0.0, i = 0.0, d = 0.0; // :)
    float erro = 0.0, erro_ant = 0.0; 
    int16_t velE, velD;
    int curva = 0;
    int volta = 0;

    int sensores[NUMSENSORS];


	for (;;) { 

        //Leitura dos sensores
        for (int i = 0; i < NUMSENSORS; i++) {
            sensores[i] = (getLineSensor(i) >= BRANCO); // TRUE se detectar a linha  
        }

        //Sensor 6 indica inicio e fim do circuito
        if (sensores[6] && !sensores[0]) {
            volta++; //1 no inicio, 2 no final
            if (volta == 2) {
                for (;;) {
                    motors (0,0);
                }
            }
        }
        
        /*  
        Erro do PID com base na leitura dos sensores 2, 3 e 4 (assumi que os outros não importam para o alinhamento)
            Negativo => virar para a direita
            Positivo => virar para a esquerda
            Zero => alinhado (situacao ideal)
        */
        if (sensores[2] && !sensores[3] && !sensores[4]) { //100
            erro = 2;
        } else
        if (sensores[2] && sensores[3] && !sensores[4]) { //110
            erro = 1;
        } else
        if (!sensores[2] && sensores[3] && !sensores[4]) { //010
            erro = 0;
        } else 
        if (!sensores[2] && sensores[3] && sensores[4]) { //011
            erro = -1;
        } else
        if (!sensores[2] && !sensores[3] && sensores[4]) { //001
            erro = -2;
        }

        //Deteccao de curva (apenas o sensor 0)
        if (sensores[0] && !sensores[6]) {
            curva = !curva; //1 no inicio da curva, 0 no final
        }

        //P = Kp * (Target - Atual); Target == 0
        //I = Ki * (I + ERRO*TIME) => se continuar errado, o I aumenta com o tempo == melhor ajuste
        //D = Kd * ((ERRO) – (ERRO_ANT))/Time =>taxa de variacao do erro        
        p = erro;   i += erro*getTick();    d = (erro - erro_ant)/getTick();

        velD = VEL_INI + ((kp*p) + (ki*i) + (kd*d));
        velE = VEL_INI - ((kp*p) + (ki*i) + (kd*d));
        //velocidades nao podem ultrapassar 255
        if (velE > 255) {
            velE = 255;
        }            
        if (velD > 255) {
            velD = 255;
        }

        if (curva) {
            motors(velE/2, velD/2);
        } else  {
            motors (velE, velD);
        }      
        
        erro_ant = erro;


		_delay_ms(10);
	}

	return 0;
}
