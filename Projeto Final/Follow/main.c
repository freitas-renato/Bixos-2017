#include "sensors.h"
#include "motors.h"
#include "timer.h"

#define NUMSENSORS 7
#define BRANCO 700
#define VEL_INI 150

int main() {
    float Kd, Kp, Ki; //Constantes PID
    float P = 0.0, I = 0.0, D = 0.0; // :)
    float erro = 0.0, erro_ant = 0.0; 
    int16_t velE, velD;

    bool sensores[NUMSENSORS];


	for (;;) { 

        //Leitura dos sensores
        for (int i = 0; i <= 6; i++) {
            sensores[i] = (getLineSensor(i) >= BRANCO); // TRUE se detectar a linha  
        }
        
        /*  
        Erro do PID com base na leitura dos sensores 
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

        //P = Kp * (Target - Atual); Target == 0
        //I = Ki * (I + ERRO*TIME) => se continuar errado, o I aumenta com o tempo == melhor ajuste
        //D = Kd * ((ERRO) – (ERRO_ANT))/Time =>taxa de variacao do erro        
        P = erro;   I += erro*getTick();    D = (erro - erro_ant)/getTick();

        velD = VEL_INI + ((Kp*P) + (Ki*I) + (Kd*D));
        velE = VEL_INI - ((Kp*P) + (Ki*I) + (Kd*D));
        //velocidades nao podem ultrapassar 255
        if (velE > 255) {
            velE = 255;
        }            
        if (velD > 255) {
            velD = 255;
        }
        motors (velE, velD);

        erro_ant = erro;





		_delay_ms(10);
	}

	return 0;
}
