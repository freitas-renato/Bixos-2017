#include "sensors.h"
#include "motors.h"
#include "timer.h"

#define NUMSENSORS 7
#define BRANCO 700
#define VEL_INI 170

int main() {
    float kd, kp, ki; //Constantes PID
    float p = 0.0, i = 0.0, d = 0.0; // :)
    float erro = 0.0, erro_ant = 0.0; 
    int16_t velE, velD;
    int curva = 0;
    int marca_direita = 0;
    uint16_t delta_T = 0, tempo_ant = 0;

    int sensores[NUMSENSORS];


	for (;;) { 

        //Leitura dos sensores
        for (int i = 0; i < NUMSENSORS; i++) {
            sensores[i] = (getLineSensor(i) >= BRANCO); // TRUE se detectar a linha  
        }

        //Sensor 6 indica inicio e fim do circuito
        if (sensores[6] && !sensores[0]) {
            marca_direita++; //1 no inicio, 2 no final
            if (!(marca_direita % 2)) {
                for (;;) {
                    motors (0,0);
                }
            }
        }

        /*Retorna o erro do PID pelo "centro de massa" das leituras dos sensores
            Negativo => virar para a direita (leitura esquerda forte)
            Positivo => virar para a esquerda (leitura direita forte)
            Zero => alinhado (situacao ideal)
        */
        erro = getErro();

        //Deteccao de curva (apenas o sensor 0)
        if (sensores[0] && !sensores[6]) {
            curva = !curva; //1 no inicio da curva, 0 no final
        }

        //P = Kp * (Target - Atual); Target == 0
        //I = Ki * (I + ERRO*TIME) => se continuar errado, o I aumenta com o tempo == melhor ajuste
        //D = Kd * ((ERRO) – (ERRO_ANT))/Time =>taxa de variacao do erro 
        delta_T = getTick() - tempo_ant;       
        p = erro;   i += erro*delta_T;    d = (erro - erro_ant)/delta_T;

        tempo_ant = getTick();
        erro_ant = erro;

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
        
        


		_delay_ms(10);
	}

	return 0;
}
