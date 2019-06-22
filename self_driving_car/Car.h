#ifndef Car_h
#define Car_h

#include <PID_v1.h>

struct Car{
    public:
    bool go = false;   //Flase in partenza, True al verde del semaforo
    int kp, kd, ki;  // Parametri PID
    float setpoint, distance, error; // Setmpoint= disyanze, Distanza,
    int absolute_speed; //Velocità
    float v_max;   // Percentuale di potenza per movimento rettilineo
    PID controller;  // controllore PID

    // PIN per i MOTORI
    // motor one
    const int enA = 10;
    const int in1 = 9;
    const int in2 = 8;

    // motor two
    const int enB = 5;
    const int in3 = 7;
    const int in4 = 6;

    // Sonar
    const int trigger = 11;
    const int echo = 12;

    // Infrarossi
    const int vout = 0;

    // FUNZIONI
    void set_arduino_pin();  // Setta i pin usati da Arduino
    void start();   // Legge sensore infrarosso e setta la variable start alla partenza
    void update_distance();  //Legge sesore di prossimità e aggiorna il valore di distanza
    void set_power(float y);  //Data l'uscita y del controllore (valore tra -1 e 1) aggiorna potenza dei motori
    void get_controller_output();  //Legge distanza calcola errore e restituisce la variabile di controllo (bilaciamento ruote dx e sx)
};

#endif
