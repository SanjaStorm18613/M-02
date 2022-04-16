package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    private final double Kp, Kd, Ki, setpoint;
    private double erro, erroAnterior, erroAcumulado;

    private final ElapsedTime tempo;

    public double erroKp, erroKi, erroKd;

    public PID (double sp, double kp, double kd, double ki) {

        Kp = kp;
        Kd = kd;
        Ki = ki;
        setpoint = sp;

        erro = 0;
        erroAnterior = 0;
        erroAcumulado = 0;
        tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        tempo.reset();

    }


    public double calcularCorrecao (double posAtual){

        double resultado, tempoAtual;

        tempoAtual = tempo.milliseconds();

        erro = posAtual - setpoint;

        erroKp = erro * Kp;
        erroKi = erroAcumulado * Ki;
        erroKd = Kd * ((erroAnterior - erro) / tempoAtual);


        resultado = erroKp + erroKi + erroKd;


        if (1 > Math.abs(resultado)) {

            erroAcumulado = erroAcumulado + ((erroAnterior + erro) / 2) * tempoAtual;

        }

        erroAnterior = erro;

        tempo.reset();


        return resultado;

    }

}
