package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;


@Autonomous(name="BotAuto", group="Linear Opmode")

public class BotAuto extends LinearOpMode {

    private Bot bot;

    private CustomElementLocation locElement;

    private final int move1  = 10;
    private final int giro1  = 52;
    private final int move2  = 83;
    private final int giro2  = 35;

    private final int giro3A = -giro1 + giro2;
    private final int move3A = 33;
    private final int move4A = 120;

    private final int giro3CR = 164 - (giro1 - giro2);
    private final int giro3CB = 13 - (-giro1 + giro2);
    private final int move3C = 113;
    private final int move4C = 10;
    private final int move5C = 64;

    private final double moveVel    = 0.8;
    private final double rotateVel  = 0.5;

    private TouchSensor alliance;
    private TouchSensor position;

    @Override
    public void runOpMode() {

        alliance = hardwareMap.get(TouchSensor.class, "alliance");
        position = hardwareMap.get(TouchSensor.class, "position");

        telemetry.addData("Status", "Starting");
        telemetry.update();

        locElement = CustomElementLocation.NOT_FOUND;

        bot        = new Bot(this, hardwareMap, telemetry);


        while (!isStarted()) {

            locElement = bot.getCustomElementLocation();

            telemetry.addData("Status", "Waiting for start: Detectando...");
            telemetry.addData("Posição do elemento detectada", locElement);
            telemetry.addData("Aliança", alliance.isPressed() ? "Azul" : "Vermelha");
            telemetry.addData("Posição", position.isPressed() ? "Armazem" : "Carrossel");
            telemetry.update();

        }

        bot.stopDetectionElement();


        waitForStart();


        if (opModeIsActive()) {

            //ENTREGA
            botShipping(alliance.isPressed() ? (position.isPressed()? 1 : -1) : (position.isPressed()? -1 : 1));

            if (alliance.isPressed()) {

                if (position.isPressed()) autoBStorage();
                else autoBWall();

            } else {

                if (position.isPressed()) autoRStorage();
                else autoRWall();

            }
        }
    }


    private void botShipping (int dir) {

        int posDistRedu;

        bot.claw(0.365);
        bot.espere(1000);


        //LAVANTA NIVEL DETECTADO
        if (locElement == CustomElementLocation.CENTER) {

            posDistRedu = 17;
            bot.setArmPos(ArmPos.TWO);

        }
        else if (locElement == CustomElementLocation.RIGHT) {

            posDistRedu = 6;
            bot.setArmPos(ArmPos.THREE);

        }
        else {

            posDistRedu = 15;
            bot.setArmPos(ArmPos.ONE);

        }

        bot.espere(500);

        //AVANÇA
        bot.move(moveVel, move1, false);

        //GIRA
        bot.rotate(rotateVel, dir * giro1);

        //AVANÇA
        bot.move(moveVel, move2 - posDistRedu, false);

        //GIRA
        bot.rotate(rotateVel, -dir * giro2);

        //LIBERA PEÇA
        bot.claw(0.9);
        bot.espere(1000);

        //VOLTA
        bot.move(moveVel, 47 - move2 + posDistRedu, false);

        //ABAIXA
        bot.setArmPos(ArmPos.ONE);

    }


    private void autoBStorage (){

        //GIRA
        bot.rotate(rotateVel, giro3A - 2);

        //AVANÇA
        bot.move(moveVel, -move3A, false);

        //AVANÇA LATERAL
        bot.move(moveVel, -move4A, true);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

    }


    private void autoRStorage (){

        //GIRA
        bot.rotate(rotateVel, -giro3A + 2);

        //AVANÇA
        bot.move(moveVel, -move3A, false);

        //AVANÇA LATERAL
        bot.move(moveVel, move4A, true);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

    }


    private void autoBWall (){

        //GIRA
        bot.rotate(rotateVel, giro3CB);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

        bot.espere(500);

        //AVANÇA LATERAL
        bot.move(moveVel, move3C, true);

        //GIRA CARROSSEL
        bot.rotationSystem(0.25, 2000);

        //VOLTA LATERAL
        bot.move(moveVel, -move4C, true);

        //AVANÇA
        bot.move(moveVel, move5C, false);

        //GIRA
        bot.rotate(rotateVel, -(-giro1 + giro2 + giro3CB));

    }


    private void autoRWall (){

        //GIRA
        bot.rotate(rotateVel, giro3CR);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

        //AVANÇA LATERAL
        bot.move(moveVel, move3C, true);

        //GIRA CARROSSEL
        bot.rotationSystem(-0.25, 2000);

        //VOLTA LATERAL
        bot.move(moveVel, -move4C, true);

        //AVANÇA
        bot.move(moveVel, -move5C, false);

        //GIRA
        bot.rotate(rotateVel, -((giro1 - giro2) + (180 - giro3CR) - 180));

    }
}
