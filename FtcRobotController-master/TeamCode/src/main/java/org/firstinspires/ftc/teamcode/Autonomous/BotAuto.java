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

    private final int move1  = 50;
    private final int giro1  = 53;
    private final int move2  = 28;

    private final int giro2A = 90 - giro1;
    private final int move3A = -100;

    private final int giro2C = 92;
    private final int move3C = 70;
    private final int move4C = 30;
    private final int move5C = 35;

    private final double moveVel    = 0.6;
    private final double rotateVel  = 0.5;
    private final double armVel     = 0.4;

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

            telemetry.addData("Status", "Detecting");
            telemetry.addData("Position Element Detected", locElement);
            telemetry.update();

        }

        bot.stopDetectionElement();

        telemetry.addData("Status", "Waiting for start");
        telemetry.addData("Position Element Detected", locElement);
        telemetry.update();


        waitForStart();


        if (opModeIsActive()) {

                if (alliance.isPressed()){
                    if (position.isPressed()) autoBStorage();
                    else autoBWall();
                }
                else {
                    if (position.isPressed()) autoRStorage();
                    else autoRWall();
                }

            }
        }


    private void botShipping (int dir) {

        int posDistRed;

        //LAVANTA NIVEL DETECTADO
        if (locElement == CustomElementLocation.CENTER) {

            posDistRed = 10;
            bot.setArmPos(ArmPos.TWO);

        }
        else if (locElement == CustomElementLocation.RIGHT) {

            posDistRed = 0;
            bot.setArmPos(ArmPos.THREE);

        }
        else {

            posDistRed = 5;
            bot.setArmPos(ArmPos.ONE);

        }

        //AVANÇA
        bot.move(moveVel, move1, false);

        //GIRA
        bot.rotate(rotateVel, dir * giro1);

        //AVANÇA
        bot.move(moveVel, move2 - posDistRed, false);

        //LIBERA PEÇA
        bot.claw(0.8);
        bot.espere(1000);

        //VOLTA
        bot.move(moveVel, -move2 + posDistRed, false);

        //ABAIXA
        bot.setArmPos(ArmPos.ONE);

    }


    private void autoBStorage (){

        //ENTREGA
        botShipping(1);

        //GIRA
        bot.rotate(rotateVel, giro2A);

        //AVANÇA
        bot.move(moveVel, move3A, false);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

    }


    private void autoRStorage (){

        //ENTREGA
        botShipping(-1);

        //GIRA
        bot.rotate(rotateVel, -giro2A);

        //AVANÇA
        bot.move(0.4, move3A, false);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

    }


    private void autoBWall (){

        //ENTREGA
        botShipping(-1);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

        //GIRA
        bot.rotate(rotateVel, giro2C);

        //VOLTA
        bot.move(moveVel, move3C, false);

        //AVANÇA LATERAL
        bot.move(moveVel, move4C, true);

        //GIRA CARROSSEL
        bot.rotationSystem(-0.2, 2000);

        //VOLTA LATERAL
        bot.move(moveVel, move5C, true);

        //GIRA
        bot.rotate(rotateVel, -(giro1 + giro2C));

    }


    private void autoRWall (){

        //ENTREGA
        botShipping(1);

        //ABAIXA
        bot.setArmPos(ArmPos.COLECT);

        //GIRA
        bot.rotate(rotateVel, 180 - giro2C);

        //VOLTA
        bot.move(moveVel, move3C, false);

        //AVANÇA LATERAL
        bot.move(moveVel, move4C, true);

        //GIRA CARROSSEL
        bot.rotationSystem(0.2, 2000);

        //VOLTA LATERAL
        bot.move(moveVel, -move5C, true);

        //GIRA
        bot.rotate(rotateVel, -(giro1 + (180 - giro2C)));

    }
}
