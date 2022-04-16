package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.FieldPosition;


@Autonomous(name="AndroidAuto", group="Linear Opmode")

public class BotAuto extends LinearOpMode {

    private Bot bot;

    private CustomElementLocation locElement;

    private final double moveVel    = 0.6;
    private final double rotateVel  = 0.5;
    private final double armVel     = 0.4;

    private final int clawStop      = 1000;
    private final int rotatSys      = 3000;

    private final int botShipgMove  = 37;
    private final int shipgDuckMove = 45;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Starting");
        telemetry.update();
        FieldPosition fieldPosition;

        locElement = CustomElementLocation.NOT_FOUND;

        bot        = new Bot(this, hardwareMap, telemetry);

        /*
        while (!isStarted() && locElement == CustomElementLocation.NOT_FOUND) {

            locElement = bot.getCustomElementLocation();

            telemetry.addData("Status", "Detecting");
            telemetry.update();

        }

         */


        telemetry.addData("Status", "Starting Vuforia");
        telemetry.addData("Position Element Detected", locElement);
        telemetry.update();

        //bot.stopDetectionElement();

        //bot.initVuforia();
        telemetry.addData("Status", "Waiting for start");
        telemetry.addData("Vuforia", "Complete");
        telemetry.addData("Position Element Detected", locElement);
        telemetry.update();


        waitForStart();


        if (opModeIsActive()) {

            while (opModeIsActive()){

                bot.rotate(rotateVel, 0);

            }
            /*

            bot.claw(.4);
            bot.espere(clawStop);

            bot.setArmPos(armVel, ArmPos.ONE);

            bot.move(moveVel, 60, false);

            bot.rotate(rotateVel, 85);


            fieldPosition = bot.getFieldPosition();


            if (fieldPosition == FieldPosition.NOT_FOUND) {

                bot.rotate(rotateVel, 95);

                fieldPosition = bot.getFieldPosition();
                fieldPosition = fieldPosition == FieldPosition.NOT_FOUND ?
                                                    FieldPosition.R_WALL : fieldPosition;
            }

            bot.stopVuforia();


            switch (fieldPosition) {

                case B_STORAGE:
                    autoBStorage();
                    break;

                case R_STORAGE:
                    autoRStorage();
                    break;

                case B_WALL:
                    autoBWall();
                    break;

                case R_WALL:
                    autoRWall();
                    break;

            }
        */
        }

    }


    private void botShipping () {

        if (locElement == CustomElementLocation.CENTER) {

            bot.setArmPos(armVel, ArmPos.TWO);

        }
        else if (locElement == CustomElementLocation.RIGHT) {

            bot.setArmPos(armVel, ArmPos.ONE);

        }


        bot.move(moveVel, botShipgMove, false);

        bot.claw(0.65);
        bot.espere(clawStop);

        bot.move(moveVel, -botShipgMove, false);

        bot.setArmPos(armVel, ArmPos.ONE);

    }


    private void autoBStorage (){

        bot.rotate(rotateVel,-122);

        botShipping();

        bot.rotate(rotateVel,35);

        bot.move(moveVel, -100, false);
    }


    private void autoRStorage (){

        bot.rotate(rotateVel,122);

        botShipping();

        bot.rotate(rotateVel,-35);

        bot.move(moveVel, -100, false);
    }


    private void autoBWall (){

        bot.rotate(rotateVel,-140);

        botShipping();

        bot.setArmPos(armVel, ArmPos.COLECT);

        bot.rotate(rotateVel,-34);

        bot.move(moveVel, -75, false);

        bot.move(moveVel, -shipgDuckMove, true);

        bot.rotationSystem(-0.2, rotatSys);

        bot.move(moveVel, shipgDuckMove + 10, true);

    }


    private void autoRWall (){

        bot.rotate(rotateVel,-128);

        botShipping();

        bot.setArmPos(armVel, ArmPos.COLECT);

        bot.rotate(rotateVel,-100);

        bot.move(moveVel, -shipgDuckMove, true);

        bot.rotationSystem(0.2, rotatSys);

        bot.move(moveVel, shipgDuckMove - 30, true);

        bot.move(moveVel, -50, false);

    }

}
