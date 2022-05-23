package org.firstinspires.ftc.teamcode.Autonomous.Trajectories;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Autonomous.Bot;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;


@Autonomous(name="BotBlueStorage", group="Linear Opmode")

public class BotBlueStorage extends LinearOpMode {

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

            //ABRE GARRA
            bot.claw(.4);
            bot.espere(clawStop);

            //LAVANTA NIVEL DETECTADO
            if (locElement == CustomElementLocation.CENTER) {

                bot.setArmPos(ArmPos.TWO);

            }
            else if (locElement == CustomElementLocation.RIGHT) {

                bot.setArmPos(ArmPos.THREE);

            }
            else {

                bot.setArmPos(ArmPos.ONE);

            }

            //AVANÇA
            bot.move(moveVel, 60, false);

            //GIRA
            bot.rotate(rotateVel,60);

            //AVANÇA
            bot.move(moveVel, 29, false);

            //LIBERA PEÇA
            bot.claw(0.65);
            bot.espere(clawStop);

            //VOLTA
            bot.move(moveVel, -25, false);

            //ABAIXA
            bot.setArmPos(ArmPos.ONE);

            //GIRA
            bot.rotate(rotateVel,30);

            //VOLTA
            bot.move(moveVel, -140, false);

            //ABAIXA
            bot.setArmPos(ArmPos.COLECT);

        }
    }
}
