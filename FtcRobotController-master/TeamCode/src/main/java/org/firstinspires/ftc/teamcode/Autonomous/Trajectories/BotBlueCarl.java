package org.firstinspires.ftc.teamcode.Autonomous.Trajectories;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Autonomous.Bot;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.FieldPosition;


@Autonomous(name="BotBlueCarl", group="Linear Opmode")

public class BotBlueCarl extends LinearOpMode {

    private Bot bot;

    private CustomElementLocation locElement;

    private final double moveVel    = 0.9;
    private final double rotateVel  = 0.55;
    private final double armVel     = 0.4;

    private final int clawStop      = 1000;
    private final int rotatSysTime = 3000;

    private final int shipgDuckMove = 30;

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

                bot.setArmPos(armVel, ArmPos.TWO);

            }
            else if (locElement == CustomElementLocation.RIGHT) {

                bot.setArmPos(armVel, ArmPos.THREE);

            }
            else {

                bot.setArmPos(armVel, ArmPos.ONE);

            }

            //AVANÇA
            bot.move(moveVel, 50, false);

            //GIRA
            bot.rotate(rotateVel,-53);

            //AVANÇA
            bot.move(moveVel, 28, false);

            //LIBERA PEÇA
            bot.claw(0.65);
            bot.espere(clawStop);

            //VOLTA
            bot.move(moveVel, -28, false);

            //ABAIXA
            bot.setArmPos(armVel, ArmPos.COLECT);

            //GIRA
            bot.rotate(rotateVel,92);

            //AVANÇA LATERAL
            bot.move(moveVel, 79, true);

            //GIRA CARROSSEL
            bot.rotationSystem(-0.2, rotatSysTime);

            //VOLTA LATERAL
            bot.move(moveVel, -42, true);

            //AVANÇA
            bot.move(moveVel, 50, false);

            //GIRA
            bot.rotate(rotateVel,-36);

        }
    }
}
