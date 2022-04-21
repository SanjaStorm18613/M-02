package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.FieldPosition;


@Autonomous(name="AndroidAuto", group="Linear Opmode")

public class BotRedCarl extends LinearOpMode {

    private Bot bot;

    private CustomElementLocation locElement;

    private final double moveVel    = 0.6;
    private final double rotateVel  = 0.5;
    private final double armVel     = 0.4;

    private final int clawStop      = 1000;
    private final int rotatSysTime = 3000;

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

            //LEVANTA BRAÇO
            bot.setArmPos(armVel, ArmPos.ONE);

            //AVANÇA
            bot.move(moveVel, 60, false);

            //GIRA
            bot.rotate(rotateVel,-140);


            //LAVANTA NIVEL DETECTADO
            if (locElement == CustomElementLocation.CENTER) {

                bot.setArmPos(armVel, ArmPos.TWO);

            }
            else if (locElement == CustomElementLocation.RIGHT) {

                bot.setArmPos(armVel, ArmPos.THREE);

            }

            //AVANÇA
            bot.move(moveVel, botShipgMove, false);

            //LIBERA PEÇA
            bot.claw(0.65);
            bot.espere(clawStop);

            //VOLTA
            bot.move(moveVel, -botShipgMove, false);

            //ABAIXA
            bot.setArmPos(armVel, ArmPos.ONE);

            //ABAIXA
            bot.setArmPos(armVel, ArmPos.COLECT);

            /*
            //GIRA
            bot.rotate(rotateVel,-128);

            //AVANÇA LATERAL
            bot.move(moveVel, -shipgDuckMove, true);

            //GIRA CARROSSEL
            bot.rotationSystem(0.2, rotatSys);

            //VOLTA LATERAL
            bot.move(moveVel, shipgDuckMove - 30, true);

            //VOLTA
            bot.move(moveVel, -50, false);
             */

            //GIRA
            bot.rotate(rotateVel,-34);

            //VOLTA
            bot.move(moveVel, -75, false);

            //AVANÇA LATERAL
            bot.move(moveVel, -shipgDuckMove, true);

            //GIRA CARROSSEL
            bot.rotationSystem(-0.2, rotatSysTime);

            //VOLTA LATERAL
            bot.move(moveVel, shipgDuckMove + 10, true);

        }
    }
}
