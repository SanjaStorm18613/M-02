package org.firstinspires.ftc.teamcode.Actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    private final ServoController claw;
    private TouchSensor limitOne, limitTwo;
    private LinearOpMode opMode;

    public Claw(HardwareMap hwM, LinearOpMode opM) {

        opMode = opM;

        claw = hwM.get(ServoController.class, "Control Hub");

        limitOne = hwM.get(TouchSensor.class, "clawRightLimit");
        limitTwo = hwM.get(TouchSensor.class, "clawLeftLimit");

    }

    public void colect(){

        ElapsedTime tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while ((!limitOne.isPressed() && !limitTwo.isPressed() || tempo.milliseconds() >= 1500) && opMode.opModeIsActive()){

            claw.setServoPosition(0, 1.0);

        }

        claw.pwmDisable();

    }

    public void release(){

        claw.pwmEnable();
        claw.setServoPosition(0,0);

    }

}
