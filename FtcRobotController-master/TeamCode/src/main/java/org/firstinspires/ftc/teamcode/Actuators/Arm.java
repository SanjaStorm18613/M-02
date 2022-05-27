package org.firstinspires.ftc.teamcode.Actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {

    private final DcMotorEx arm;
    private LinearOpMode opM;
    private TouchSensor armLimit;

    public Arm (HardwareMap hwM, LinearOpMode opM){

        this.opM = opM;
        arm = hwM.get(DcMotorEx.class, "Braco");
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setTargetPositionTolerance(10);

        armLimit = hwM.get(TouchSensor.class, "armLimit");

    }

    public void setArmPos(ArmPos pos){

        int position;
        int erroMov = 0;

        switch (pos){
            case COLECT:
            default:
                position = 0;
                break;
            case ONE:
                position = 150;
                break;
            case TWO:
                position = 270;
                break;
            case THREE:
                position = 460;
                break;
        }
        if (arm.getTargetPosition() == 0 && !armLimit.isPressed()){

            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(0.4 * 2000);

            while (opM.opModeIsActive()){

                if (armLimit.isPressed()) break;
                else erroMov = arm.getCurrentPosition();

            }
        }

        arm.setTargetPosition(position + erroMov);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(0.4 * 2000);

    }
/*
    public void setArmVel(double forcaMotor){

        if (arm.getCurrentPosition() <= 50 && arm.getTargetPosition() == 0){

            arm.setVelocity(0);

        }
        else if (arm.getCurrentPosition() < arm.getTargetPosition() + 10 &&
                 arm.getCurrentPosition() > arm.getTargetPosition() - 10) {

            arm.setVelocity(350);

        }
        else {

            arm.setVelocity(forcaMotor * 2000.0);

        }

    }
*/
}
