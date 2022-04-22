package org.firstinspires.ftc.teamcode.Actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {

    private final TouchSensor armLimit;
    private final DcMotorEx arm;

    private boolean limitPress;


    public Arm (HardwareMap hwM){


        limitPress = false;
        armLimit = hwM.get(TouchSensor.class, "armLimit");

        arm = hwM.get(DcMotorEx.class, "Braco");
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setTargetPositionTolerance(10);

    }

    public void setArmPos(double forcaMotor, ArmPos pos){

        int position;

        switch (pos){
            case COLECT:
            default:
                position = 0;
                break;
            case ONE:
                position = 120;
                break;
            case TWO:
                position = 250;
                break;
            case THREE:
                position = 400;
                break;
        }


        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(forcaMotor * 2000);

    }

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


        if (limitPress && arm.getTargetPosition() == 0 && armLimit.isPressed()){

            arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        limitPress = !armLimit.isPressed();

    }

}
