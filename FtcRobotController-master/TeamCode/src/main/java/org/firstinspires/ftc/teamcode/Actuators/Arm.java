package org.firstinspires.ftc.teamcode.Actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    private final DcMotorEx arm;

    public Arm (HardwareMap hwM){

        arm = hwM.get(DcMotorEx.class, "Braco");
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setTargetPositionTolerance(10);

    }

    public void setArmPos(ArmPos pos){

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
                position = 300;
                break;
            case THREE:
                position = 430;
                break;
        }

        arm.setTargetPosition(position);
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
