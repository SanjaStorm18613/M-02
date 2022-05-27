package org.firstinspires.ftc.teamcode.Actuators;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveTrain {

    private final DcMotorEx[] motors;
    private final double coef;

    public int encodertFE;
    public int encodertFD;
    public int encodertTE;
    public int encodertTD;


    public DriveTrain(HardwareMap hwM){

        motors = new DcMotorEx[] {
                hwM.get(DcMotorEx.class, "TracaoFE"),
                hwM.get(DcMotorEx.class, "TracaoFD"),
                hwM.get(DcMotorEx.class, "TracaoTE"),
                hwM.get(DcMotorEx.class, "TracaoTD")
        };

        motors[0].setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);
        motors[3].setDirection(DcMotorEx.Direction.REVERSE);


        for (DcMotorEx mt: motors) {

            mt.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            mt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        }


        coef = 1800.0;
        relaseEncPos();
    }


    public void drive(double fE, double fD, double tE, double tD){

        motors[0].setVelocity(fE * coef);
        motors[1].setVelocity(fD * coef);
        motors[2].setVelocity(tE * coef);
        motors[3].setVelocity(tD * coef);

    }


    public void arcadeDrive(double powerMotor, double gyro){

        //tankDrive(powerMotor, powerMotor + gyro, powerMotor - gyro);
        omniDrive(0, powerMotor, gyro);

    }


    public void tankDrive(double left, double right){

        //arcadeDrive((left + right)/2, (left - right)/2);
        omniDrive(0,(left + right)/2.0, (left - right)/2.0);

    }


    public void omniDrive(double moveX, double moveY, double gyro){

        drive(  (moveY + moveX + gyro),
                (moveY - moveX - gyro),
                (moveY - moveX + gyro),
                (moveY + moveX - gyro));

    }


    public void relaseEncPos(){

        this.encodertFE = motors[0].getCurrentPosition();
        this.encodertFD = motors[1].getCurrentPosition();
        this.encodertTE = motors[2].getCurrentPosition();
        this.encodertTD = motors[3].getCurrentPosition();

    }
}
