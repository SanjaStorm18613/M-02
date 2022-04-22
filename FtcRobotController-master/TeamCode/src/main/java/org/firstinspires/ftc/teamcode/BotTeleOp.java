package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.Actuators.Arm;
import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Actuators.Claw;
import org.firstinspires.ftc.teamcode.Actuators.DriveTrain;

@TeleOp(name="BotTeleOp", group="Linear Opmode")
@Disabled

public class BotTeleOp extends LinearOpMode {

    DriveTrain driveTrain;
    Arm arm;
    Claw claw;
    private DcMotor carousel;

    
    private boolean upPos      = true;
    private boolean downPos    = true;
    private boolean moveClaw   = true;
    private boolean collect    = false;

    double slow;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Starting");
        telemetry.update();

        driveTrain = new DriveTrain(hardwareMap);
        arm        = new Arm(hardwareMap);
        claw       = new Claw(hardwareMap, this);
        carousel   = hardwareMap.get(DcMotor.class, "Carrossel");

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                slow = 1 - gamepad1.right_trigger;

                driveTrain.omniDrive(gamepad1.left_stick_x * slow, gamepad1.left_stick_y * slow, gamepad1.right_stick_x);

                if (gamepad1.left_bumper && upPos){

                    /*
                    switch (arm.armPos){

                        case THREE:
                            arm.setArmPos(0.4, ArmPos.TWO);
                            break;
                        case TWO:
                            arm.setArmPos(0.4, ArmPos.ONE);
                            break;
                        case ONE:
                        default:
                            arm.setArmPos(0.4, ArmPos.COLECT);
                            break;

                    }

                }
                upPos = !gamepad1.left_bumper;

                if (gamepad1.right_bumper && downPos){

                    switch (arm.armPos){

                        case COLECT:
                            arm.setArmPos(0.4, ArmPos.ONE);
                            break;
                        case ONE:
                            arm.setArmPos(0.4, ArmPos.TWO);
                            break;
                        case TWO:
                        default:
                            arm.setArmPos(0.4, ArmPos.THREE);
                            break;

                    }

                     */

                }
                downPos = !gamepad1.right_bumper;

                arm.setArmVel(0.4);

                if (gamepad1.a && moveClaw){

                    if (collect) claw.release();
                    else claw.colect();

                }

                moveClaw = !gamepad1.a;

                if (gamepad1.x) carousel.setPower(.2 + gamepad1.left_trigger);
                else if (gamepad1.b) carousel.setPower(-(.2 + gamepad1.left_trigger));
                else carousel.setPower(0);



            }
        }
    }
}

    /*private void ralativeMove () {

        double stickVeloc;
        double angCord;
        double angResult;
        double yAngResult;
        double xAngResult;

        double gpX = gamepad1.left_stick_x;
        double gpY = gamepad1.left_stick_y;
        int botAngle = (int) angleImu();


        stickVeloc = (Math.abs(gpX) + Math.abs(gpY)) * 1 - gamepad1.right_trigger;

        angCord = (gpX + 1) * 90 * signum(gpY);

        if (botAngle < 0) {

            if (botAngle < angCord - 180) angResult = (angCord - botAngle) - 360;
            else angResult = angCord - botAngle;

        }
        else angResult = angCord - botAngle;


        yAngResult = (1 - Math.abs(angResult) / 90) * stickVeloc;
        xAngResult = signum(angResult) * (1 - Math.abs(yAngResult)) * stickVeloc;

            driveTrain.omniDrive(xAngResult, yAngResult, gamepad1.right_stick_x);

    }


    private double signum(double var) {
        return (var == 0 ? 1.0 : Math.signum(var));
    }


    private void initImu() {

        BNO055IMU.Parameters GYRO_imu_parameters;

        GYRO_imu_parameters                = new BNO055IMU.Parameters();
        GYRO_imu_parameters.mode           = BNO055IMU.SensorMode.IMU;
        GYRO_imu_parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        GYRO_imu_parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        GYRO_imu_parameters.loggingEnabled = false;

        imu2.initialize(GYRO_imu_parameters);

    }


    private float angleImu() {

        return -imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }*/
