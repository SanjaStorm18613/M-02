package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Actuators.Arm;
import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Actuators.DriveTrain;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.Detection;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.FieldPosition;

public class Bot {

    private final Gyro gyro;
    private final Detection detector;

    private final DriveTrain driveTrain;
    private final Arm arm;
    private final ServoImplEx claw;
    private final DcMotor rotatMotor;
    private final DcMotor ledCam;

    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    private double yawCorrecao;
    private double giroAnterior;


    public Bot (LinearOpMode opM, HardwareMap hwM, Telemetry tl){

        telemetry  = tl;
        opMode     = opM;

        gyro       = new Gyro(hwM);
        detector   = new Detection(hwM, tl, opMode);

        driveTrain = new DriveTrain(hwM);
        arm        = new Arm(hwM, opM);
        claw       = hwM.get(ServoImplEx.class, "Garra");
        rotatMotor = hwM.get(DcMotorEx.class, "Carrossel");
        ledCam     = hwM.get(DcMotorEx.class, "Cam");

    }


    public void move (double powerMotor, int dist, boolean horMove) {

        PID distPID;
        PID yawPID;

        double encPos;
        double distCorrecao;
        double angulo;

        double forcaMotorInicial = 0.25 * powerMotor;


        driveTrain.relaseEncPos();


        if (horMove){

            encPos = (driveTrain.encodertFE - driveTrain.encodertTE) / 2.0 + dist * 25.5;

        }
        else {

            encPos = (driveTrain.encodertFE + driveTrain.encodertFD) / 2.0 + dist * 21.8;

        }


        yawPID  = new PID(gyro.getAngle(), .05, 5.0, .000001);
        distPID = new PID(encPos, -.001, 50.0, 0);


        while (opMode.opModeIsActive()) {

            driveTrain.relaseEncPos();
            arm.setArmVel(.4);


            angulo = gyro.getContinuosAngle();


            if (horMove){

                encPos = (driveTrain.encodertFE - driveTrain.encodertTE) / 2.0;

            }
            else {

                encPos = (driveTrain.encodertFE + driveTrain.encodertFD) / 2.0;

            }


            yawCorrecao  = yawPID.calcularCorrecao(angulo);
            distCorrecao = distPID.calcularCorrecao(encPos);

            distCorrecao = Math.min(Math.abs(distCorrecao), powerMotor + 0.2) * signum(distCorrecao);


            if (Math.abs(distCorrecao) <= 0.01 && Math.abs(yawCorrecao) <= 0.01) break;


            distCorrecao = distCorrecao * forcaMotorInicial;

            if (forcaMotorInicial < 1) {
                forcaMotorInicial = 0.05 * forcaMotorInicial + forcaMotorInicial;
                forcaMotorInicial = Math.min(forcaMotorInicial, 1);
            }


            if (horMove){

                driveTrain.omniDrive(distCorrecao, 0 , yawCorrecao);

            }
            else {

                driveTrain.arcadeDrive(distCorrecao, yawCorrecao);

            }


            telemetry.update();

        }

        driveTrain.arcadeDrive(0.0, 0.0);
        espere(500);

    }


    public void rotate (double forcaMotor, int angulo) {

        int gyroContAng;

        //ledCam.setPower(0.9);


        PID rotatePID;


        angulo %= 360;

        if (180 < Math.abs(angulo)) {

            angulo = Math.abs(angulo % 180) * -1 * (int) signum(angulo);

        }

        //gyroContAng        = gyro.getContinuosAngle();
        this.giroAnterior += angulo;
        //angulo            += gyroContAng;


        rotatePID = new PID(this.giroAnterior, 0.7, -1.0, 0.00001);


        while (opMode.opModeIsActive()) {

            arm.setArmVel(0.4);


            gyroContAng = (int) gyro.getContinuosAngle();

            //if (
            //    Math.abs(this.giroAnterior - gyroContAng) <= 1 &&
            //    0.5 >= Math.abs(((Gyroscope) gyro.imu)
            //                    .getAngularVelocity(AngleUnit.DEGREES)
             //                   .zRotationRate)
            //) break;


            yawCorrecao = rotatePID.calcularCorrecao(gyroContAng);
            yawCorrecao = Math.min(Math.abs(forcaMotor * yawCorrecao), 1) * yawCorrecao;


            driveTrain.arcadeDrive(0, yawCorrecao);


            //telemetry.addData("Ang_vel", ((Gyroscope) gyro.imu).getAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            //telemetry.addData("Ang_Erro", angulo - gyroContAng);
            //telemetry.addData("Angulo", gyroContAng);

            telemetry.addData("erroKp", String.valueOf(rotatePID.erroKp));
            telemetry.addData("erroKi", String.valueOf(rotatePID.erroKi));
            telemetry.addData("erroKd", String.valueOf(rotatePID.erroKd));
            telemetry.update();

        }

        driveTrain.arcadeDrive(0.0, 0.0);
        //ledCam.setPower(0);

        espere(500);

    }


    private double signum (double val) {
        return Math.signum(val) != 0 ? Math.signum(val) : 1.0;
    }


    public void setArmPos (double forcaMotor, ArmPos pos){

        arm.setArmPos(forcaMotor, pos);

    }


    public void rotationSystem (double vel, int time){

        ElapsedTime tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        rotatMotor.setPower(vel);

        while (opMode.opModeIsActive()) {
            if (time - 1200 < tempo.milliseconds()) {
                rotatMotor.setPower(1 * signum(vel));
            }

            if (time < tempo.milliseconds()){
                rotatMotor.setPower(0);
                break;
            }

        }
    }

    public void rotationSystem (double vel){
        rotatMotor.setPower(vel);
    }


    public void claw (double position){

        claw.setPwmEnable();
        claw.setPosition(position);

        /*
        while (opMode.opModeIsActive()){
            if (status da limit 1 && status da limit 2){
                claw.setPwmDisable();
                break;
            }
            claw.setPosition(position);
        }
        */

    }

    public void espere (int duracao) {

        ElapsedTime tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opMode.opModeIsActive()) {

            arm.setArmVel(0.4);

            if (duracao < tempo.milliseconds()) break;

        }
    }

   public void initVuforia () {
        detector.initVuforia();
    }


    public void stopVuforia (){
        detector.stopVuforia();
    }


    public FieldPosition getFieldPosition () {
        return detector.getBotPosition();
    }


    public void stopDetectionElement (){
        detector.stopDetectionElement();
    }


    public CustomElementLocation getCustomElementLocation (){ return detector.getElementDetection(); }

}