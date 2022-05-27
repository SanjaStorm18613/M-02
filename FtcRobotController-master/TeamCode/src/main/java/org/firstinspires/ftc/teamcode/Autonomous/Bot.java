package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Actuators.Arm;
import org.firstinspires.ftc.teamcode.Actuators.ArmPos;
import org.firstinspires.ftc.teamcode.Actuators.DriveTrain;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.Detection;

public class Bot {

    private final Gyro gyro;
    private final Detection detector;

    private final DriveTrain driveTrain;
    private final Arm arm;
    private final ServoImplEx claw;
    private final DcMotor rotatMotor;

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
        claw.setDirection(Servo.Direction.FORWARD);
        rotatMotor = hwM.get(DcMotorEx.class, "Carrossel");

    }


    public void move (double powerMotor, int dist, boolean horMove) {

        PID distPID;
        PID yawPID;

        double encPos;
        double distCorrecao;
        double angulo;

        double forcaMotorInicial = 0.25 * powerMotor;


        driveTrain.relaseEncPos();


        if (horMove) encPos = (driveTrain.encodertFE - driveTrain.encodertTE) / 2.0 + dist * 25.5;
        else         encPos = (driveTrain.encodertFE + driveTrain.encodertFD) / 2.0 + dist * 21.8;


        yawPID  = new PID(gyro.getAngle(), .04, 0, 0);
        distPID = new PID(encPos, .0045, 0, 0);


        while (opMode.opModeIsActive()) {

            driveTrain.relaseEncPos();

            angulo = gyro.getContinuosAngle();


            if (horMove){

                encPos = (driveTrain.encodertFE - driveTrain.encodertTE) / 2.0;

            }
            else {

                encPos = (driveTrain.encodertFE + driveTrain.encodertFD) / 2.0;

            }


            yawCorrecao  = yawPID.calcularCorrecao(angulo);
            distCorrecao = distPID.calcularCorrecao(encPos);

            telemetry.addData("yaw", yawCorrecao);


            if (Math.abs(distCorrecao) <= 0.1 && Math.abs(yawCorrecao) <= 0.1) break;


            distCorrecao *= forcaMotorInicial;

            distCorrecao = Math.min(Math.abs(distCorrecao), powerMotor) * signum(distCorrecao);


            if (forcaMotorInicial < powerMotor) {
                forcaMotorInicial = 0.08 * forcaMotorInicial + forcaMotorInicial;
                forcaMotorInicial = Math.min(forcaMotorInicial, powerMotor);
            }

            telemetry.addData("dist", distCorrecao);
            telemetry.update();

            if (horMove){

                driveTrain.omniDrive(-distCorrecao, 0 , -yawCorrecao);
                //driveTrain.arcadeDrive(0.0, 0.0);


            }
            else {

                driveTrain.arcadeDrive(-distCorrecao, -yawCorrecao);
                //driveTrain.arcadeDrive(0.0, 0.0);

            }

            telemetry.update();

        }

        driveTrain.arcadeDrive(0.0, 0.0);
        espere(500);

    }

    public void move (double powerMotor, int time) {

        PID yawPID;

        double angulo;


        yawPID  = new PID(gyro.getAngle(), .04, 0, 0);

        ElapsedTime tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opMode.opModeIsActive()) {

            angulo = gyro.getContinuosAngle();


            yawCorrecao  = yawPID.calcularCorrecao(angulo);


            if (time < tempo.milliseconds()) break;

            driveTrain.arcadeDrive(powerMotor, 0);//-yawCorrecao);

        }

        driveTrain.arcadeDrive(0.0, 0.0);
        espere(500);

    }

    public void rotate (double forcaMotor, int angulo) {

        int gyroContAng;

        PID rotatePID;


        angulo %= 360;

        if (180 < Math.abs(angulo)) {

            angulo = Math.abs(angulo % 180) * -1 * (int) signum(angulo);

        }

        //gyroContAng        = gyro.getContinuosAngle();
        this.giroAnterior += angulo;
        //angulo            += gyroContAng;


        rotatePID = new PID(this.giroAnterior, 0.03, 0.0, 0.00001);


        while (opMode.opModeIsActive()) {



            gyroContAng = (int) gyro.getContinuosAngle();

            if (
                Math.abs(this.giroAnterior - gyroContAng) <= 3 &&
                0.5 >= Math.abs(((Gyroscope) gyro.imu)
                                .getAngularVelocity(AngleUnit.DEGREES)
                                .zRotationRate)
            ) break;


            yawCorrecao = rotatePID.calcularCorrecao(gyroContAng);
            yawCorrecao = Math.min(Math.abs(forcaMotor * yawCorrecao), 1) * yawCorrecao;


            telemetry.addData("yaw", -yawCorrecao);

            driveTrain.arcadeDrive(0, -yawCorrecao);

            telemetry.addData("Angulo", gyroContAng);
            telemetry.addData("erroKp", String.valueOf(rotatePID.erroKp));
            telemetry.addData("erroKi", String.valueOf(rotatePID.erroKi));
            telemetry.addData("erroKd", String.valueOf(rotatePID.erroKd));
            telemetry.update();

        }

        driveTrain.arcadeDrive(0.0, 0.0);
        espere(500);

    }


    private double signum (double val) {
        return Math.signum(val) != 0 ? Math.signum(val) : 1.0;
    }


    public void setArmPos (ArmPos pos){

        arm.setArmPos(pos);

    }


    public void rotationSystem (double vel, int time){

        ElapsedTime tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        rotatMotor.setPower(vel);

        while (opMode.opModeIsActive()) {

            if (time < tempo.milliseconds()) break;

        }

        rotatMotor.setPower(0);

    }


    public void claw (double position){

        claw.setPosition(position);

    }

    public void espere (int duracao) {

        ElapsedTime tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opMode.opModeIsActive()) {

            if (duracao < tempo.milliseconds()) break;

        }
    }


    public void stopDetectionElement (){
        detector.stopDetectionElement();
    }


    public CustomElementLocation getCustomElementLocation (){ return detector.getElementDetection(); }

}
