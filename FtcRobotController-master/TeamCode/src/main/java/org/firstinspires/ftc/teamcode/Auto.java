package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.PID;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.CustomElementLocation;
import org.firstinspires.ftc.teamcode.Autonomous.Detection.SquareLocationDetectorOpenCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;


@Autonomous(name="TesteAndroid", group="Linear Opmode")
@Disabled

public class Auto extends LinearOpMode {

    private BNO055IMU imu2;
    private DcMotor TracaoTD;
    private DcMotor TracaoTE;
    private DcMotor TracaoFE;
    private DcMotor TracaoFD;

    double GYRO_angulo_atual;

    double GYRO_angulo_anterior;
    double angulo_real;
    double yawCorrecao;
    double GIRAR_giroAnterior;
    int GYRO_revolu_C3_A7_C3_B5es;

    double forca_motor;

    CustomElementLocation resultDetection = CustomElementLocation.NOT_FOUND;

    ElapsedTime tempo;


    @Override
    public void runOpMode() {

        OpenCvWebcam webcam;

        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");

        TracaoTD = hardwareMap.get(DcMotor.class, "TracaoTD");
        TracaoTE = hardwareMap.get(DcMotor.class, "TracaoTE");
        TracaoFE = hardwareMap.get(DcMotor.class, "TracaoFE");
        TracaoFD = hardwareMap.get(DcMotor.class, "TracaoFD");

        telemetry.addData("Status", "Iniciando");
        telemetry.update();

        GYRO_inicializar();
        InitMotors();


        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources()
                .getIdentifier
                        ("cameraMonitorViewId"
                                , "id"
                                , hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam
                        (hardwareMap.get
                                        (WebcamName.class, "Webcam 1")
                                , cameraMonitorViewId);


        SquareLocationDetectorOpenCV detector = new SquareLocationDetectorOpenCV(this.telemetry);
        webcam.setPipeline(detector);


        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


                ExposureControl ec = webcam.getExposureControl();
                ec.setMode(ExposureControl.Mode.Manual);
                ec.setExposure(1, TimeUnit.MILLISECONDS);
                webcam.showFpsMeterOnViewport(true);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error while opening camera: ", errorCode);
            }
        });


        while (!opModeIsActive()) {
            resultDetection = detector.getLocation();

            telemetry.addData("Status", "Pronto");
            telemetry.addData("Location", resultDetection);
            telemetry.update();
        }

        telemetry.addData("Status", "Pronto");
        telemetry.update();


        waitForStart();


        if (opModeIsActive()) {

            while (opModeIsActive()) {

                telemetry.addData("Posição do cubo: ", resultDetection);
                telemetry.update();
            }
        }
        webcam.stopStreaming();

    }


    private void InitMotors() {

        TracaoTE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TracaoTD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TracaoFE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TracaoFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TracaoTE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TracaoTD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TracaoFE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TracaoFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TracaoTE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TracaoTD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TracaoTE.setDirection(DcMotorSimple.Direction.FORWARD);
        TracaoFE.setDirection(DcMotorSimple.Direction.FORWARD);
        TracaoFD.setDirection(DcMotorSimple.Direction.REVERSE);
        TracaoTD.setDirection(DcMotorSimple.Direction.REVERSE);

        this.forca_motor = 0.4;
    }

    private void girar(int ang) {
        double angulo = Double.valueOf(ang);
        PID GIRAR_PID;

        angulo = -angulo;
        angulo = angulo % 360;

        if (180 < Math.abs(angulo)) {
            angulo = Math.abs(angulo % 180) * -1 * signum(angulo);
        }

        angulo_real = AnguloReal();
        this.GIRAR_giroAnterior = this.GIRAR_giroAnterior + angulo;
        angulo = angulo + angulo_real;

        GIRAR_PID = new PID(this.GIRAR_giroAnterior, 0.05, 2.0, 0.000001);

        while (opModeIsActive()) {
            angulo_real = AnguloReal();
            if (Math.abs(this.GIRAR_giroAnterior - angulo_real) <= 1 && 0.5 >= Math.abs(((Gyroscope) imu2).getAngularVelocity(AngleUnit.DEGREES).zRotationRate)) {
                break;
            }

            yawCorrecao = GIRAR_PID.calcularCorrecao(angulo_real);
            yawCorrecao = minOf(Math.abs(forca_motor * yawCorrecao), 0.7) * yawCorrecao;

            backFrontArcadeDrive(yawCorrecao, -yawCorrecao, 0);

            telemetry.addData("Ang_vel", ((Gyroscope) imu2).getAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Ang_Erro", angulo - angulo_real);
            telemetry.addData("Angulo", angulo_real);
            telemetry.update();

        }
        backFrontArcadeDrive(0.0, 0.0, 0.0);
        espere(500);
    }

    private double AnguloReal() {

        GYRO_angulo_atual = GYRO_anguloRealIMU();

        if (180 < Math.abs(GYRO_angulo_anterior - GYRO_angulo_atual)) {

            if (GYRO_angulo_anterior > GYRO_angulo_atual) {
                GYRO_revolu_C3_A7_C3_B5es = GYRO_revolu_C3_A7_C3_B5es + 1;

            } else {
                GYRO_revolu_C3_A7_C3_B5es = GYRO_revolu_C3_A7_C3_B5es - 1;

            }
        }

        GYRO_angulo_anterior = GYRO_angulo_atual;
        return GYRO_revolu_C3_A7_C3_B5es * 360 + GYRO_angulo_atual;
    }

    private double GYRO_anguloRealIMU() {
        return imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private void GYRO_inicializar() {
        BNO055IMU.Parameters GYRO_imu_parameters;

        GYRO_imu_parameters = new BNO055IMU.Parameters();
        GYRO_imu_parameters.mode = BNO055IMU.SensorMode.IMU;
        GYRO_imu_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        GYRO_imu_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        GYRO_imu_parameters.loggingEnabled = false;
        imu2.initialize(GYRO_imu_parameters);

        GYRO_angulo_atual = GYRO_anguloRealIMU();
        GYRO_angulo_anterior = GYRO_angulo_atual;
        GYRO_revolu_C3_A7_C3_B5es = 0;
        GIRAR_giroAnterior = 0;
    }

    private void backFrontArcadeDrive(double esq, double dir, double corr) {
        ((DcMotorEx) TracaoFD).setVelocity((dir - corr) * 2300);
        ((DcMotorEx) TracaoTD).setVelocity((dir - corr) * 2300);
        ((DcMotorEx) TracaoFE).setVelocity((esq + corr) * 2300);
        ((DcMotorEx) TracaoTE).setVelocity((esq + corr) * 2300);
    }

    private void espere(int duracao) {
        tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (opModeIsActive()) {
            if (duracao < tempo.milliseconds()) {
                break;
            }
        }
    }

    private double signum(double val) {
        if (val == 0) {
            return 1;
        }
        return val / Math.abs(val);
    }

    private double minOf(double val1, double val2) {
        return Math.min(val1, val2);
    }

    private void InitVuforia() {
        VuforiaCurrentGame vuforiaFreightFrenzy = new VuforiaCurrentGame();

        vuforiaFreightFrenzy.initialize(
                "",
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                "",
                false,
                true,
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0,
                0,
                0,
                AxesOrder.XZY,
                90,
                90,
                0,
                false);

        vuforiaFreightFrenzy.activate();

    }
}


/***

package org.firstinspires.ftc.teamcode;

import PID;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "Encoder_PID (Blocks to Java)")
public class Encoder_PID extends LinearOpMode {

  private DcMotor Braco;
  private DcMotor Carrossel;
  private Servo Garra;
  private VuforiaCurrentGame vuforiaFreightFrenzy;
  private DcMotor TracaoTD;
  private DcMotor TracaoTE;
  private BNO055IMU imu2;
  private DcMotor TracaoFE;
  private DcMotor TracaoFD;

  float GYRO_angulo_atual;
  ElapsedTime tempo;
  PID ANDAR_yaw_PID;
  List Positions;
  double forca_motor;
  float GYRO_angulo_anterior;
  int GYRO_revolu_C3_A7_C3_B5es;
  String detectionResult;
  String RobotPosition;
  double angulo_real;
  int GIRAR_giroAnterior;
  double yawCorrecao;

private void RedAlianceWallAut() {
    girar(-125);
    NivelEntrega();
    ((DcMotorEx) Braco).setVelocity(1000);
    girar(-100);
    sideways_arcade_drive(2400, true);
    Carrossel.setPower(0.25);
    espere(3000);
    Carrossel.setPower(0);
    sideways_arcade_drive(1100, false);
    Andar(-50);
}


    private void StartAutonomus() {
        Garra.setPosition(0.4);
        Braco.setTargetPosition(160);
        Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) Braco).setVelocity(1000);
        Andar(60);
        girar(80);
        RobotPosition = VerifVufResult();
        if (RobotPosition.equals("ND")) {
            girar(100);
            RobotPosition = VerifVufResult();
            if (RobotPosition.equals("ND")) {
                RobotPosition = "Red Alliance Wall";
            }
        }
        telemetry.addData("Posição", RobotPosition.toUpperCase());
        telemetry.update();
        vuforiaFreightFrenzy.deactivate();
        if (RobotPosition.equals("Blue Storage")) {
            BlueStorageAut();
        } else if (RobotPosition.equals("Red Storage")) {
            RedStorageAut();
        } else if (RobotPosition.equals("Blue Alliance Wall")) {
            BlueAlianceWallAut();
        } else {
            RedAlianceWallAut();
        }
        ((DcMotorEx) Braco).setVelocity(0);
        espere(1000);
    }


    private void InitVuforia() {
        vuforiaFreightFrenzy.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                false); // useCompetitionFieldTargetLocations
        vuforiaFreightFrenzy.activate();
        Positions = JavaUtil.createListWith("Blue Storage", "Blue Alliance Wall", "Red Storage", "Red Alliance Wall");
    }


    private void Andar(int distancia) {
        double forcaMotorInicial;
        double ANDAR_encPosition;
        PID ANDAR_DistPID;
        int ANDAR_realEncPosition;
        double ANDAR_distCorrecao;

        forcaMotorInicial = 0.25 * forca_motor;
        ANDAR_encPosition = (TracaoTD.getCurrentPosition() + TracaoTE.getCurrentPosition()) / 2 + distancia * 21.8;
        ANDAR_yaw_PID = PID.createPID(GYRO_anguloRealIMU(), 0.05, 5, 0.000001, 1);
        ANDAR_DistPID = PID.createPID(ANDAR_encPosition, -0.001, 50, 0, 1);
        while (opModeIsActive()) {
            angulo_real = AnguloReal();

            ANDAR_realEncPosition = (TracaoTD.getCurrentPosition() + TracaoTE.getCurrentPosition()) / 2;
            ANDAR_distCorrecao = PID.calcularCorrecao(ANDAR_DistPID, ANDAR_realEncPosition);
            yawCorrecao = PID.calcularCorrecao(ANDAR_yaw_PID, angulo_real);
            ANDAR_distCorrecao = minOf(Math.abs(ANDAR_distCorrecao), forca_motor + 0.2) * signum((int) ANDAR_distCorrecao);
            if (Math.abs(ANDAR_distCorrecao) <= 0.01 && Math.abs(yawCorrecao) <= 0.01) {
                break;
            }
            ANDAR_distCorrecao = ANDAR_distCorrecao * forcaMotorInicial;
            if (forcaMotorInicial < 1) {
                forcaMotorInicial = 0.05 * forcaMotorInicial + forcaMotorInicial;
                forcaMotorInicial = minOf(forcaMotorInicial, 1);
            }
            backFrontArcadeDrive((int) ANDAR_distCorrecao, (int) ANDAR_distCorrecao, (int) yawCorrecao);
            telemetry.addData("Ardar", " %" + Double.parseDouble(JavaUtil.formatNumber((ANDAR_realEncPosition * 100) / ANDAR_encPosition, 0)));
            telemetry.update();
        }
        backFrontArcadeDrive(0, 0, 0);
        espere(500);
    }


    private void girar(int angulo) {
        PID GIRAR_PID;

        angulo = -angulo;
        angulo = angulo % 360;
        if (180 < Math.abs(angulo)) {
            angulo = Math.abs(angulo % 180) * -1 * signum(angulo);
        }
        angulo_real = AnguloReal();
        GIRAR_giroAnterior = GIRAR_giroAnterior + angulo;
        angulo = angulo + angulo_real;
        GIRAR_PID = PID.createPID(GIRAR_giroAnterior, 0.05, 2, 0.000001, 1);
        while (opModeIsActive()) {
            angulo_real = AnguloReal();
            if (Math.abs(angulo - angulo_real) <= 0.8 && 0.5 >= Math.abs(((Gyroscope) imu2).getAngularVelocity(AngleUnit.DEGREES).zRotationRate)) {
                break;
            }
            yawCorrecao = PID.calcularCorrecao(GIRAR_PID, angulo_real);
            yawCorrecao = minOf(Math.abs(forca_motor * yawCorrecao), 0.7) * yawCorrecao;
            backFrontArcadeDrive((int) yawCorrecao, (int) -yawCorrecao, 0);
            telemetry.addData("Ang_vel", ((Gyroscope) imu2).getAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Ang_Erro", angulo - angulo_real);
            telemetry.addData("Angulo", angulo_real);
            telemetry.update();
        }
        backFrontArcadeDrive(0, 0, 0);
        espere(500);
    }
    private float AnguloReal() {
        GYRO_angulo_atual = GYRO_anguloRealIMU();
        if (180 < Math.abs(GYRO_angulo_anterior - GYRO_angulo_atual)) {
            if (GYRO_angulo_anterior > GYRO_angulo_atual) {
                GYRO_revolu_C3_A7_C3_B5es = GYRO_revolu_C3_A7_C3_B5es + 1;
            } else {
                GYRO_revolu_C3_A7_C3_B5es = GYRO_revolu_C3_A7_C3_B5es - 1;
            }
        }
        GYRO_angulo_anterior = GYRO_angulo_atual;
        return GYRO_revolu_C3_A7_C3_B5es * 360 + GYRO_angulo_atual;
    }


    private void InitMotors() {
        TracaoTE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TracaoTD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Braco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TracaoFE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TracaoFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TracaoTE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TracaoTD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Braco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TracaoFE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TracaoFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TracaoTE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TracaoTD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Braco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TracaoTE.setDirection(DcMotorSimple.Direction.FORWARD);
        TracaoFE.setDirection(DcMotorSimple.Direction.FORWARD);
        TracaoFD.setDirection(DcMotorSimple.Direction.REVERSE);
        TracaoTD.setDirection(DcMotorSimple.Direction.REVERSE);
        Braco.setDirection(DcMotorSimple.Direction.REVERSE);
        Carrossel.setDirection(DcMotorSimple.Direction.FORWARD);
        forca_motor = 0.4;
    }


    private int signum(int val) {
        if (val == 0) {
            return 1;
        }
        return val / Math.abs(val);
    }

     *
    private double minOf(double val1, double val2) {
        if (val1 < val2) {
            return val1;
        }
        return val2;
    }

    private void RedStorageAut() {
        girar(125);
        NivelEntrega();
        ((DcMotorEx) Braco).setVelocity(1000);
        girar(-35);
        Andar(-100);
    }


    private float GYRO_anguloRealIMU() {
        return imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    private void GYRO_inicializar() {
        BNO055IMU.Parameters GYRO_imu_parameters;

        // Não leia o gyro diretamente
        // pois a saída não é continua
        // Use a variável angulo_real
        GYRO_imu_parameters = new BNO055IMU.Parameters();
        GYRO_imu_parameters.mode = BNO055IMU.SensorMode.IMU;
        GYRO_imu_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        GYRO_imu_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        GYRO_imu_parameters.loggingEnabled = false;
        imu2.initialize(GYRO_imu_parameters);
        GYRO_angulo_atual = GYRO_anguloRealIMU();
        GYRO_angulo_anterior = GYRO_angulo_atual;
        GYRO_revolu_C3_A7_C3_B5es = 0;
        GIRAR_giroAnterior = 0;
    }


    private String VerifVufResult() {
        VuforiaBase.TrackingResults VERIVULRESULT_namePos;
        double i;

        VERIVULRESULT_namePos = "ND";
        for (i = 1; i <= 4; i++) {

            if (vuforiaFreightFrenzy.track(((String) JavaUtil.inListGet(Positions, JavaUtil.AtMode.FROM_START, (i - 1), false))).isVisible) {
                VERIVULRESULT_namePos = vuforiaFreightFrenzy.track(((String) JavaUtil.inListGet(Positions, JavaUtil.AtMode.FROM_START, (i - 1), false)));
                break;
            }
        }
        if (VERIVULRESULT_namePos.equals("ND")) {
            return "ND";
        }
        return VERIVULRESULT_namePos.name;
    }


    private void BlueAlianceWallAut() {
        girar(-135);
        NivelEntrega();
        ((DcMotorEx) Braco).setVelocity(0);
        girar(-80);
        sideways_arcade_drive(2400, true);
        Carrossel.setPower(-0.25);
        espere(3000);
        Carrossel.setPower(0);
        sideways_arcade_drive(1100, false);
        Andar(-50);
    }


    private void NivelEntrega() {
        if (detectionResult.equals("CENTER")) {
            Braco.setTargetPosition(300);
        } else if (detectionResult.equals("RIGHT")) {
            Braco.setTargetPosition(430);
        }
        Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) Braco).setVelocity(1000);
        Andar(37);
        Garra.setPosition(0.65);
        espere(1000);
        Andar(-37);
        Braco.setTargetPosition(160);
        Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void backFrontArcadeDrive(int esq, int dir, int corr) {
        ((DcMotorEx) TracaoFD).setVelocity((dir - corr) * 2300);
        ((DcMotorEx) TracaoTD).setVelocity((dir - corr) * 2300);
        ((DcMotorEx) TracaoFE).setVelocity((esq + corr) * 2300);
        ((DcMotorEx) TracaoTE).setVelocity((esq + corr) * 2300);
    }

    @Override
    public void runOpMode() {
        Braco = hardwareMap.get(DcMotor.class, "Braco");
        Carrossel = hardwareMap.get(DcMotor.class, "Carrossel");
        Garra = hardwareMap.get(Servo.class, "Garra");
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        TracaoTD = hardwareMap.get(DcMotor.class, "TracaoTD");
        TracaoTE = hardwareMap.get(DcMotor.class, "TracaoTE");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        TracaoFE = hardwareMap.get(DcMotor.class, "TracaoFE");
        TracaoFD = hardwareMap.get(DcMotor.class, "TracaoFD");

        telemetry.addData("Status", "Iniciando");
        telemetry.update();
        GYRO_inicializar();
        InitMotors();
        InitVuforia();
        RobotPosition = "ND";
        detectionResult = "RIGHT";
        telemetry.addData("Status", "Pronto");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                girar(90);
            }
        }

        vuforiaFreightFrenzy.close();
    }


    private void BlueStorageAut() {
        girar(-125);
        ((DcMotorEx) Braco).setVelocity(1000);
        girar(35);
        Andar(-100);
    }

    private void leftRigthArcadeDrive(int dirEsq, int esqDir, int corr) {
        ((DcMotorEx) TracaoFD).setVelocity((dirEsq + corr) * 2300);
        ((DcMotorEx) TracaoTE).setVelocity((dirEsq - corr) * 2300);
        ((DcMotorEx) TracaoFE).setVelocity((esqDir + corr) * 2300);
        ((DcMotorEx) TracaoTD).setVelocity((dirEsq - corr) * 2300);
    }

    private void sideways_arcade_drive(int duracao, boolean leftDirection) {
        tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ANDAR_yaw_PID = PID.createPID(GYRO_anguloRealIMU(), 0.05, 5, 0.000001, 1);
        while (opModeIsActive()) {
            angulo_real = AnguloReal();
            yawCorrecao = PID.calcularCorrecao(ANDAR_yaw_PID, angulo_real);
            if (duracao < tempo.milliseconds()) {
                break;
            }
            if (leftDirection) {
                leftRigthArcadeDrive((int) forca_motor, (int) -forca_motor, (int) yawCorrecao);
            } else {
                leftRigthArcadeDrive((int) -forca_motor, (int) forca_motor, (int) yawCorrecao);
            }
        }
        leftRigthArcadeDrive(0, 0, 0);
        espere(500);
    }


    private void espere(int duracao) {
        tempo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (opModeIsActive()) {
            if (duracao < tempo.milliseconds()) {
                break;
            }
        }
    }
}
*/