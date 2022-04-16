package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Gyro {

    public final BNO055IMU imu;

    private double anguloAtual;
    private double anguloAnterior;
    private int    revoluecoes;


    public Gyro(HardwareMap hwM){

        BNO055IMU.Parameters GYRO_imu_parameters;

        imu = hwM.get(BNO055IMU.class, "imu2");

        GYRO_imu_parameters                = new BNO055IMU.Parameters();
        GYRO_imu_parameters.mode           = BNO055IMU.SensorMode.IMU;
        GYRO_imu_parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        GYRO_imu_parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        GYRO_imu_parameters.loggingEnabled = false;

        imu.initialize(GYRO_imu_parameters);


        anguloAnterior = anguloAtual;
        revoluecoes    = 0;

    }


    public double getAngle() {

        return -imu.getAngularOrientation(AxesReference
                                            .INTRINSIC, AxesOrder
                                            .ZYX, AngleUnit
                                            .DEGREES)
                                            .firstAngle;
    }


    public double getContinuosAngle() {

        anguloAtual = getAngle();


        if (180 < Math.abs(anguloAnterior - anguloAtual)) {

            if (anguloAnterior > anguloAtual) {

                revoluecoes = revoluecoes + 1;

            }
            else {

                revoluecoes = revoluecoes - 1;

            }
        }

        anguloAnterior = anguloAtual;


        return revoluecoes * 360 + anguloAtual;

    }

}
