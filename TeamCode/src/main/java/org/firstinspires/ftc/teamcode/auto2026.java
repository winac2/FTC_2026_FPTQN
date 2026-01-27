package org.firstinspires.ftc.teamcode;
//up update
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;


public class auto2026 extends LinearOpMode {

//    DcMotorEx motor;
    DcMotorEx left;
    DcMotorEx right;

    private BNO055IMU imu;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
//    double Kf = 10;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
//        motor = hardwareMap.get(DcMotorEx.class, "motor");
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right = hardwareMap.get(DcMotorEx.class, "right");
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left = hardwareMap.get(DcMotorEx.class, "left");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        left.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        double referenceAngle = Math.toRadians(90);
        while (opModeIsActive()) {
            double power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
            left.setPower(power);
            right.setPower(-power);
        }
    }


    public double PIDControl(double reference, double state){
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;


        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
//        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf); //Feedforward: predict target dieu chinh toc do nhanh hon
        return output;
    }

    public double angleWrap(double radians){
        while (radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return  radians;
    }
    public void drive(double power_left, double power_right, long time){
        left.setPower(power_left);
        right.setPower(power_right);
        sleep(time);
        left.setPower(0);
        right.setPower(0);
    }

}


