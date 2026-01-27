package org.firstinspires.ftc.teamcode;
//up update
//import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="auto2026")
public class auto2026 extends LinearOpMode {

//    DcMotorEx motor;
//    public DcMotor DTLeftMotor, DTRightMotor, Intake1, Intake2, Intake3, Intake4, Outtake1, Outtake2;

    DcMotor DTLeftMotor, DTRightMotor;
    public IMU imu;


    //==PID_config==
    double integralSum = 0;
    double Kp = 2.0;  //quay nhanh, nhung khong bi overshoot
    double Ki = 0.0;    //tranh tinh trang overshoot
    double Kd = 0.2;    //phanh muot hon
//    double Kf = 10;

    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
//        motor = hardwareMap.get(DcMotorEx.class, "motor");
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTRightMotor = hardwareMap.get(DcMotor.class, "DTRightMotor");
        DTRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTLeftMotor = hardwareMap.get(DcMotor.class, "DTLeftMotor");
        DTLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DTLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                );

        IMU.Parameters imuParameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParameters);
        imu.resetYaw();

        waitForStart();

        double targetAngle = Math.toRadians(90);
        timer.reset();

        while (opModeIsActive()) {

            //==execute== arms bu cac cho
            double currentAngle = getYaw();
            double power = PIDControl(targetAngle, currentAngle);

            DTLeftMotor.setPower(power);
            DTLeftMotor.setPower(-power);


            telemetry.addData("Target (deg)", 90);
            telemetry.addData("Current (deg)", Math.toDegrees(currentAngle));
            telemetry.addData("Error (deg)", Math.toDegrees(targetAngle - currentAngle));
            telemetry.update();

        }
    }

    //===every another stupid AF functions===


    //PID
    public double PIDControl(double reference, double state){
        double error = angleWrap(reference - state);

        double dt = timer.seconds();
        timer.reset();

        integralSum += error * dt;
        integralSum = clamp(integralSum, -0.4, 0.4);

        double derivative = (error - lastError) / dt;
        lastError = error;


        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
//        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf); //Feedforward: predict target dieu chinh toc do nhanh hon
        return clamp(output, -1, 1);
    }


    // ===== IMU YAW =====
    public double getYaw() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    // ===== UTILS =====
    public double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }


    public void drive(double power_left, double power_right, long time){
        DTLeftMotor.setPower(power_left);
        DTRightMotor.setPower(power_right);
        sleep(time);
        DTLeftMotor.setPower(0);
        DTRightMotor.setPower(0);
    }

}


