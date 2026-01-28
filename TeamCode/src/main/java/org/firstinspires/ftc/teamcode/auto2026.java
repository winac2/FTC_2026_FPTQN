package org.firstinspires.ftc.teamcode;
//up update
//import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="auto2026")
public class auto2026 extends LinearOpMode {

//    DcMotorEx motor;
//    public DcMotor DTLeftMotor, DTRightMotor, Intake1, Intake2, Intake3, Intake4, Outtake1, Outtake2;

    DcMotor DTLeftMotor, DTRightMotor, Intake1, Intake2,Intake3, Intake4, Outtake1, Outtake2;
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
        DTRightMotor = hardwareMap.get(DcMotor.class, "dtrightmotor");
        DTRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTLeftMotor = hardwareMap.get(DcMotor.class, "dtleftmotor");
        DTLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DTRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        Intake1 = hardwareMap.get(DcMotor.class, "intake1"); // port 0 - Control Hub
        Intake2 = hardwareMap.get(DcMotor.class, "intake2"); // port 0 - Expansion Hub
        Intake3 = hardwareMap.get(DcMotor.class, "intake3"); // port 1 - Control Hub
        Intake4 = hardwareMap.get(DcMotor.class, "intake4"); // port 1 - Expansion Hub
        Intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake4.setDirection(DcMotorSimple.Direction.REVERSE);

//        state = true;

        // Init outtake motor
        Outtake1 = hardwareMap.get(DcMotor.class, "outtake1"); // port 3 - Control Hub
        Outtake2 = hardwareMap.get(DcMotor.class, "outtake2"); // port 3 - Expansion Hub
//        SvOuttake1 = hardwareMap.get(Servo.class, "svouttake1"); // port 1 - Control Hub
//        SvOuttake2 = hardwareMap.get(Servo.class, "svouttake2"); // port 2 - Control Hub
        Outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);


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

        double targetAngle = Math.toRadians(80);
        timer.reset();

        while (opModeIsActive()) {

            //==execute==
            double currentAngle = getYaw();
            double power = PIDControl(targetAngle, currentAngle);
            double error = angleWrap(targetAngle - currentAngle);

            DTRightMotor.setPower(power);
            DTLeftMotor.setPower(-power);


            telemetry.addData("Target (deg)", 90);
            telemetry.addData("Current (deg)", Math.toDegrees(currentAngle));
            telemetry.addData("Error (deg)", Math.toDegrees(targetAngle - currentAngle));
            telemetry.update();

            if (Math.abs(error) < Math.toRadians(0.1)) break;

            sleep(10);
//            rotate(90);

        }
    }



    //===another stupid AF functions===


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
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
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


    public void rotate(double degree) {

        // Quay = KHÔNG dùng encoder
        DTLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetRad = Math.toRadians(degree) + getYaw();

        integralSum = 0;
        lastError = 0;
        timer.reset();

        while (opModeIsActive()) {

            double current = getYaw();
            double error = angleWrap(targetRad - current);

            double dt = timer.seconds();
            timer.reset();

            integralSum += error * dt;
            integralSum = clamp(integralSum, -0.4, 0.4);

            double derivative = (error - lastError) / dt;
            lastError = error;

            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            power = clamp(power, -1, 1);

            DTLeftMotor.setPower(power);
            DTRightMotor.setPower(-power);

            // ===== STOP CONDITION =====
            if (Math.abs(error) < Math.toRadians(1)) break;

            sleep(10); // 100Hz
        }

        DTLeftMotor.setPower(0);
        DTRightMotor.setPower(0);
    }

}


