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

@Autonomous(name="auto26_c2")
public class auto26_c2 extends LinearOpMode {
    DcMotor DTLeftMotor, DTRightMotor, Intake1, Intake2,Intake3, Intake4, Outtake1, Outtake2;
    public IMU imu;


    //==PID_config==
    double integralSum = 0;
    double Kp = 0.5;  //quay nhanh, nhung khong bi overshoot
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
        sleep(10);

//        double targetAngle = Math.toRadians(Math.toDegrees(getYaw()) + 90 );
//        double targetAngle = Math.toRadians(90);
        timer.reset();
        //auto case 1
        drive_encoder(300,0.5);
        sleep(100);
        rotate2(50);
        sleep(10);
        shoot(1);
        sleep(3000);
        take(0.8);
        sleep(3000);


//        rotate2(110);

        sleep(10);
        while (opModeIsActive()) {
            sleep(10);
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


    public void shoot(double power){
        Outtake1.setPower(power);
        Outtake2.setPower(power);
    }

    public void take(double power){
        Intake1.setPower(power);
        Intake2.setPower(power);
        Intake3.setPower(-power);
        Intake4.setPower(-power);

    }


    public void drive_encoder(int target_tick, double power){
        DTLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DTRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reset encoder
        DTRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DTLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DTLeftMotor.setTargetPosition(target_tick);
        DTRightMotor.setTargetPosition(target_tick);

        DTRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DTLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //RUN
        DTLeftMotor.setPower(power);
        DTRightMotor.setPower(power);

        while (opModeIsActive()
                && DTRightMotor.isBusy()
                && DTLeftMotor.isBusy()){
            telemetry.addData("Left", DTLeftMotor.getCurrentPosition());
            telemetry.addData("Right", DTRightMotor.getCurrentPosition());
            telemetry.update();
        }
        DTLeftMotor.setPower(0);
        DTRightMotor.setPower(0);

        DTLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }






    public void rotate2(double degree) {

        DTLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DTLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DTRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double target = angleWrap(getYaw() + Math.toRadians(degree));

        lastError = 0;
        integralSum = 0;

        ElapsedTime pidTimer = new ElapsedTime();     // cho dt
        ElapsedTime timeoutTimer = new ElapsedTime(); // cho timeout

        pidTimer.reset();
        timeoutTimer.reset();

        while (opModeIsActive()) {

            double current = getYaw();
            double error = angleWrap(target - current);

            double dt = Math.max(pidTimer.seconds(), 0.01);
            pidTimer.reset();

            double derivative = (error - lastError) / dt;
            lastError = error;

            double power = (Kp * error) + (Kd * derivative);
            power = clamp(power, -0.5, 0.5);

            DTLeftMotor.setPower(-power);
            DTRightMotor.setPower(power);

            telemetry.addData("Target (deg)", Math.toDegrees(target));
            telemetry.addData("Yaw (deg)", Math.toDegrees(current));
            telemetry.addData("Error (deg)", Math.toDegrees(error));
            telemetry.addData("Time (s)", timeoutTimer.seconds());
            telemetry.update();


            if (Math.abs(error) < Math.toRadians(1.0)) {
                DTLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                DTRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            }

            if (timeoutTimer.seconds() > 3.4) {
                telemetry.addLine("ROTATE TIMEOUT");
                telemetry.update();
                DTLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                DTRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            }

            sleep(10);
        }

        DTLeftMotor.setPower(0);
        DTRightMotor.setPower(0);

        sleep(50);
        imu.resetYaw();

        DTLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DTRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }





}


