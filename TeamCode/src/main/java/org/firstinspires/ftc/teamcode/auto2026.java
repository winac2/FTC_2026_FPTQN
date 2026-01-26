package org.firstinspires.ftc.teamcode;
//up vcs
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class auto2026 extends LinearOpMode {

    DcMotorEx motor;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
//    double Kf = 1;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double power = PIDControl(1000, motor.getVelocity());
            motor.setPower(power);
        }
    }


    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;


        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
//        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf); //Feedforward: predict target dieu chinh toc do nhanh hon
        return output;
    }

}


