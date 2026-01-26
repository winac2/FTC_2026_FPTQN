package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FTC2026_1")
public class FTC2026_1 extends OpMode {
    public DcMotor DTLeftMotor, DTRightMotor, Intake1, Intake2, Intake3, Intake4, Outtake1, Outtake2;

    // Drive function
    public void drive() {
        DTLeftMotor.setPower(-gamepad1.right_stick_y);
        DTRightMotor.setPower(-gamepad1.right_stick_y);
        DTLeftMotor.setPower(-gamepad1.left_stick_x);
        DTRightMotor.setPower(gamepad1.left_stick_x);
    }

    // Intake function
    public void intake() {

        if (gamepad2.right_bumper) {
            Intake1.setPower(1.0);
            Intake2.setPower(1.0);
            Intake3.setPower(1.0);
            Intake4.setPower(1.0);
        }
        else {
            Intake1.setPower(0);
            Intake2.setPower(0);
            Intake3.setPower(0);
            Intake4.setPower(0);
        }

        if (gamepad2.left_bumper) {
            Intake1.setPower(-1.0);
            Intake2.setPower(-1.0);
            Intake3.setPower(-1.0);
            Intake4.setPower(-1.0);
        }
        else {
            Intake1.setPower(0);
            Intake2.setPower(0);
            Intake3.setPower(0);
            Intake4.setPower(0);
        }
    }

    //Outtake function
    public void outtake() {
        /*if (gamepad2.a) {
            Outtake1.setPower(1.0);
            Outtake2.setPower(1.0);
        }
        else {
            Outtake1.setPower(0.0);
            Outtake2.setPower(0.0);
        }*/

        Outtake1.setPower(gamepad2.right_stick_y);
        Outtake2.setPower(gamepad2.right_stick_y);
    }

    @Override
    public void init() {
        // Init drivetrain motor
        DTLeftMotor = hardwareMap.get(DcMotor.class, "dtleftmotor"); // port 2 - Control Hub
        DTRightMotor = hardwareMap.get(DcMotor.class, "dtrightmotor"); // port 2 - Expansion Hub
        DTLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DTRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DTRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init intake motor
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

        // Init outtake motor
        Outtake1 = hardwareMap.get(DcMotor.class, "outtake1"); // port 3 - Control Hub
        Outtake2 = hardwareMap.get(DcMotor.class, "outtake2"); // port 3 - Expansion Hub
        Outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drive();
        intake();
        outtake();
        telemetry.addData("Right Y:", gamepad2.right_stick_y);
    }
}
