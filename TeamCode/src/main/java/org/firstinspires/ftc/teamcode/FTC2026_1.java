package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FTC2026_1")
public class FTC2026_1 extends OpMode {
    public DcMotor DTLeftMotor, DTRightMotor, Intake1, Intake2, Intake3, Intake4, Outtake1, Outtake2;


    public Servo SvOuttake1, SvOuttake2;

    boolean state;

    // Drive function
    public void drive() {
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        //deadzone
        if (Math.abs(forward) < 0.05) {
            forward = 0;
        }

        if (Math.abs(turn) < 0.05) {
            turn = 0;
        }

        //curve (expo)
        forward = Math.signum(forward) * forward * forward;
        turn = Math.signum(turn) * turn * turn;

        // scale
        forward *= 1.0;
        turn *= 1.0;

        double leftMotor = forward + turn;
        double rightMotor = forward - turn;

        //normalize
        double max = Math.max(Math.abs(leftMotor), Math.abs(rightMotor));
        if (max > 1.0) {
            leftMotor /= max;
            rightMotor /= max;
        }

        DTLeftMotor.setPower(leftMotor);
        DTRightMotor.setPower(rightMotor);
    }

    // Intake function
    public void intake() {

        Intake1.setPower(-gamepad1.right_trigger);
        if (gamepad1.left_bumper) {
            Intake1.setPower(-1);
            Intake2.setPower(-1);
            Intake3.setPower(1);
            Intake4.setPower(1);
        }

        if (gamepad1.left_trigger != 0) {
            Intake1.setPower(gamepad1.left_trigger);
            Intake2.setPower(gamepad1.left_trigger);
            Intake3.setPower(-gamepad1.left_trigger);
            Intake4.setPower(-gamepad1.left_trigger);
        }

        if (gamepad1.x) {
            Intake1.setPower(0);
            Intake2.setPower(0);
        }

        if (gamepad2.dpad_up) {
            Intake1.setPower(1);
            Intake2.setPower(1);
        }
        else {
            Intake1.setPower(0);
            Intake2.setPower(0);
        }

        if (gamepad2.dpad_down) {
            Intake1.setPower(-1);
            Intake2.setPower(-1);
        }
        else {
            Intake1.setPower(0);
            Intake2.setPower(0);
        }

        if (gamepad2.right_bumper) {
            Intake3.setPower(1);
            Intake4.setPower(1);
        }
        else {
            Intake3.setPower(0);
            Intake4.setPower(0);
        }

        if (gamepad2.left_bumper) {
            Intake3.setPower(-1);
            Intake4.setPower(-1);
        }
        else {
            Intake3.setPower(0);
            Intake4.setPower(0);
        }

        Intake1.setPower(-gamepad2.left_stick_y);
        Intake2.setPower(-gamepad2.left_stick_y);
        Intake3.setPower(gamepad2.left_stick_y);
        Intake4.setPower(gamepad2.left_stick_y);
    }

    //Outtake function
    public void outtake() {
        Outtake1.setPower(Range.scale(gamepad2.right_trigger, 0, 1, 0.0, 0.85));
        Outtake2.setPower(Range.scale(gamepad2.right_trigger, 0, 1, 0.0, 0.85));

        if (gamepad2.a) {
            Outtake1.setPower(0.81);
            Outtake2.setPower(0.81);
        }
        if (gamepad2.y) {
            Outtake1.setPower(0.65);
            Outtake2.setPower(0.65);
        }
        if (gamepad2.x) {
            Outtake1.setPower(0.55);
            Outtake2.setPower(0.55);
        }
        if (gamepad2.b) {
            Outtake1.setPower(0.0);
            Outtake2.setPower(0.0);
        }
    }

    @Override
    public void init() {
        // Init drivetrain motor
        DTLeftMotor = hardwareMap.get(DcMotor.class, "dtleftmotor"); // port 2 - Control Hub
        DTRightMotor = hardwareMap.get(DcMotor.class, "dtrightmotor"); // port 2 - Expansion Hub
        DTLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DTRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DTLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        state = true;

        // Init outtake motor
        Outtake1 = hardwareMap.get(DcMotor.class, "outtake1"); // port 3 - Control Hub
        Outtake2 = hardwareMap.get(DcMotor.class, "outtake2"); // port 3 - Expansion Hub
        SvOuttake1 = hardwareMap.get(Servo.class, "svouttake1"); // port 1 - Control Hub
        SvOuttake2 = hardwareMap.get(Servo.class, "svouttake2"); // port 2 - Control Hub
        Outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        SvOuttake1.setDirection(Servo.Direction.FORWARD);
        SvOuttake2.setDirection(Servo.Direction.REVERSE);
        SvOuttake1.setPosition(0.0);
        SvOuttake2.setPosition(0.0);

        Intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        drive();
        intake();
        outtake();

        telemetry.addData("Right Y:", gamepad2.right_stick_y);
        telemetry.addData("Left Y:", gamepad2.left_stick_y);
    }
}
