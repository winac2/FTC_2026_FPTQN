package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.Queue;

public class GameFunctions {

    /* ================= HARDWARE ================= */
    private DcMotor classifyMotor, fireMotor, intakeMotor;

    private Servo fireServo;

    /* ================= QUEUE ================= */
    private final Queue<String> motifQueue = new ArrayDeque<>();
    private Telemetry Telemetry;

    /* ================= STATE ================= */
    public enum ClassifyState {
        IDLE,
        ROTATE_TO_COLOR,
        ROTATE,
        FIRE_PIXEL_OPEN,
        FIRE_PIXEL_CLOSE,
        OFF
    }

    public static ClassifyState state = ClassifyState.IDLE;
    private String currentColor = null;

    private final ElapsedTime timer = new ElapsedTime();

    public final PIDController pid = new PIDController(0.01, 0.0, 0.001);

    public ColorSensor CS = new ColorSensor();

    /* ================= INIT ================= */
    public void init(HardwareMap hwMap) {

        classifyMotor = hwMap.get(DcMotor.class, "classify");
        fireMotor = hwMap.get(DcMotor.class, "fire_motor");
        fireServo = hwMap.get(Servo.class, "fire_servo");
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");

        classifyMotor.setDirection(DcMotor.Direction.FORWARD);
        classifyMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        classifyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        classifyMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fireMotor.setDirection(DcMotor.Direction.FORWARD);
        fireMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fireServo.setPosition(0);
    }

    /*========================OUTTAKE=============================*/
    /* ================= LOAD MOTIF ================= */
    public void loadMotif(String c1, String c2, String c3) {
        motifQueue.clear();

        if (c1 != null) motifQueue.add(c1);
        if (c2 != null) motifQueue.add(c2);
        if (c3 != null) motifQueue.add(c3);
    }

    /* ================= UPDATE (CALL IN LOOP) ================= */
    public void update() {

        switch (state) {

            case IDLE:
                if (!motifQueue.isEmpty() && (!motifQueue.poll().equals("UNKNOWN"))) {
                    currentColor = motifQueue.poll();
                    state = ClassifyState.ROTATE_TO_COLOR;
                    timer.reset();
                }
                else if (!motifQueue.isEmpty() && (motifQueue.poll().equals("UNKNOWN"))) {
                    state = ClassifyState.ROTATE;
                    timer.reset();
                }
                else {
                    state = ClassifyState.OFF;
                }

            case ROTATE_TO_COLOR:
                String checking = rotateToColor();

                if (checking.equals(currentColor)) {
                    classifyMotor.setPower(0);
                    state = ClassifyState.FIRE_PIXEL_OPEN;
                    timer.reset();
                }
                break;

            case ROTATE:
                pid.rotateToAnglePID(
                        classifyMotor,
                        pid,
                        120,
                        0.5,
                        5
                );
                state = ClassifyState.FIRE_PIXEL_OPEN;
                break;

            case FIRE_PIXEL_OPEN:
                if (timer.milliseconds() > 300) {
                    fireServo.setPosition(0.8);
                    timer.reset();
                    state = ClassifyState.FIRE_PIXEL_CLOSE;
                }
                break;

            case FIRE_PIXEL_CLOSE:
                if (timer.milliseconds() > 200) {
                    fireServo.setPosition(0);
                    state = ClassifyState.IDLE;
                }
                break;

            case OFF:
                state = ClassifyState.IDLE;
        }
    }

    /* ================= ACTIONS ================= */
    private String rotateToColor() {

        pid.rotateToAnglePID(
                classifyMotor,
                pid,
                120,
                0.5,
                5
        );

        return CS.getDetectedColor();
    }

    public void fire_on(){
        fireMotor.setPower(1);
    }
    public void fire_off(){
        fireMotor.setPower(0);
    }

    /*===========================INTAKE=========================*/
    public void intake(double power){
        intakeMotor.setPower(power);
    }

    public void fill(){
        int dem = 0;
        while (!(CS.getDetectedColor().equals("UNKNOWN")) && (dem != 3)) {
            pid.rotateToAnglePID(
                    classifyMotor,
                    pid,
                    120,
                    0.5,
                    5
            );
            dem += 1;
        }
    }
}
