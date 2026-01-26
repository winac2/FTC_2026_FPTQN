package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDController {

    /* ================= PID CORE ================= */

    public double kP, kI, kD;
    private double integral = 0;
    private double lastError = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double error) {
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }

    /* ================= CONSTANTS ================= */

    // Core Hex
    public static final double TICKS_PER_REV_CORE = 288.0;
    public static final double TICKS_PER_DEGREE_CORE =
            TICKS_PER_REV_CORE / 360.0;

    // HD Hex 40:1
    public static final double TICKS_PER_REV_HD_40 = 1120.0;

    // Mecanum wheel
    public static final double WHEEL_DIAMETER_CM = 9.6; //sua lai sau
    public static final double WHEEL_CIRCUMFERENCE_CM =
            Math.PI * WHEEL_DIAMETER_CM;

    public static final double TICKS_PER_CM =
            TICKS_PER_REV_HD_40 / WHEEL_CIRCUMFERENCE_CM;

    /* ================= ROTATE CORE HEX ================= */

    /**
     * Quay motor theo góc TƯƠNG ĐỐI bằng PID (non-blocking)
     */
    public static void rotateToAnglePID(
            DcMotor motor,
            PIDController pid,
            double deltaAngleDeg,
            double maxPower,
            int toleranceTicks
    ) {
        int targetTicks = motor.getCurrentPosition()
                + (int)(deltaAngleDeg * TICKS_PER_DEGREE_CORE);

        int currentTicks = motor.getCurrentPosition();
        double error = targetTicks - currentTicks;

        if (Math.abs(error) <= toleranceTicks) {
            motor.setPower(0);
            pid.reset();
        }

        double power = pid.update(error);
        power = clamp(power, maxPower, 0.07);

        motor.setPower(power);
    }

    /* ================= MECANUM DRIVE ================= */

    /**
     * Di chuyển mecanum theo khoảng cách (encoder trung bình)
     * Dùng được forward / strafe / chéo
     */
    public static boolean driveDistancePID(
            DcMotor fl, DcMotor fr,
            DcMotor bl, DcMotor br,
            PIDController pid,
            double distanceCm,
            double x, double y,
            double maxPower,
            int toleranceTicks
    ) {
        // Normalize direction vector
        double mag = Math.hypot(x, y);
        if (mag > 1.0) {
            x /= mag;
            y /= mag;
        }

        int targetTicks = (int)(distanceCm * TICKS_PER_CM);

        int avgTicks =
                (Math.abs(fl.getCurrentPosition()) +
                        Math.abs(fr.getCurrentPosition()) +
                        Math.abs(bl.getCurrentPosition()) +
                        Math.abs(br.getCurrentPosition())) / 4;

        double error = targetTicks - avgTicks;

        if (Math.abs(error) <= toleranceTicks) {
            stopMotors(fl, fr, bl, br);
            pid.reset();
            return true;
        }

        double basePower = pid.update(error);
        basePower = clamp(basePower, maxPower, 0.08);

        double flP = basePower * (y + x);
        double frP = basePower * (y - x);
        double blP = basePower * (y - x);
        double brP = basePower * (y + x);

        normalizeAndSet(fl, fr, bl, br, flP, frP, blP, brP);
        return false;
    }

    /* ================= HELPERS ================= */

    private static double clamp(double power, double max, double min) {
        power = Math.max(-max, Math.min(max, power));
        if (Math.abs(power) < min)
            power = Math.signum(power) * min;
        return power;
    }

    private static void stopMotors(
            DcMotor fl, DcMotor fr,
            DcMotor bl, DcMotor br
    ) {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private static void normalizeAndSet(
            DcMotor fl, DcMotor fr,
            DcMotor bl, DcMotor br,
            double flP, double frP,
            double blP, double brP
    ) {
        double max = Math.max(
                Math.max(Math.abs(flP), Math.abs(frP)),
                Math.max(Math.abs(blP), Math.abs(brP))
        );

        if (max > 1.0) {
            flP /= max;
            frP /= max;
            blP /= max;
            brP /= max;
        }

        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);
    }
}
