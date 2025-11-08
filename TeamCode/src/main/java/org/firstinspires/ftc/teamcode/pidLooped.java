package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="BadWolf", group="Linear OpMode")
public class pidLooped extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, intakeMotor;
    private Servo clawServo, hoodServo;

    // shooter control state
    private boolean shooterOn = true;
    private static final double MAX_RPM = 200;
    private static final double TICKS_PER_REV = 537.6;
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTime = 0L;

    private double targetRPM = 6000.0;
    private double kP = 0.0003;
    private double emaAlpha = 0.15;

    private double rpmScale = 1.0;
    private boolean yPressedLast = false;

    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;

    private boolean atTargetLast = false;
    private boolean rumbling = false;
    private long rumbleEndTimeMs = 0L;
    private static final double TARGET_TOLERANCE_RPM = 5.0;
    private static final long RUMBLE_DURATION_MS = 1000L;

    // hood/claw timing
    private double hoodPosition = 0.12;
    private long lastHoodAdjustMs = 0L;
    private static final long HOOD_ADJUST_DEBOUNCE_MS = 120L;

    // claw action phases
    private int clawActionPhase = 0;
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 500L;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initial servo positions
        clawServo.setPosition(0.63);
        hoodServo.setPosition(hoodPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lastShooterPosition = shooter.getCurrentPosition();
        lastShooterTime = System.currentTimeMillis();
        targetRPM = MAX_RPM;
        shooterOn = true;

        waitForStart();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // DRIVE
            double axial   = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // DPAD shooter toggles
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) shooterOn = !shooterOn;
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) targetRPM = Math.max(0.0, targetRPM - 10.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) targetRPM = Math.min(MAX_RPM, targetRPM + 10.0);
            dpadRightLast = dpadRightNow;

            // INTAKE
            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                intakeMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            // RPM measurement
            int currentPosition = shooter.getCurrentPosition();
            int deltaTicks = currentPosition - lastShooterPosition;
            long deltaTimeMs = nowMs - lastShooterTime;
            if (deltaTimeMs <= 0) deltaTimeMs = 1;
            double ticksPerSec = (deltaTicks * 1000.0) / deltaTimeMs;
            double measuredRPMRaw = (ticksPerSec / TICKS_PER_REV) * 60.0;
            double measuredRPMScaled = measuredRPMRaw * rpmScale;

            currentRPM = (1.0 - emaAlpha) * currentRPM + emaAlpha * measuredRPMScaled;
            if (currentRPM < 0.0) currentRPM = 0.0;
            lastShooterPosition = currentPosition;
            lastShooterTime = nowMs;

            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast) {
                double safeMeasured = Math.abs(measuredRPMRaw);
                if (safeMeasured >= 1.0) {
                    double candidateScale = targetRPM / measuredRPMRaw;
                    rpmScale = Math.max(0.2, Math.min(3.0, candidateScale));
                }
            }
            yPressedLast = yNow;

            double ff = targetRPM / Math.max(1.0, MAX_RPM);
            double error = targetRPM - currentRPM;
            double pTerm = kP * error;
            double shooterPower = ff + pTerm;
            shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));
            shooter.setPower(shooterOn ? shooterPower : 0.0);

            boolean atTargetNow = Math.abs(targetRPM - currentRPM) <= TARGET_TOLERANCE_RPM;
            if (atTargetNow && !atTargetLast) {
                rumbling = true;
                rumbleEndTimeMs = nowMs + RUMBLE_DURATION_MS;
                try { gamepad1.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
            }
            atTargetLast = atTargetNow;
            if (rumbling && nowMs > rumbleEndTimeMs) rumbling = false;

            // HOOD adjustments
            if (gamepad1.a && nowMs - lastHoodAdjustMs > HOOD_ADJUST_DEBOUNCE_MS) {
                lastHoodAdjustMs = nowMs;
                hoodPosition += 0.05;
                if (hoodPosition > 0.45) hoodPosition = 0.45;
                hoodServo.setPosition(hoodPosition);
            }
            else if (gamepad1.b && nowMs - lastHoodAdjustMs > HOOD_ADJUST_DEBOUNCE_MS) {
                lastHoodAdjustMs = nowMs;
                hoodPosition -= 0.05;
                if (hoodPosition < 0.15) hoodPosition = 0.15;
                hoodServo.setPosition(hoodPosition);
            }
            // CLAW timed cycle
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawActionStartMs = System.currentTimeMillis();
                clawActionPhase = 1;
                // Close immediately
                clawServo.setPosition(0.3);
            }
            xPressedLast = xNow;

            if (clawActionPhase == 1 && System.currentTimeMillis() >= (clawActionStartMs + CLAW_CLOSE_MS)) {
                // After delay, reopen
                clawServo.setPosition(0.63);
                clawActionPhase = 2;
            }
            if (clawActionPhase == 2 && !xNow) {
                // Reset to idle after cycle completes and button released
                clawActionPhase = 0;
            }

            // TELEMETRY
            telemetry.addData("RPM", "%.1f", currentRPM);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Hood Position", "%.2f", hoodPosition);
            telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
            telemetry.update();
        }
    }
}