package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.BooleanSupplier;

/**
 * Reusable drive-to-point controller. Not an OpMode -- create one inside
 * whatever LinearOpMode you're running and call driveTo(...) from there.
 *
 * Example usage inside a LinearOpMode:
 *
 *   PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);
 *   DriveToPointController driveController =
 *           new DriveToPointController(hardwareMap, localizer, telemetry);
 *   waitForStart();
 *   driveController.driveTo(24, 0, 0, this::opModeIsActive);
 */
public class DriveToPointController {

    static final double KP_TRANSLATION = 0.03; // proportional gain for translation, e.g. 10 in error * 0.03 = 0.3 power
    static final double KP_HEADING = 0.02;      // proportional gain for rotation, e.g. 30 deg error * 0.02 = 0.6 power
    static final double POSITION_TOLERANCE_IN = 1.0;
    static final double HEADING_TOLERANCE_DEG = 2.0;

    private final PinpointLocalizer localizer;
    private final Telemetry telemetry;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    public DriveToPointController(HardwareMap hardwareMap, PinpointLocalizer localizer, Telemetry telemetry) {
        this.localizer = localizer;
        this.telemetry = telemetry;

        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
    }

    /**
     * Drives to the target field position and blocks until it arrives or
     * isActive returns false. Pass this::opModeIsActive from your LinearOpMode
     * as isActive so this loop respects stop-requested/timeouts correctly.
     */
    public void driveTo(double targetX, double targetY, double targetHeading, BooleanSupplier isActive) {
        while (isActive.getAsBoolean()) {
            localizer.update();
            Pose2D pose = localizer.getPose();

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - x;
            double errorY = targetY - y;
            double errorHeading = targetHeading - heading;

            // angles wrap around, so keep error in -180..180
            while (errorHeading > 180) errorHeading -= 360;
            while (errorHeading < -180) errorHeading += 360;

            if (Math.hypot(errorX, errorY) < POSITION_TOLERANCE_IN
                    && Math.abs(errorHeading) < HEADING_TOLERANCE_DEG) {
                stopDrive();
                return;
            }

            // rotate field-frame error into the robot's local frame
            double headingRad = Math.toRadians(heading);
            double forwardError = errorX * Math.cos(headingRad) + errorY * Math.sin(headingRad);
            double strafeError = -errorX * Math.sin(headingRad) + errorY * Math.cos(headingRad);

            double forwardPower = forwardError * KP_TRANSLATION;
            double strafePower = strafeError * KP_TRANSLATION;
            double turnPower = errorHeading * KP_HEADING;

            driveMecanum(forwardPower, strafePower, turnPower);

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", heading);
            telemetry.update();
        }
    }

    /**
     * Non-blocking version for iterative OpModes: call this once per loop()
     * iteration. Returns true once the robot has arrived (and stops it),
     * false if it's still driving and needs more loop() calls.
     */
    public boolean updateDriveTo(double targetX, double targetY, double targetHeading) {
        localizer.update();
        Pose2D pose = localizer.getPose();

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.DEGREES);

        double errorX = targetX - x;
        double errorY = targetY - y;
        double errorHeading = targetHeading - heading;

        while (errorHeading > 180) errorHeading -= 360;
        while (errorHeading < -180) errorHeading += 360;

        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", heading);
        telemetry.update();

        if (Math.hypot(errorX, errorY) < POSITION_TOLERANCE_IN
                && Math.abs(errorHeading) < HEADING_TOLERANCE_DEG) {
            stopDrive();
            return true;
        }

        double headingRad = Math.toRadians(heading);
        double forwardError = errorX * Math.cos(headingRad) + errorY * Math.sin(headingRad);
        double strafeError = -errorX * Math.sin(headingRad) + errorY * Math.cos(headingRad);

        driveMecanum(forwardError * KP_TRANSLATION, strafeError * KP_TRANSLATION, errorHeading * KP_HEADING);
        return false;
    }

    private void driveMecanum(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    private void stopDrive() {
        driveMecanum(0, 0, 0);
    }
}
