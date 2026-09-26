    package org.firstinspires.ftc.teamcode.odometry;

    import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

    /**
     * Thin wrapper around the goBILDA Pinpoint driver. Handles setup once,
     * then gives the rest of the robot code a simple pose interface so
     * nothing else in your code needs to touch the driver directly.
     */
    public class PinpointLocalizer {

        private final GoBildaPinpointDriver pinpoint;

        public PinpointLocalizer(HardwareMap hardwareMap) {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PinpointConfig.HARDWARE_NAME);

            pinpoint.setOffsets(PinpointConfig.X_POD_OFFSET_MM, PinpointConfig.Y_POD_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(PinpointConfig.POD_TYPE);
            pinpoint.setEncoderDirections(PinpointConfig.X_ENCODER_DIRECTION, PinpointConfig.Y_ENCODER_DIRECTION);
            pinpoint.resetPosAndIMU();
        }

        /** Call once per loop, before reading pose or velocity. */
        public void update() {
            pinpoint.update();
        }

        /** Current field-frame pose: x, y, heading. */
        public Pose2D getPose() {
            return pinpoint.getPosition();
        }

        public double getX(DistanceUnit unit) {
            return getPose().getX(unit);
        }

        public double getY(DistanceUnit unit) {
            return getPose().getY(unit);
        }

        public double getHeading(AngleUnit unit) {
            return getPose().getHeading(unit);
        }

        public void updateTelemetry(Telemetry telemetry) {
            update();
            Pose2D pose = getPose();
            telemetry.addData("X", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", pose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        /** Re-zero the pose at wherever the robot physically is right now. */
        public void resetPose() {
            pinpoint.resetPosAndIMU();
        }

        /** Manually set the current pose, e.g. to a known starting position at the start of autonomous. */
        public void setPose(Pose2D pose) {
            pinpoint.setPosition(pose);
        }
    }
