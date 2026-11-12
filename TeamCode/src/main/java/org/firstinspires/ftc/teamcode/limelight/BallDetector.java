
package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

public class BallDetector {

    // ===== PIPELINES =====

    private static final String CAMERA_NAME = "limelight";

    private static final int POLLEN_PIPELINE = 0;
    private static final int RED_PIPELINE = 1;
    private static final int BLUE_PIPELINE = 2;

    private static final double MAX_FRAME_AGE_MS = 150;

    // Accept zero-area results for initial testing.
    private static final double MIN_TARGET_AREA = 0.0;

    // ===== BALL DIMENSIONS =====

    private static final double POLLEN_RADIUS_IN = 1.4;
    private static final double NECTAR_RADIUS_IN = 1.8;

    // ===== CAMERA GEOMETRY =====

    private static final double CAMERA_TILT_DEG = 20.0;
    private static final double CAMERA_HEIGHT_IN = 13.2283465;
    private static final double CAMERA_FORWARD_IN = 5.0;
    private static final double CAMERA_LEFT_IN = 0.0;

    private static final double MIN_RANGE_IN = 4.0;
    private static final double MAX_RANGE_IN = 60.0;

    public enum BallType {
        POLLEN,
        RED_NECTAR,
        BLUE_NECTAR
    }

    // ===== RESULT FOR EACH POLLEN CANDIDATE =====

    public static class PollenDetection {

        public final Pose2D pose;
        public final double tx;
        public final double ty;
        public final double area;

        public PollenDetection(
                Pose2D pose,
                double tx,
                double ty,
                double area) {

            this.pose = pose;
            this.tx = tx;
            this.ty = ty;
            this.area = area;
        }
    }

    private final Limelight3A limelight;

    private double robotX;
    private double robotY;
    private double robotYaw;

    private int activePipeline = POLLEN_PIPELINE;

    private BallType lastDetectedType;
    private LLResultTypes.ColorResult lastDetectedTarget;

    private double detectionX;
    private double detectionY;
    private double detectionYaw;

    // ===== CONSTRUCTOR =====

    public BallDetector(
            HardwareMap hardwareMap,
            double x,
            double y,
            double yawRadians) {

        robotX = x;
        robotY = y;
        robotYaw = yawRadians;

        limelight = hardwareMap.get(
                Limelight3A.class, CAMERA_NAME);

        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(POLLEN_PIPELINE);

        activePipeline = POLLEN_PIPELINE;
        limelight.start();
    }

    // ===== UPDATE ROBOT POSE =====

    public void updateRobotPose(
            double x,
            double y,
            double yawRadians) {

        robotX = x;
        robotY = y;
        robotYaw = yawRadians;
    }

    // ===== READ ALL RESULTS FOR A PIPELINE =====

    private List<LLResultTypes.ColorResult> readTargets(
            int pipeline) {

        List<LLResultTypes.ColorResult> empty =
                new ArrayList<>();

        if (activePipeline != pipeline) {
            limelight.pipelineSwitch(pipeline);
            activePipeline = pipeline;
            return empty;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return empty;
        }

        if (result.getPipelineIndex() != pipeline) {
            return empty;
        }

        double age = result.getStaleness();

        if (!Double.isFinite(age)
                || age < 0
                || age > MAX_FRAME_AGE_MS) {
            return empty;
        }

        List<LLResultTypes.ColorResult> colors =
                result.getColorResults();

        if (colors == null) {
            return empty;
        }

        List<LLResultTypes.ColorResult> accepted =
                new ArrayList<>();

        for (LLResultTypes.ColorResult color : colors) {

            double area = color.getTargetArea();
            double tx =
                    color.getTargetXDegreesNoCrosshair();
            double ty =
                    color.getTargetYDegreesNoCrosshair();

            if (Double.isFinite(area)
                    && area >= MIN_TARGET_AREA
                    && Double.isFinite(tx)
                    && Double.isFinite(ty)) {

                accepted.add(color);
            }
        }

        return accepted;
    }

    // ===== ORIGINAL SINGLE-BALL METHODS =====

    public LLResultTypes.ColorResult detectPollen() {
        return detectLargest(
                POLLEN_PIPELINE, BallType.POLLEN);
    }

    public LLResultTypes.ColorResult detectRedNectar() {
        return detectLargest(
                RED_PIPELINE, BallType.RED_NECTAR);
    }

    public LLResultTypes.ColorResult detectBlueNectar() {
        return detectLargest(
                BLUE_PIPELINE, BallType.BLUE_NECTAR);
    }

    private LLResultTypes.ColorResult detectLargest(
            int pipeline, BallType type) {

        lastDetectedTarget = null;
        lastDetectedType = null;

        List<LLResultTypes.ColorResult> targets =
                readTargets(pipeline);

        LLResultTypes.ColorResult best = null;
        double largestArea = -1;

        for (LLResultTypes.ColorResult target : targets) {
            double area = target.getTargetArea();

            if (area > largestArea) {
                best = target;
                largestArea = area;
            }
        }

        if (best != null) {
            lastDetectedTarget = best;
            lastDetectedType = type;

            detectionX = robotX;
            detectionY = robotY;
            detectionYaw = robotYaw;
        }

        return best;
    }

    // ===== NEW: DETECT ALL POLLEN =====

    public List<PollenDetection> detectAllPollen() {

        List<PollenDetection> pollenList =
                new ArrayList<>();

        List<LLResultTypes.ColorResult> targets =
                readTargets(POLLEN_PIPELINE);

        // Snapshot robot location for this batch.
        double x = robotX;
        double y = robotY;
        double yaw = robotYaw;

        for (LLResultTypes.ColorResult target : targets) {

            Pose2D pose = calculatePose(
                    target,
                    POLLEN_RADIUS_IN,
                    x, y, yaw);

            pollenList.add(new PollenDetection(
                    pose,
                    target.getTargetXDegreesNoCrosshair(),
                    target.getTargetYDegreesNoCrosshair(),
                    target.getTargetArea()));
        }

        return pollenList;
    }

    // ===== EXISTING POSE METHOD =====

    public Pose2D calculateBallPose(
            LLResultTypes.ColorResult target) {

        if (target == null
                || target != lastDetectedTarget
                || lastDetectedType == null) {
            return null;
        }

        double radius =
                lastDetectedType == BallType.POLLEN
                        ? POLLEN_RADIUS_IN
                        : NECTAR_RADIUS_IN;

        return calculatePose(
                target,
                radius,
                detectionX,
                detectionY,
                detectionYaw);
    }

    // ===== SHARED GEOMETRY =====

    private Pose2D calculatePose(
            LLResultTypes.ColorResult target,
            double ballRadius,
            double x,
            double y,
            double yaw) {

        double tx = Math.toRadians(
                target.getTargetXDegreesNoCrosshair());

        double ty = Math.toRadians(
                target.getTargetYDegreesNoCrosshair());

        double pitch = Math.toRadians(
                CAMERA_TILT_DEG);

        double tanTx = Math.tan(tx);
        double tanTy = Math.tan(ty);

        // Ground-plane projection to ball-center height.
        // Assumes forward-facing camera without roll.

        double downward =
                Math.sin(pitch)
                        - Math.cos(pitch) * tanTy;

        double forwardRay =
                Math.cos(pitch)
                        + Math.sin(pitch) * tanTy;

        if (!Double.isFinite(downward)
                || !Double.isFinite(forwardRay)
                || downward <= 0.001
                || forwardRay <= 0) {
            return null;
        }

        double scale =
                (CAMERA_HEIGHT_IN - ballRadius)
                        / downward;

        double cameraForward =
                scale * forwardRay;

        double cameraLeft =
                -scale * tanTx;

        double forward =
                CAMERA_FORWARD_IN + cameraForward;

        double left =
                CAMERA_LEFT_IN + cameraLeft;

        double range = Math.hypot(forward, left);

        if (!Double.isFinite(range)
                || range < MIN_RANGE_IN
                || range > MAX_RANGE_IN) {
            return null;
        }

        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);

        double fieldX =
                x + forward * cos - left * sin;

        double fieldY =
                y + forward * sin + left * cos;

        double approachYaw = Math.atan2(
                fieldY - y,
                fieldX - x);

        if (!Double.isFinite(fieldX)
                || !Double.isFinite(fieldY)
                || !Double.isFinite(approachYaw)) {
            return null;
        }

        return new Pose2D(
                DistanceUnit.INCH,
                fieldX,
                fieldY,
                AngleUnit.RADIANS,
                approachYaw);
    }

    public BallType getLastDetectedType() {
        return lastDetectedType;
    }

    public void stop() {
        limelight.stop();
    }
}
