package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * All tunable odometry constants live here so you're not hunting
 * through code to change a pod offset or flip a direction.
 */
public class PinpointConfig {

    // Physical offsets of the odometry pods from the robot's center of
    // rotation, in millimeters. Measure these with the robot's center
    // as your reference point (see the center-of-rotation measurement steps).
    public static double X_POD_OFFSET_MM = 0.0;
    public static double Y_POD_OFFSET_MM = 0.0;

    // Which goBILDA pod hardware you're using
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // Encoder count directions. Flip either of these during testing
    // if X or Y reads backwards when you push the robot by hand.
    public static GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Name given to the Pinpoint in your hardware configuration
    public static String HARDWARE_NAME = "pinpoint";
}

