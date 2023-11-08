package frc.robot;

public final class Constants {
    // Motors
    public static final int lMotor1 = 1; // Left Back | PLACEHOLDER VALUE
    public static final int lMotor2 = 2; // Left Middle | PLACEHOLDER VALUE
    public static final int lMotor3 = 3; // Left Front | PLACEHOLDER VALUE

    public static final int rMotor1 = 4; // Right Back | PLACEHOLDER VALUE
    public static final int rMotor2 = 5; // Right Back | PLACEHOLDER VALUE
    public static final int rMotor3 = 6; // Right Front | PLACEHOLDER VALUE

    public static final double trackWidth = 1; // Distance between the left and right wheels in meters | PLACEHOLDER VALUE

    // Encoders
    public static final int lEncoderA = 1; // Left Encoder port 1 | PLACEHOLDER VALUE
    public static final int lEncoderB = 2; // Left Encoder port 2 | PLACEHOLDER VALUE
    public static final boolean lEncoderReverse = true; // Left Encoder Reverse | PLACEHOLDER VALUE

    public static final int rEncoderA = 4; // Right Encoder port 1 | PLACEHOLDER VALUE
    public static final int rEncoderB = 5; // Right Encoder port 2 | PLACEHOLDER VALUE
    public static final boolean rEncoderReverse = true; // Right Encoder Reverse | PLACEHOLDER VALUE

    public static final double encoderDistancePerPulse = 1; // PLACEHOLDER VALUE

    // Control
    public static final int xBoxControllerPort = 0; // PLACEHOLDER VALUE
    public static final double turnSensitivity = 1; // The sensitivity of the turn control (right joystick x)

    // PID Gains
    public static final double kP = 0.01; // Proportional Gain | PLACEHOLDER VALUE
    public static final double kI = 0.0;  // Integral Gain
    public static final double kD = 0.01; // Derivitave Gain | PLACEHOLDER VALUE
    // Feed Forward Gains
    public static final double kS = 0.01; // Static Gain | PLACEHOLDER VALUE
    public static final double kV = 0.01; // Velocity Gain | PLACEHOLDER VALUE
    public static final double kA = 0.01; // Acceleration Gain | PLACEHOLDER VALUE
}
