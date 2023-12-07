package frc.robot;

public final class Constants {
    // Motors
    public static final int lMotor1 = 1; // Left Back
    public static final int lMotor2 = 2; // Left Middle
    public static final int lMotor3 = 3; // Left Front

    public static final int rMotor1 = 4; // Right Back
    public static final int rMotor2 = 5; // Right Back
    public static final int rMotor3 = 6; // Right Front

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

    // Drivetrain PID Gains
    public static final double driveKP = 0.01; // Proportional Gain | PLACEHOLDER VALUE
    public static final double driveKI = 0.0;  // Integral Gain
    public static final double driveKD = 0.01; // Derivitave Gain | PLACEHOLDER VALUE
    // Drivetrain Feed Forward Gains
    public static final double driveKS = 0.01; // Static Gain | PLACEHOLDER VALUE
    public static final double driveKV = 0.01; // Velocity Gain | PLACEHOLDER VALUE
    public static final double driveKA = 0.01; // Acceleration Gain | PLACEHOLDER VALUE

    // Shooter

    // Intake
    public static final int intakeMotorPort = 7;
    public static final double intakeMotorVoltage = 1;

    // Indexer
    public static final int maxProximity = 0;
    public static final int conveyorPort = 0;

    // Flywheel
    public static final int FlywheelMotorPort = 8;
    // PID Gains
    public static final double shooterKP = 0.01;
    public static final double shooterKI = 0.0;
    public static final double shooterKD = 0.01;
    // Feed Forward gains
    public static final double shooterKS = 0.01;
    public static final double shooterKV = 0.01;
    public static final double shooterKA = 0.01;
}
