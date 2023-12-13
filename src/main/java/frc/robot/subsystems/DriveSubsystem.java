package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.Timer.delay;
import static java.lang.Thread.sleep;

public class DriveSubsystem extends SubsystemBase {
    CANSparkMax motor1 = new CANSparkMax(Constants.lMotor1,MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(Constants.lMotor2,MotorType.kBrushless);
    CANSparkMax motor3 = new CANSparkMax(Constants.lMotor3,MotorType.kBrushless);

    MotorControllerGroup lMotors = new MotorControllerGroup(
        motor1,
        motor2,
        motor3
    );

    CANSparkMax motor4 = new CANSparkMax(Constants.rMotor1,MotorType.kBrushless);
    CANSparkMax motor5 = new CANSparkMax(Constants.rMotor2,MotorType.kBrushless);
    CANSparkMax motor6 = new CANSparkMax(Constants.rMotor3,MotorType.kBrushless);
    MotorControllerGroup rMotors = new MotorControllerGroup(
        motor4,
        motor5,
        motor6
    );


    DifferentialDrive dDrive = new DifferentialDrive(lMotors, rMotors);

    private final Encoder lEncoder = new Encoder(
            Constants.lEncoderA,
            Constants.lEncoderB,
            Constants.lEncoderReverse);

    private final Encoder rEncoder = new Encoder(
            Constants.rEncoderA,
            Constants.rEncoderB,
            Constants.rEncoderReverse);

    DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Constants.trackWidth);

    private final AHRS gyro = new AHRS();

    private final DifferentialDriveOdometry odometry;

    private final PIDController lPid = new PIDController(Constants.driveKP, Constants.driveKI, Constants.driveKP);
    private final PIDController rPid = new PIDController(Constants.driveKP, Constants.driveKI, Constants.driveKD);

    private final SimpleMotorFeedforward lFeedForward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);
    private final SimpleMotorFeedforward rFeedForward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);

    public void resetEncoders() {
        lEncoder.reset();
        rEncoder.reset();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(getRot(), getRDist(), getLDist(), pose);
    }

    public void resetRot() {
        gyro.reset();
    }

    public DriveSubsystem() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor3.restoreFactoryDefaults();
        motor4.restoreFactoryDefaults();
        motor5.restoreFactoryDefaults();
        motor6.restoreFactoryDefaults();

        motor1.setInverted(false);
        motor2.setInverted(false);
        motor3.setInverted(false);
        motor4.setInverted(false);
        motor5.setInverted(false);
        motor6.setInverted(false);

        delay(1);

        motor1.burnFlash();
        motor2.burnFlash();
        motor3.burnFlash();
        motor4.burnFlash();
        motor5.burnFlash();
        motor6.burnFlash();

        //lMotors.setInverted(true);

        lEncoder.setDistancePerPulse(Constants.encoderDistancePerPulse);
        rEncoder.setDistancePerPulse(Constants.encoderDistancePerPulse);

        resetEncoders();

        odometry = new DifferentialDriveOdometry(
                gyro.getRotation2d(),
                lEncoder.getDistance(),
                rEncoder.getDistance());
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), lEncoder.getDistance(), rEncoder.getDistance());
        SmartDashboard.putNumber("left motors drive current",
                motor1.getOutputCurrent() + motor2.getOutputCurrent() + motor3.getOutputCurrent()
        );
        SmartDashboard.putNumber("right motors drive current",
                motor4.getOutputCurrent() + motor5.getOutputCurrent() + motor6.getOutputCurrent()
        );
    }

    public void arcadeDrive(double speed, double rotSpeed) {
        dDrive.arcadeDrive(speed, rotSpeed);
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds driveWheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
        lMotors.setVoltage(lPid.calculate(lEncoder.getRate(), driveWheelSpeeds.leftMetersPerSecond)
                + lFeedForward.calculate(driveWheelSpeeds.leftMetersPerSecond));
        rMotors.setVoltage(rPid.calculate(rEncoder.getRate(), driveWheelSpeeds.rightMetersPerSecond)
                + rFeedForward.calculate(driveWheelSpeeds.rightMetersPerSecond));
    }

    public double getLDist() {
        return lEncoder.getDistance();
    }

    public double getRDist() {
        return rEncoder.getDistance();
    }

    public Rotation2d getRot() {
        return gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(lEncoder.getRate(), rEncoder.getRate()));
    }

    // Assuming this is a method in your drive subsystem
    public Command followPathCommand(PathPlannerPath path) {
        // You must wrap the path following command in a FollowPathWithEvents command in
        // order for event markers to work
        return new FollowPathWithEvents(
                new FollowPathRamsete(
                        path,
                        this::getPose, // Robot pose supplier
                        this::getSpeeds, // Current ChassisSpeeds supplier
                        this::drive, // Method that will drive the robot given ChassisSpeeds
                        new ReplanningConfig(), // Default path replanning config. See the API for the options here
                        this // Reference to this subsystem to set requirements
                ),
                path, // FollowPathWithEvents also requires the path
                this::getPose // FollowPathWithEvents also requires the robot pose supplier
        );
    }

    public Command driveForward() {
        return run(
                () -> arcadeDrive(1, 0)
        );
    }

    public Command stopDriving() {
        return runOnce(
                () -> arcadeDrive(0, 0)
        );
    }

    public Command randomAuto() {
        return driveForward().withTimeout(0.5).andThen(stopDriving());
    }
}