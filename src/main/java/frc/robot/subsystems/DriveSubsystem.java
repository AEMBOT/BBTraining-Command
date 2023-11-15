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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    MotorControllerGroup m_lMotors = new MotorControllerGroup(
        new CANSparkMax(Constants.lMotor1, MotorType.kBrushless),
        new CANSparkMax(Constants.lMotor2, MotorType.kBrushless),
        new CANSparkMax(Constants.lMotor3, MotorType.kBrushless)
    );

    MotorControllerGroup m_rMotors = new MotorControllerGroup(
        new CANSparkMax(Constants.rMotor1,MotorType.kBrushless),
        new CANSparkMax(Constants.rMotor2,MotorType.kBrushless),
        new CANSparkMax(Constants.rMotor3,MotorType.kBrushless)
    );

    DifferentialDrive m_dDrive = new DifferentialDrive(m_lMotors, m_rMotors);

    private final Encoder m_lEncoder = new Encoder(
            Constants.lEncoderA,
            Constants.lEncoderB,
            Constants.lEncoderReverse);

    private final Encoder m_rEncoder = new Encoder(
            Constants.rEncoderA,
            Constants.rEncoderB,
            Constants.rEncoderReverse);

    DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    
    private final AHRS m_gyro = new AHRS();

    private final DifferentialDriveOdometry m_odometry;

    private final PIDController m_lPid = new PIDController(Constants.driveKP, Constants.driveKI, Constants.driveKP);
    private final PIDController m_rPid = new PIDController(Constants.driveKP, Constants.driveKI, Constants.driveKD);
    
    private final SimpleMotorFeedforward m_lFeedForward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV,Constants.driveKA);
    private final SimpleMotorFeedforward m_rFeedForward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV,Constants.driveKA);

    public void resetEncoders() {
        m_lEncoder.reset();
        m_rEncoder.reset();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(getRot(), getRDist(), getLDist(), pose);
    }

    public void resetRot() {
        m_gyro.reset();
    }

    public DriveSubsystem() {
        m_rMotors.setInverted(true);

        m_lEncoder.setDistancePerPulse(Constants.encoderDistancePerPulse);
        m_rEncoder.setDistancePerPulse(Constants.encoderDistancePerPulse);

        resetEncoders();

        m_odometry = new DifferentialDriveOdometry(
                m_gyro.getRotation2d(),
                m_lEncoder.getDistance(),
                m_rEncoder.getDistance());
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), m_lEncoder.getDistance(), m_rEncoder.getDistance());
    }

    public void arcadeDrive(double speed, double rotSpeed) {
        m_dDrive.arcadeDrive(speed, rotSpeed);
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds driveWheelSpeeds = m_driveKinematics.toWheelSpeeds(speeds);
        m_lMotors.setVoltage(m_lPid.calculate(m_lEncoder.getRate(),driveWheelSpeeds.leftMetersPerSecond)
            +m_lFeedForward.calculate(driveWheelSpeeds.leftMetersPerSecond));
        m_rMotors.setVoltage(m_rPid.calculate(m_rEncoder.getRate(),driveWheelSpeeds.rightMetersPerSecond)
            +m_rFeedForward.calculate(driveWheelSpeeds.rightMetersPerSecond));
    }

    public double getLDist() {
        return m_lEncoder.getDistance();
    }

    public double getRDist() {
        return m_rEncoder.getDistance();
    }

    public Rotation2d getRot() {
        return m_gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return m_driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(m_lEncoder.getRate(),m_rEncoder.getRate()));
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
}