package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax m_lbMotor = new CANSparkMax(Constants.lbMotor, MotorType.kBrushless);
    private final CANSparkMax m_lmMotor = new CANSparkMax(Constants.lmMotor, MotorType.kBrushless);
    private final CANSparkMax m_lfMotor = new CANSparkMax(Constants.lfMotor, MotorType.kBrushless);

    private final CANSparkMax m_rbMotor = new CANSparkMax(Constants.rbMotor,MotorType.kBrushless);
    private final CANSparkMax m_rmMotor = new CANSparkMax(Constants.rmMotor,MotorType.kBrushless);
    private final CANSparkMax m_rfMotor = new CANSparkMax(Constants.rfMotor,MotorType.kBrushless);



    private final Encoder m_lEncoder = new Encoder(
            Constants.lEncoder1,
            Constants.lEncoder2,
            Constants.lEncoderReverse);

    private final Encoder m_rEncoder = new Encoder(
            Constants.rEncoder1,
            Constants.rEncoder2,
            Constants.rEncoderReverse);

    DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    
    private final AHRS m_gyro = new AHRS();

    private final DifferentialDriveOdometry m_odometry;

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
        m_rbMotor.setInverted(true);
        m_rmMotor.setInverted(true);
        m_rfMotor.setInverted(true);

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
        // m_dDrive.arcadeDrive(speed, rotSpeed);
        
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds driveWheelSpeeds = m_driveKinematics.toWheelSpeeds(speeds);
        // m_dDrive.tankDrive(driveWheelSpeeds.leftMetersPerSecond, driveWheelSpeeds.rightMetersPerSecond);
        m_lbMotor.set(driveWheelSpeeds.leftMetersPerSecond);
        m_lmMotor.set(driveWheelSpeeds.leftMetersPerSecond);
        m_lfMotor.set(driveWheelSpeeds.leftMetersPerSecond);

        m_rbMotor.set(driveWheelSpeeds.rightMetersPerSecond);
        m_rmMotor.set(driveWheelSpeeds.rightMetersPerSecond);
        m_rfMotor.set(driveWheelSpeeds.rightMetersPerSecond);
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