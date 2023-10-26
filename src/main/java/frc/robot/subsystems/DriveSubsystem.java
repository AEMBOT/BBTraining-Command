package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final MotorControllerGroup m_lMotors = new MotorControllerGroup(
        new VictorSP(Constants.lbMotor),
        new VictorSP(Constants.lfMotor));

    private final MotorControllerGroup m_rMotors = new MotorControllerGroup(
        new VictorSP(Constants.rbMotor),
        new VictorSP(Constants.rfMotor));

    private final DifferentialDrive m_dDrive = new DifferentialDrive(m_lMotors, m_rMotors);

    private final Encoder m_lEncoder = 
        new Encoder(
            Constants.lEncoder1,
            Constants.lEncoder2,
            Constants.lEncoderReverse
        );

    private final Encoder m_rEncoder = 
        new Encoder(
            Constants.rEncoder1,
            Constants.rEncoder2,
            Constants.rEncoderReverse
        );

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
            m_rMotors.setInverted(true);

            m_lEncoder.setDistancePerPulse(Constants.encoderDistancePerPulse);
            m_rEncoder.setDistancePerPulse(Constants.encoderDistancePerPulse);

            resetEncoders();

            m_odometry =
                new DifferentialDriveOdometry(
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

        public double getLDist() {
            return m_lEncoder.getDistance();
        }

        public double getRDist() {
            return m_rEncoder.getDistance();
        }

        public Rotation2d getRot() {
            return m_gyro.getRotation2d();
        }
}