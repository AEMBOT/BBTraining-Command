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
import frc.robot.constants;

public class DriveSubsystem extends SubsystemBase {
    private final MotorControllerGroup m_lMotors = new MotorControllerGroup(
        new VictorSP(constants.lbMotor),
        new VictorSP(constants.lfMotor));

    private final MotorControllerGroup m_rMotors = new MotorControllerGroup(
        new VictorSP(constants.rbMotor),
        new VictorSP(constants.rfMotor));

    private final DifferentialDrive m_dDrive = new DifferentialDrive(m_lMotors, m_rMotors);

    private final Encoder m_lEncoder = 
        new Encoder(
            constants.lEncoder1,
            constants.lEncoder2,
            constants.lEncoderReverse
        );

    private final Encoder m_rEncoder = 
        new Encoder(
            constants.rEncoder1,
            constants.rEncoder2,
            constants.rEncoderReverse
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

            m_lEncoder.setDistancePerPulse(constants.encoderDistancePerPulse);
            m_rEncoder.setDistancePerPulse(constants.encoderDistancePerPulse);

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
