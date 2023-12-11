package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class ShooterSubsystem extends SubsystemBase {
    ColorSensorV3 indexerSensor = new ColorSensorV3(I2C.Port.kMXP);

    CANSparkMax conveyorMotor = new CANSparkMax(Constants.conveyorPort, MotorType.kBrushless);
    CANSparkMax flywheelMotor = new CANSparkMax(Constants.FlywheelMotorPort, MotorType.kBrushless);

    Boolean indexerOverride = false;

    PIDController flywheelPID = new PIDController(Constants.shooterKP, Constants.shooterKI, Constants.shooterKD);
    SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(Constants.shooterKS, Constants.shooterKV, Constants.shooterKA);

    public ShooterSubsystem() {
        flywheelMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("indexer proximity", indexerSensor.getProximity());
        if (!indexerOverride) {
            if (indexerSensor.getProximity() >= Constants.maxProximity) {
                conveyorMotor.setVoltage(0);
            } else {
                conveyorMotor.setVoltage(5);
            }
        }

    }

    public void IndexerOverrideOn() {
        indexerOverride = true;
    }

    public void IndexerOverrideOff() {
        indexerOverride = false;
    }

    private void IndexerOn() {
        IndexerOverrideOn();

        conveyorMotor.setVoltage(5);
    }

    private void IndexerReverse() {
        IndexerOverrideOn();

        conveyorMotor.setVoltage(-5);
    }

    public boolean GetIndexerOverride() {
        return indexerOverride;
    }

    public CommandBase RunIndexerCommand() {
        return this.startEnd(() -> IndexerOn(), () -> IndexerOverrideOff());
    }

    public CommandBase ReverseIndexerCommand() {
        return this.startEnd(() -> IndexerReverse(), () -> IndexerOverrideOff());
    }

    public void SetFlywheelSpeed(Double speed) {
        flywheelMotor.setVoltage(flywheelPID.calculate(flywheelMotor.getEncoder().getVelocity(), speed)
                + flywheelFeedForward.calculate(speed));
    }

    public void tempSetFlywheelSpeed(Double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    public void FlywheelOff() {
        flywheelMotor.setVoltage(0);
    }

    public CommandBase FlywheelOnCommand() {
        return runOnce(
                () -> {
                    tempSetFlywheelSpeed(Constants.tempFlywheelSpeed);
                }
        );
    }

    public CommandBase IndexerOnCommand() {
        return runOnce(
                () -> {
                    IndexerOn();
                }
        );
    }

    public CommandBase ShooterOffCommand() {
        return runOnce(
                () -> {
                    FlywheelOff();
                    IndexerOverrideOff();
                }
        );
    }

    public CommandBase ShootCommand() {
        return FlywheelOnCommand().andThen(waitSeconds(1).andThen(IndexerOnCommand().andThen(waitSeconds(1).andThen(ShooterOffCommand()))));
    }
}