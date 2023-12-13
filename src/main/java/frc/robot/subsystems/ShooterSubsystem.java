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

    public void indexerOverrideOn() {
        indexerOverride = true;
    }

    public void indexerOverrideOff() {
        indexerOverride = false;
    }

    private void indexerOn() {
        indexerOverrideOn();

        conveyorMotor.setVoltage(5);
    }

    private void indexerReverse() {
        indexerOverrideOn();

        conveyorMotor.setVoltage(-5);
    }

    public boolean getIndexerOverride() {
        return indexerOverride;
    }

    public CommandBase runIndexerCommand() {
        return this.startEnd(this::indexerOn, this::indexerOverrideOff);
    }

    public CommandBase reverseIndexerCommand() {
        return this.startEnd(this::indexerReverse, this::indexerOverrideOff);
    }

    public void setFlywheelSpeed(Double speed) {
        flywheelMotor.setVoltage(flywheelPID.calculate(flywheelMotor.getEncoder().getVelocity(), speed)
                + flywheelFeedForward.calculate(speed));
    }

    public void tempSetFlywheelSpeed(Double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    public void flywheelOff() {
        flywheelMotor.setVoltage(0);
    }

    public CommandBase flywheelOnCommand() {
        return runOnce(
                () -> tempSetFlywheelSpeed(Constants.tempFlywheelSpeed)
        );
    }

    public CommandBase indexerOnCommand() {
        return runOnce(
                this::indexerOn
        );
    }

    public CommandBase shooterOffCommand() {
        return runOnce(
                () -> {
                    flywheelOff();
                    indexerOverrideOff();
                }
        );
    }

    public CommandBase shootCommand() {
        return flywheelOnCommand().andThen(waitSeconds(1).andThen(indexerOnCommand().andThen(waitSeconds(1).andThen(shooterOffCommand()))));
    }
}