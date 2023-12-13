package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class ShooterSubsystem extends SubsystemBase {
    ColorSensorV3 indexerSensor = new ColorSensorV3(I2C.Port.kMXP);

    CANSparkMax conveyorMotor = new CANSparkMax(Constants.conveyorPort, MotorType.kBrushless);
    CANSparkMax flywheelMotor = new CANSparkMax(Constants.FlywheelMotorPort, MotorType.kBrushless);

    PIDController flywheelPID = new PIDController(Constants.shooterKP, Constants.shooterKI, Constants.shooterKD);
    SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(Constants.shooterKS, Constants.shooterKV, Constants.shooterKA);

    public ShooterSubsystem() {
        flywheelMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("indexer proximity", indexerSensor.getProximity());
    }

    public Command defaultCommand() {
        return run(() -> {
            if (indexerSensor.getProximity() >= Constants.maxProximity) {
                conveyorMotor.setVoltage(0);
            } else {
                conveyorMotor.setVoltage(5);
            }
        });
    }


    private void indexerOn() {
        conveyorMotor.setVoltage(5);
    }

    private void indexerReverse() {
        conveyorMotor.setVoltage(-5);
    }

    public CommandBase runIndexerCommand() {
        return this.run(this::indexerOn);
    }

    public CommandBase reverseIndexerCommand() {
        return this.run(this::indexerReverse);
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
                this::flywheelOff
        );
    }

    public CommandBase shootCommand() {
        return flywheelOnCommand().andThen(waitSeconds(1).andThen(indexerOnCommand().andThen(waitSeconds(1).andThen(shooterOffCommand()))));
    }
}