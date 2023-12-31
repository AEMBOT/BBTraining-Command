package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(Constants.intakeMotorPort, MotorType.kBrushless);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake current draw", motor.getOutputCurrent());
    }

    public void intakeOn() {
        motor.setVoltage(Constants.intakeMotorVoltage);
    }

    public void intakeOff() {
        motor.setVoltage(0);
    }

    public void intakeReverse() {
        motor.setVoltage(-Constants.intakeMotorVoltage);
    }

    public Command enableIntake() {
        return this.startEnd(this::intakeOn, this::intakeOff);
    }

    public Command reverseIntake() {
        return this.startEnd(this::intakeReverse, this::intakeOff);
    }
}
