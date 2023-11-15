package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax m_motor = new CANSparkMax(Constants.intakeMotorPort, MotorType.kBrushless);

    public void intakeOn() {
        m_motor.setVoltage(Constants.intakeMotorVoltage);
    }
    
    public void intakeOff() {
        m_motor.setVoltage(0);
    }

    public Command enableIntake() {
        return this.startEnd(()->{intakeOn();}, ()->{intakeOff();});
    }
}
