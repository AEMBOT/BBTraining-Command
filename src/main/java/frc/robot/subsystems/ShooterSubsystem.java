package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    ColorSensorV3 indexerSensor = new ColorSensorV3(I2C.Port.kMXP);
    
    CANSparkMax conveyorMotor = new CANSparkMax(Constants.conveyorPort, MotorType.kBrushless);

    Boolean indexerOveride = false;

    @Override 
    public void periodic() {
       if(indexerOveride == false){
        if(indexerSensor.getProximity()<=Constants.maxProximity){
            conveyorMotor.setVoltage(0);
        } else{ 
            conveyorMotor.setVoltage(1);
        }
       }
        
    }
}