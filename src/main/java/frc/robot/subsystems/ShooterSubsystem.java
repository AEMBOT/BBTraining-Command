package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    ColorSensorV3 indexerSensor = new ColorSensorV3(I2C.Port.kMXP);
    
    CANSparkMax conveyorMotor = new CANSparkMax(Constants.conveyorPort, MotorType.kBrushless);
    CANSparkMax flywheelMotor = new CANSparkMax(Constants.FlywheelMotorPort, MotorType.kBrushless);

    Boolean indexerOveride = false;

    PIDController flywheelPID = new PIDController(Constants.shooterKP, Constants.shooterKI, Constants.shooterKP);
    SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(Constants.shooterKS, Constants.shooterKV,Constants.shooterKA);

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

    public void IndexerOverideOn(){
        indexerOveride = true;
    }

    public void IndexerOverideOff(){
        indexerOveride = false;
    }

    public boolean GetIndexerOveride(){
        return indexerOveride;
    }

    public void SetFlywheelSpeed(Double speed) {
        flywheelMotor.setVoltage(flywheelPID.calculate(flywheelMotor.getEncoder().getVelocity(),speed)
        +flywheelFeedForward.calculate(speed));
    }

    public void FlywheelOff() {
        flywheelMotor.setVoltage(0);
    }
}