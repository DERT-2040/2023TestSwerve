package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtendSubsystem extends SubsystemBase {
    
    CANSparkMax extendMotor;
    int extendID = 21; //3;//21;
    private RelativeEncoder extendEncoder;
    SparkMaxPIDController extendController;
    double desiredPosition;

    public IntakeExtendSubsystem() {
        extendMotor = new CANSparkMax(extendID, MotorType.kBrushless); //extendMotor = new Spark(extendID);//
        extendMotor.setIdleMode(IdleMode.kBrake);
        extendMotor.setSecondaryCurrentLimit(20);
        extendMotor.setSmartCurrentLimit(20);
        extendEncoder = extendMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        extendEncoder.setPosition(0);
        
        //extendEncoder.setPositionConversionFactor(1/);
        extendController = extendMotor.getPIDController();
        extendController.setP(.015);
        extendController.setI(0);
        extendController.setD(0);
        desiredPosition = 0;
    }

    public void extend(double speed) {
        extendMotor.set(speed);
        
    }

    public void goToPosition(int chosenLocation) {
        desiredPosition = chosenLocation;
        //extendController.setReference(chosenLocation, ControlType.kPosition);
        //SmartDashboard.putNumber("Intake Position", extendEncoder.getPosition());
        //SmartDashboard.putNumber("Target Intake Position", chosenLocation);
    }

    public void periodic() {
        extendController.setReference(desiredPosition, ControlType.kPosition);
    }

}

