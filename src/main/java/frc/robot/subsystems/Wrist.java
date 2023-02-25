// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {

  public static CANSparkMax wristMotor;
  public static CANSparkMax intakeMotor;
  public static AbsoluteEncoder m_AbsoluteEncoder;
  //private RelativeEncoder wristEncoder;

  private SparkMaxPIDController wristPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private double maxSpeed = 0.25;
  private double deadBand = 0.1;

  /** Creates a new Intake. */
  public Wrist() {
    wristMotor = new CANSparkMax(Constants.RobotConstants.wristMotorCANID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.RobotConstants.intakeMotorCANID, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    wristPIDController = wristMotor.getPIDController();
    m_AbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristPIDController.setFeedbackDevice(m_AbsoluteEncoder);


    //m_AbsoluteEncoder.setPositionConversionFactor(360);
    //m_AbsoluteEncoder.setVelocityConversionFactor(1);
    wristMotor.setInverted(true);
    wristMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    m_AbsoluteEncoder.setZeroOffset(0.6526145);


    wristMotor.setSmartCurrentLimit(80);
    intakeMotor.setSmartCurrentLimit(80);

    kP = 2;
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = .5; 
    kMinOutput = -.5;
    
    wristPIDController.setP(kP);
    wristPIDController.setI(kI);
    wristPIDController.setD(kD);
    wristPIDController.setIZone(kIz);
    wristPIDController.setFF(kFF);
    wristPIDController.setOutputRange(kMinOutput, kMaxOutput);

    wristPIDController.setPositionPIDWrappingEnabled(true);
    wristPIDController.setPositionPIDWrappingMinInput(0);
    wristPIDController.setPositionPIDWrappingMaxInput(1);
  }

  @Override

  public void periodic() {
    //SmartDashboard.putNumber("Wrist", getAbsoluteEncoderCounts());
    //SmartDashboard.putNumber("Wrist Joystick", RobotContainer.operator.getRawAxis(5));
    SmartDashboard.putNumber("Wrist encoder", wristMotor.getEncoder().getPosition());

    if (RobotContainer.operator.getRawAxis(5) > deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(5) < -deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    }


    


    if (RobotContainer.operator.getRawButtonPressed(1)) {
      intakeMotor.set(1);
    }
      else if (RobotContainer.operator.getRawButtonReleased(1)) {
        intakeMotor.set(0);
      }

      if (RobotContainer.operator.getRawButtonPressed(2)) {
        intakeMotor.set(-1);
      }
      else if (RobotContainer.operator.getRawButtonReleased(2)) {
        intakeMotor.set(0);
      }
    }
    
  

  public void setSpeed(double speed)
  {
    wristMotor.set(speed);
  }

  public void stop()
  {
    wristMotor.stopMotor();
  }
/*   public double getAbsoluteEncoderCounts()
  {
    return m_AbsoluteEncoder.getPosition();
  } */


  public void setPosition(double position) {
    wristPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }
  
}
