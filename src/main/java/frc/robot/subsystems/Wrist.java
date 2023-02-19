// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {

  public static CANSparkMax wristMotor;
  public static CANSparkMax intakeMotor;
  public static AbsoluteEncoder m_AbsoluteEncoder;
  
  private double maxSpeed = 0.25;
  private double deadBand = 0.1;

  /** Creates a new Intake. */
  public Wrist() {
    wristMotor = new CANSparkMax(Constants.RobotConstants.wristMotorCANID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.RobotConstants.intakeMotorCANID, MotorType.kBrushless);
    m_AbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    wristMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();
    
    wristMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);


    
  }

  @Override

  public void periodic() {
    SmartDashboard.putNumber("Wrist", getAbsoluteEncoderCounts());

    if (RobotContainer.operator.getRawAxis(1) > deadBand) {
      intakeMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(1) < -deadBand) {
      intakeMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    } else {
      intakeMotor.stopMotor();
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

  public double getAbsoluteEncoderCounts()
  {
    return m_AbsoluteEncoder.getPosition();
  }
}
