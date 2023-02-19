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

public class Arm extends SubsystemBase {

  public static CANSparkMax armMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  /** Creates a new Intake. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.RobotConstants.armMotorCANID, MotorType.kBrushless);
    m_AbsoluteEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Arm", getAbsoluteEncoderPosition());
    
  }

  public void setSpeed(double speed) {
    armMotor.set(speed);
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public double getAbsoluteEncoderPosition() {
    return m_AbsoluteEncoder.getPosition();
  }
}
