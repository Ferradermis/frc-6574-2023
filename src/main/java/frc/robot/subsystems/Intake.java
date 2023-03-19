// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public CANSparkMax intakeMotor;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.RobotConstants.intakeMotorCANID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(25);


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setOutakeSpeed() {
    intakeMotor.set(-1);
  }
}
