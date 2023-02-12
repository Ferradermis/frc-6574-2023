// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

  public CANSparkMax leftMotor;
  public CANSparkMax rightMotor;

  private double maxSpeed = 0.25;
  private double deadBand = 0.1;

  /** Creates a new Elevator. Gatorvator*/
  public Elevator() {
    leftMotor = new CANSparkMax(19, MotorType.kBrushless);
    rightMotor = new CANSparkMax(11, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);
  }


  public void driveElevator(double speed) {
    rightMotor.set(speed * maxSpeed);
  }

  @Override
  public void periodic() {

    if (RobotContainer.operator.getRawAxis(1) > deadBand) {
      leftMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(1) < -deadBand) {
      leftMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    } else {
      leftMotor.stopMotor();
    }

    // This method will be called once per scheduler run
  }
}
