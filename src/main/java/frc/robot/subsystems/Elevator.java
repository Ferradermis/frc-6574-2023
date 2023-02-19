// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

  public CANSparkMax leftMotor;
  public CANSparkMax rightMotor;

  private SparkMaxPIDController elevatorPIDController;
  private RelativeEncoder elevatorEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private double maxSpeed = 0.25;
  private double deadBand = 0.1;

  /** Creates a new Elevator. Gatorvator*/
  public Elevator() {
    leftMotor = new CANSparkMax(Constants.RobotConstants.elevatorLeftMotorCANID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.RobotConstants.elevatorRightMotorCANID, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);

    elevatorPIDController = leftMotor.getPIDController();
    elevatorEncoder = leftMotor.getEncoder();

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    elevatorPIDController.setP(kP);
    elevatorPIDController.setI(kI);
    elevatorPIDController.setD(kD);
    elevatorPIDController.setIZone(kIz);
    elevatorPIDController.setFF(kFF);
    elevatorPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }


  public void driveElevator(double speed) {
    rightMotor.set(speed * maxSpeed);
  }

  public void setPosition(double position) {
    elevatorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
  @Override
  public void periodic() {
    /* 
    if (RobotContainer.operator.getRawAxis(1) > deadBand) {
      leftMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(1) < -deadBand) {
      leftMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    } else {
      leftMotor.stopMotor();
    }
*/
    // This method will be called once per scheduler run
  }


}
