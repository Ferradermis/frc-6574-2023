// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public CANSparkMax leftMotor;
  public CANSparkMax rightMotor;
  private SparkMaxLimitSwitch elevatorReverseLimit;
  private SparkMaxPIDController elevatorPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private double maxSpeed = 0.25;
  private float maxElevatorExtension = 32;

  /** Creates a new Elevator. Gatorvator*/
  public Elevator() {
    leftMotor = new CANSparkMax(Constants.RobotConstants.elevatorLeftMotorCANID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.RobotConstants.elevatorRightMotorCANID, MotorType.kBrushless);

    elevatorReverseLimit = leftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(25);
    rightMotor.setSmartCurrentLimit(25);

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);

    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, maxElevatorExtension);

    leftMotor.getEncoder().setPosition(0);

    elevatorPIDController = leftMotor.getPIDController();
    leftMotor.getEncoder();

    kP = 0.15;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = .5;
    kMinOutput = -.5;

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

    elevatorReverseLimit.enableLimitSwitch(true);
    SmartDashboard.putNumber("Elevator encoder", leftMotor.getEncoder().getPosition());

    /*     if (RobotContainer.operator.getRawButtonPressed(3)) {
      leftMotor.set(.15);
    }
      else if (RobotContainer.operator.getRawButtonReleased(3)) {
        leftMotor.set(0);
      }

      if (RobotContainer.operator.getRawButtonPressed(4)) {
        leftMotor.set(-.15);
      }
      else if (RobotContainer.operator.getRawButtonReleased(4)) {
        leftMotor.set(0);
      } */

    }


    // This method will be called once per scheduler run
  }



