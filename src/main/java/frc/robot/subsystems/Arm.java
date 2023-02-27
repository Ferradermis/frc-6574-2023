// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public CANSparkMax armMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  //private final RelativeEncoder armEncoder;
  private SparkMaxPIDController armPIDController;

  //private double maxSpeed = 0.25;
  //private double deadBand = 0.1;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new Intake. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.RobotConstants.armMotorCANID, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();

    armPIDController = armMotor.getPIDController();
    m_AbsoluteEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    armPIDController.setFeedbackDevice(m_AbsoluteEncoder);

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(true);
    armMotor.setSmartCurrentLimit(25);
    m_AbsoluteEncoder.setZeroOffset(0.2721140);


    //armMotor.getEncoder().setPosition(0);


    kP = 3;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = .5;
    kMinOutput = -.5;

    armPIDController.setP(kP);
    armPIDController.setI(kI);
    armPIDController.setD(kD);
    armPIDController.setIZone(kIz);
    armPIDController.setFF(kFF);
    armPIDController.setOutputRange(kMinOutput, kMaxOutput);

    armPIDController.setPositionPIDWrappingEnabled(true);
    armPIDController.setPositionPIDWrappingMinInput(0);
    armPIDController.setPositionPIDWrappingMaxInput(1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Arm encoder", armMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("Arm", getAbsoluteEncoderPosition());

    /*  if (RobotContainer.operator.getRawAxis(1) > deadBand) {
      armMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(1) < -deadBand) {
      armMotor.set(-RobotContainer.operator.getRawAxis(1) * maxSpeed);
    }  */

    /* else {
      armMotor.stopMotor();
    } */

  }

  public void setSpeed(double speed) {
    armMotor.set(speed);
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public void setPosition(double position) {
    armPIDController.setReference(position, ControlType.kPosition);
  }
  public double getAbsoluteEncoderPosition() {
    return m_AbsoluteEncoder.getPosition();
  }
}
