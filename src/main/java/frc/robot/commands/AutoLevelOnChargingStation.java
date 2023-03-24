// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoLevelOnChargingStation extends CommandBase {
  /** Creates a new AutoLevelOnChargingStation. */
  private double error;
  private double currentAngle;
  private double drivePower;
  private double kP = 0.005875;

  public AutoLevelOnChargingStation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve);
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      RobotContainer.blinkin.set(-0.95);
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      RobotContainer.blinkin.set(-0.93);
    } else {
      RobotContainer.blinkin.set(-0.99);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running AutoLevel");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentAngle = RobotContainer.s_Swerve.getPitch();

    error = 0 - currentAngle;
    drivePower = Math.min(kP * error, 1);

    //Limit max zoom
    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
    }

    RobotContainer.s_Swerve.drive(
      new Translation2d(drivePower, 0).times(4.5),
      0,
      true,
      true
  );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoLevel completed");

    RobotContainer.s_Swerve.drive(
      new Translation2d(0, 0).times(4.5),
      0,
      true,
      true
  );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //commenting out lines 75 through 77 should allow it to balance indefinitely; let driver take over in teleop if needed
    //if (Math.abs(error) < 1) {
    //  return true;
    //}
    if (RobotContainer.driver.getRawAxis(XboxController.Axis.kLeftY.value) > 0.1) {
      return true;
    }
    else {
      return false;
    }
  }
}
