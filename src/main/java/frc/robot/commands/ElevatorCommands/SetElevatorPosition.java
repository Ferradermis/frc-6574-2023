// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetElevatorPosition extends CommandBase {
  private double position;
  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevator);
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Beginning SetElevatorPosition");
    RobotContainer.elevator.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.elevator.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.elevator.leftMotor.getEncoder().getPosition() >= (position - 1)) { //TODO CHANGE TO TOLERANCE BASED ON SUPPLIED POSITION FROM ROBOT CONTAINER
      System.out.println("SetElevatorPosition Complete");
      return true;
    }
    else {
      return false;
    }
  }
}
