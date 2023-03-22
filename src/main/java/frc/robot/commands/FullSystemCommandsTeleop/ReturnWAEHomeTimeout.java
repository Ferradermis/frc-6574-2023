// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ArmCommands.SetArmPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.WristCommands.SetWristIntakeSpeedInstant;
import frc.robot.commands.WristCommands.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnWAEHomeTimeout extends SequentialCommandGroup {
  /** Creates a new ReturnWAEHome. */
  public ReturnWAEHomeTimeout() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetWristIntakeSpeedInstant(0),
      new SetArmPosition(0).withTimeout(0.3),
      new SetElevatorPosition(0).withTimeout(0.3),
      new WaitCommand(.1),
      new SetWristPosition(RobotConstants.WRIST_HOME_POSITION).withTimeout(0.3),
      new InstantCommand(() -> RobotContainer.arm.stop()),
      new InstantCommand(() -> RobotContainer.elevator.stopMotors())
    );
  }
}
