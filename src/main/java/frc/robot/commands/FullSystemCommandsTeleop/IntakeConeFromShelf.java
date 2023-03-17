// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.SetBlinkin;
import frc.robot.commands.ArmCommands.SetArmPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.WristCommands.IntakeCone;
import frc.robot.commands.WristCommands.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeConeFromShelf extends SequentialCommandGroup {
  /** Creates a new ScoreConeMid. */
  public IntakeConeFromShelf() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetBlinkin(0.69),
      new SetElevatorPosition(RobotConstants.ELEVATOR_HIGH_POSITION),
      new ParallelCommandGroup(
                               new SetWristPosition(RobotConstants.WRIST_CONE_FLOOR_INTAKE_POSITION),
                               new SetArmPosition(RobotConstants.ARM_CONE_HIGHSCORE_POSITION),
                               new IntakeCone())
      );
      //new SetArmPosition(RobotConstants.ARM_CUBE_FLOOR_INTAKE_POSITION)
  }
}
