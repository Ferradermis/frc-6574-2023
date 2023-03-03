// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ArmCommands.SetArmPositionInstant;
import frc.robot.commands.ElevatorCommands.SetElevatorPositionInstant;
import frc.robot.commands.WristCommands.SetWristIntakeSpeedInstant;
import frc.robot.commands.WristCommands.SetWristPositionInstant;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeConeFromFloorInstant extends SequentialCommandGroup {
  /** Creates a new IntakeCubeFromFloor. */
  public IntakeConeFromFloorInstant() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelCommandGroup(
      new SetElevatorPositionInstant(RobotConstants.ELEVATOR_INTAKECUBE_POSITION),
      new SetWristPositionInstant(RobotConstants.WRIST_CONE_FLOOR_INTAKE_POSITION)),
      new SetArmPositionInstant(RobotConstants.ARM_CUBE_FLOOR_INTAKE_POSITION),
      new SetWristIntakeSpeedInstant(-1)
    );
  }
}
