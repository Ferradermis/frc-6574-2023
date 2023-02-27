package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeConeFromFloor;
import frc.robot.commands.IntakeCubeFromFloor;
import frc.robot.commands.ReturnWAEHome;
import frc.robot.commands.ScoreConeCubeHigh;
import frc.robot.commands.ScoreConeCubeMid;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WristCommands.setWristIntakeSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public static CommandXboxController driverController = new CommandXboxController(0);
    public static CommandXboxController operatorController = new CommandXboxController(1);
    public static Joystick driver = new Joystick(0);
    public static Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton elevatorPositionTest = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton armPositionTest = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    //private final JoystickButton wristPositionTest = new JoystickButton(operator, XboxController.Button.kRightBumper.value); */


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    public static Elevator elevator = new Elevator();
    public static Wrist wrist = new Wrist();
    public static Arm arm = new Arm();

    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        s_Swerve::getPose, // Pose2d supplier
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(Constants.AutoConstants.kPYController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );
    SendableChooser<CommandBase> autoChooser = new SendableChooser<>();



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        eventMap.put("IntakeConeFromFloor", new IntakeConeFromFloor());
        eventMap.put("IntakeCubeFromFloor", new IntakeCubeFromFloor());
        eventMap.put("ReturnWAEHome", new ReturnWAEHome());
        eventMap.put("ScoreConeCubeHigh", new ScoreConeCubeHigh());
        eventMap.put("ScoreConeCubeMid", new ScoreConeCubeMid());


        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driverController.getRawAxis(translationAxis),
                () -> -driverController.getRawAxis(strafeAxis),
                () -> -driverController.getRawAxis(rotationAxis),
                () -> false
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        driverController.leftTrigger().whileTrue(new TeleopSwerve(
            s_Swerve,
            () -> -driverController.getRawAxis(translationAxis),
            () -> -driverController.getRawAxis(strafeAxis),
            () -> -driverController.getRawAxis(rotationAxis),
            () -> true
        ));
        driverController.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driverController.x().onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        driverController.rightBumper().onTrue(new IntakeCubeFromFloor());
        driverController.leftBumper().onTrue(new IntakeConeFromFloor());
        driverController.b().onTrue(new ReturnWAEHome());

        /* Operator Buttons */
        operatorController.b().onTrue(new ReturnWAEHome());
        operatorController.rightBumper().whileTrue(new setWristIntakeSpeed(1));
        operatorController.leftBumper().whileTrue(new setWristIntakeSpeed(-1));
        operatorController.a().onTrue(new ScoreConeCubeMid());
        operatorController.b().onTrue(new ScoreConeCubeHigh());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        ArrayList<PathPlannerTrajectory> pathGroup = new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Event Test", new PathConstraints(1, 2)));
        return autoBuilder.fullAuto(pathGroup);

        //return new exampleAuto(s_Swerve);
    }
}
