package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoLevelOnChargingStation;
import frc.robot.commands.IntakeConeFromFloorInstant;
import frc.robot.commands.IntakeCubeFromFloorInstant;
import frc.robot.commands.ReturnWAEHomeTimeout;
import frc.robot.commands.ScoreConeCubeHighRelease;
import frc.robot.commands.ScoreCubeHighAuto;
import frc.robot.commands.ScoreCubeHighAutoRelease;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.FullSystemCommandsTeleop.IntakeConeFromFloor;
import frc.robot.commands.FullSystemCommandsTeleop.IntakeConeFromShelf;
import frc.robot.commands.FullSystemCommandsTeleop.IntakeCubeFromFloor;
import frc.robot.commands.FullSystemCommandsTeleop.ReturnWAEHome;
import frc.robot.commands.FullSystemCommandsTeleop.ScoreConeCubeHigh;
import frc.robot.commands.FullSystemCommandsTeleop.ScoreConeCubeMid;
import frc.robot.commands.WristCommands.ScoreCone;
import frc.robot.commands.WristCommands.ScoreCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
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

    public static Spark blinkin = new Spark(2);

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
    public static Swerve s_Swerve = new Swerve();
    public static Elevator elevator = new Elevator();
    public static Wrist wrist = new Wrist();
    public static Intake intake = new Intake();
    public static Arm arm = new Arm();
    public static Limelight limelight = new Limelight();
    
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

        //Shuffleboard Tabs. Default ones are SmartDashboard and LiveWindow
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight Data");


        CameraServer.startAutomaticCapture();

        // Map all of the events referenced by PathPlanner to their respective commands
        eventMap.put("IntakeConeFromFloor", new IntakeConeFromFloor());
        eventMap.put("IntakeCubeFromFloor", new IntakeCubeFromFloor());
        eventMap.put("IntakeConeFromFloorInstant", new IntakeConeFromFloorInstant());
        eventMap.put("IntakeCubeFromFloorInstant", new IntakeCubeFromFloorInstant());
        eventMap.put("ReturnWAEHome", new ReturnWAEHome());
        eventMap.put("ReturnWAEHomeTimeout", new ReturnWAEHomeTimeout());
        eventMap.put("ScoreConeCubeHigh", new ScoreConeCubeHigh());
        eventMap.put("ScoreConeCubeMid", new ScoreConeCubeMid());
        eventMap.put("ScoreConeCubeHighRelease", new ScoreConeCubeHighRelease());
        eventMap.put("AutoLevelOnChargingStation", new AutoLevelOnChargingStation());
        eventMap.put("ScoreCubeHighAuto", new ScoreCubeHighAuto());
        eventMap.put("ScoreCubeHighAutoRelease", new ScoreCubeHighAutoRelease());


        double autoVelocityConstraint = 2.5;
        double autoAccelerationConstraint = 2.7;

        // Build out sendable chooser commands for each of the generated PathPlanner routines
        /*File[] fileList = Filesystem.getDeployDirectory().toPath().resolve("output/").toFile().listFiles();
        for (File file : fileList) {
            if (file.getName().endsWith(".path")) {
                autoChooser.addOption(file.getName().replace(".path", ""), autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup(file.getName().replace(".path", ""), new PathConstraints(autoVelocityConstraint, autoAccelerationConstraint)))));
            }
        }*/

        autoChooser.addOption("Two Piece Left", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Two Piece Red+Blue Left", new PathConstraints(autoVelocityConstraint, autoAccelerationConstraint)))));
        autoChooser.addOption("Two Piece Right", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Two Piece Red+Blue Right", new PathConstraints(autoVelocityConstraint, autoAccelerationConstraint)))));
        autoChooser.addOption("Score and Drive Back", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Score and Drive Back", new PathConstraints(autoVelocityConstraint, autoAccelerationConstraint)))));
        autoChooser.addOption("One Piece Level Left", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("One Piece Level Left", new PathConstraints(autoVelocityConstraint, autoAccelerationConstraint)))));
        autoChooser.addOption("One Piece Level Right", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("One Piece Level Right", new PathConstraints(autoVelocityConstraint, autoAccelerationConstraint)))));
        autoChooser.addOption("Calibrate Path Distance", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Calibrate Path Distance", new PathConstraints(autoVelocityConstraint, autoAccelerationConstraint)))));
        //autoChooser.addOption("TEST: Score Cone Timing", new ScoreConeCubeHighRelease());
        //autoChooser.addOption("TEST: Shoot Cube High", new ScoreCubeHighAuto());
        //autoChooser.addOption("TEST: Shoot Cone High", new ScoreConeCubeHighRelease());


        SmartDashboard.putData(autoChooser);

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
    private void configureButtonBindings() { //These are driver controls

        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        driverController.leftTrigger().whileTrue(new TeleopSwerve( //Drive robot centric
            s_Swerve,
            () -> driverController.getRawAxis(translationAxis),
            () -> driverController.getRawAxis(strafeAxis),
            () -> -driverController.getRawAxis(rotationAxis),
            () -> true
            ));

        driverController.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driverController.x().onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        driverController.rightBumper().onTrue(new IntakeCubeFromFloor());
        driverController.leftBumper().onTrue(new IntakeConeFromFloor());
        driverController.b().onTrue(new ReturnWAEHomeTimeout());
        //driverController.start().onTrue(new AutoLevelOnChargingStation());

        /* Operator Buttons */
        operatorController.b().onTrue(new ReturnWAEHomeTimeout());
        operatorController.rightBumper().whileTrue(new ScoreCube());
        operatorController.leftBumper().whileTrue(new ScoreCone());
        operatorController.a().onTrue(new ScoreConeCubeMid());
        operatorController.x().onTrue(new ScoreConeCubeHigh());
        operatorController.y().onTrue(new IntakeConeFromShelf());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();

        //return new exampleAuto(s_Swerve);
    }
}
