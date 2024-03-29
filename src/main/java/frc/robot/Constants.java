package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.05;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false;
        public static final double swerveSpeedModifier = .6; // Always ensure Gyro is CCW+ CW- 
        public static final double turboSpeedModifier = 1;
        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.15848 / 12);
        public static final double driveKV = (2.2698 / 12);
        public static final double driveKA = (0.32845 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(338.29);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(112.23);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(224.47);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(116.36);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final class RobotConstants {

        public static final int elevatorLeftMotorCANID = 19;
        public static final int elevatorRightMotorCANID = 11;
        public static final int wristMotorCANID = 3;
        public static final int armMotorCANID = 2;
        public static final int intakeMotorCANID = 1;

        /* Elevator Position constants */
        //Gatorvator 🐊

        public static final double ELEVATOR_HOME_POSITION = 0;
        public static final double ELEVATOR_INTAKECUBE_POSITION = 5.023;
        public static final double ELEVATOR_LOW_POSITION = 0;
        public static final double ELEVATOR_MID_POSITION = 20.00;
        public static final double ELEVATOR_HIGH_POSITION = 32;
        public static final double ELEVATOR_LOW_SCORING_POSITION = 2.61;
        public static final double ELEVATOR_CUBE_CHUTE_POSITION = 20.00;
        public static final double ELEVATOR_CONE_CHUTE_POSITION = 24.85;

        /* Wrist position constants */
        public static final double WRIST_HOME_POSITION = .042;
        public static final double WRIST_CONE_FLOOR_INTAKE_POSITION = .18;
        public static final double WRIST_CONE_LOWSCORE_POSITION = 0;
        public static final double WRIST_CONE_MIDSCORE_POSITION = .12;
        public static final double WRIST_CONE_HIGHSCORE_POSITION = .08;
        

        public static final double WRIST_CUBE_FLOOR_INTAKE_POSITION = .180;
        public static final double WRIST_CUBE_LOWSCORE_POSITION = 0;
        public static final double WRIST_CUBE_MIDSCORE_POSITION = 0;
        public static final double WRIST_CUBE_HIGHSCORE_POSITION = 0;
        public static final double WRIST_CUBE_NEW_LOWSCORE_POSITION = 0.07;

        public static final double WRIST_CONE_CHUTE_POSITION = .121; //working at .121
        public static final double WRIST_CHUTE_POSITION = .998; //.03 works for cone intake if we drive back a bit; .998 is working value
        public static final double WRIST_SHELF_POSITION = .175;

        /* Wrist intake constants */
        public static final double INTAKE_CONE_SPEED = -1;
        public static final double SCORE_CONE_SPEED = 1;

        public static final double INTAKE_CUBE_SPEED = 1;
        public static final double SCORE_CUBE_SPEED = -1;


        /* Arm position constants */
        public static final double ARM_HOME_POSITION = 0;
        public static final double ARM_CONE_FLOOR_INTAKE_POSITION = .026;
        public static final double ARM_CONE_LOWSCORE_POSITION = 0;
        public static final double ARM_CONE_MIDSCORE_POSITION = .1;
        public static final double ARM_CONE_HIGHSCORE_POSITION = .15;

        public static final double ARM_CUBE_FLOOR_INTAKE_POSITION = 0;
        public static final double ARM_CUBE_LOWSCORE_POSITION = 0;
        public static final double ARM_CUBE_MIDSCORE_POSITION = 0;
        public static final double ARM_CUBE_HIGHSCORE_POSITION = 0;
        public static final double ARM_CUBE_NEW_LOWSCORE_POSITION = 0.03;

        public static final double ARM_CHUTE_POSITION = 0;
        public static final double ARM_CONE_CHUTE_POSITION = 0.12; //working at .12
        public static final double ARM_SHELF_POSITION = 0.16;
    }
}
