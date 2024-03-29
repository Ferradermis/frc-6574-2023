package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;


public class TeleopSwervePrecise extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double trainingWheels = .1;

    public TeleopSwervePrecise(Swerve s_Swerve,
                        DoubleSupplier translationSup,
                        DoubleSupplier strafeSup,
                        DoubleSupplier rotationSup,
                        BooleanSupplier robotCentricSup
                        ) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = trainingWheels * MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = trainingWheels * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = trainingWheels * MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double speedLimit = Constants.Swerve.maxSpeed;
        double angularVelocityLimit = (Constants.Swerve.maxAngularVelocity);
        // check if any other tippy commands are scheduled, override with a lower value
        if (RobotContainer.elevator.leftMotor.getEncoder().getPosition() >= 5) {
            speedLimit *= 0.3;
            //angularVelocityLimit *= 0.3;
        }
        //else if (RobotContainer.driver.getTrigger() == true) {
        //    speedLimit *= 1.5;
        //}

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(speedLimit),
            rotationVal * angularVelocityLimit,
            !robotCentricSup.getAsBoolean(),
            true
        );
    }
}