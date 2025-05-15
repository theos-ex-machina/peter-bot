package raidzero.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.robot.subsystems.arm.Arm;
import raidzero.robot.subsystems.arm.Intake;
import raidzero.robot.subsystems.drivetrain.Swerve;

/**
 * Defines the robot state as a combination of all the subsystem's state
 */
public class Superstructure {
    private static Swerve swerve = Swerve.system();
    private static Intake intake = Intake.system();
    private static Arm arm = Arm.system();

    public static Trigger WITH_CORAL_READY_REEF = intake.hasCoral()
        .and(swerve.atReef().negate());

    public static Trigger AT_REEF_READY_ARM = intake.hasCoral()
        .and(swerve.atReef())
        .and(arm.atL4().or(arm.atL3()).or(arm.atL2()).or(arm.atL1()).negate());

    public static Trigger AT_REEF_READY_EXTAKE = intake.hasCoral()
        .and(swerve.atReef())
        .and(arm.atL4().or(arm.atL3()).or(arm.atL2()).or(arm.atL1()));

    public static Trigger WANTS_CORAL = intake.hasCoral().negate()
        .and(swerve.atStation().negate());

    public static Trigger AT_STATION_READY_ARM = intake.hasCoral().negate()
        .and(swerve.atStation())
        .and(arm.atStation().negate());

    public static Trigger AT_STATION_READY_INTAKE = intake.hasCoral().negate()
        .and(swerve.atStation())
        .and(arm.atStation());
}