package raidzero.robot.subsystems;

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

    public static Trigger READY_FOR_REEF = (intake.hasCoral().or(intake.hasAlgae().negate()))
        .and(swerve.atReef().negate());

    public static Trigger AT_REEF_READY_ARM = (intake.hasCoral().or(intake.hasAlgae().negate()))
        .and(swerve.atReef())
        .and(arm.atL4().or(arm.atL3()).or(arm.atL2()).or(arm.atL1()).or(arm.atL2Algae()).or(arm.atL3Algae()).negate());

    public static Trigger AT_REEF_READY_ROLLERS = (intake.hasCoral().or(intake.hasAlgae().negate()))
        .and(swerve.atReef())
        .and(arm.atL4().or(arm.atL3()).or(arm.atL2()).or(arm.atL1()));

    public static Trigger WANTS_GAMEPEICE = intake.hasCoral().negate()
        .and(swerve.atStation().negate())
        .and(arm.atHome());

    public static Trigger AT_STATION_READY_ARM = intake.hasCoral().negate()
        .and(swerve.atStation())
        .and(arm.atStation().negate());

    public static Trigger AT_STATION_READY_INTAKE = intake.hasCoral().negate()
        .and(swerve.atStation())
        .and(arm.atStation());

    public static Trigger AT_GROUND_READY_INTAKE = intake.hasCoral().negate()
        .and(arm.atGroundIntake());

    public static Trigger HAS_ALGAE = intake.hasAlgae()
        .and(arm.atHome())
        .and(swerve.atProcessor().negate());

    public static Trigger AT_PROCESSOR_READY_ARM = intake.hasAlgae()
        .and(swerve.atProcessor())
        .and(arm.atProcessor().negate());

    public static Trigger AT_PROCESSOR_READY_ROLLERS = intake.hasAlgae()
        .and(swerve.atProcessor())
        .and(arm.atProcessor());
}