package raidzero.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static raidzero.robot.subsystems.Superstructure.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import javax.naming.directory.AttributeModificationException;
import raidzero.robot.subsystems.arm.Arm;
import raidzero.robot.subsystems.arm.ArmConstants;
import raidzero.robot.subsystems.arm.ArmConstants.Positions;
import raidzero.robot.subsystems.arm.Intake;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.drivetrain.TunerConstants;

public class Bindings {
    private static final Swerve swerve = Swerve.system();
    private static final Arm arm = Arm.system();
    // private static final Intake intake = Intake.system();

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static void setButtonBinds() {
        DRIVER_WANTS_CONTROL.whileTrue(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // L4.and(READY_FOR_REEF)
        // .whileTrue(swerve.goToNearestReef(4));

        // L4.and(AT_REEF_READY_ARM)
        // .whileTrue(arm.interpolateTo(Positions.L4_INTERPOLATION_PATH, Positions.L4_WRIST_ANGLE));

        // L4.and(AT_REEF_READY_ROLLERS)
        // .whileTrue(intake.extakeCoral());

        // STATION.and(WANTS_GAMEPEICE)
        // .whileTrue(swerve.goToNearestStation());

        // STATION.and(AT_STATION_READY_ARM)
        // .whileTrue(arm.moveTo(Positions.STATION, Positions.STATION_WRIST_ANGLE));

        // STATION.and(AT_STATION_READY_INTAKE)
        // .whileTrue(intake.intakeCoral());

        // // allow driver movment for ground intake
        // GROUND_INTAKE.and(WANTS_GAMEPEICE)
        // .whileTrue(arm.moveTo(Positions.GROUND_INTAKE, Positions.GROUND_INTAKE_WRIST_ANGLE));

        // // TODO: Add autopath logic to ground intake
        // GROUND_INTAKE.and(AT_GROUND_READY_INTAKE)
        // .whileTrue(intake.intakeCoral());

        // L2_ALGAE.and(READY_FOR_REEF)
        // .whileTrue(swerve.goToNearestL2Algae());

        // L2_ALGAE.and(AT_REEF_READY_ARM)
        // .whileTrue(arm.moveTo(Positions.L2_ALGAE, Positions.L2_ALGAE_WRIST_ANGLE));

        // L2_ALGAE.and(AT_REEF_READY_ROLLERS)
        // .whileTrue(intake.intakeAlgae());

        // PROCESSOR.and(HAS_ALGAE)
        // .whileTrue(swerve.goToProcessor());

        // PROCESSOR.and(AT_PROCESSOR_READY_ARM)
        // .whileTrue(arm.moveTo(Positions.PROCESSOR, Positions.PROCESSOR_WRIST_ANGLE));

        // PROCESSOR.and(AT_PROCESSOR_READY_ROLLERS)
        // .whileTrue(intake.extakeAlgae());

    }

    public static void setDefaultCommands() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // // auto home when let go or obtain algae or coral
        arm.setDefaultCommand(arm.home());
        // intake.setDefaultCommand(intake.stop());

        // intake.hasCoral().onTrue(arm.home());
        // intake.hasAlgae().onTrue(arm.home());

        // intake.hasCoral().onFalse(arm.home());
        // intake.hasAlgae().onFalse(arm.home());
    }

    public static void applyTempDriverOnlyBindings() {
        // driver.b().onTrue(arm.moveTo(ArmConstants.Positions.L1));
        driver.x().onTrue(arm.moveTo(Positions.L2));
        driver.y().onTrue(arm.moveTo(Positions.L3));
        driver.b().onTrue(arm.moveTo(Positions.L4));
        driver.a().onTrue(arm.home());
    }

    public static void applyManualBindings() {
        L1.onTrue(arm.moveTo(Positions.L1));
        L2.onTrue(arm.moveTo(Positions.L2));
        L3.onTrue(arm.moveTo(Positions.L3));

        STATION.whileTrue(arm.moveTo(Positions.STATION));
        GROUND_INTAKE.whileTrue(arm.moveTo(Positions.GROUND_INTAKE));

        PROCESSOR.whileTrue(arm.moveTo(Positions.PROCESSOR));
        BARGE.whileTrue(arm.moveTo(Positions.BARGE));

        // CORAL_INTAKE.whileTrue(intake.intakeCoral());
        // CORAL_EXTAKE.whileTrue(intake.extakeCoral());

        // ALGAE_INTAKE.whileTrue(intake.intakeAlgae());
        // ALGAE_EXTAKE.whileTrue(intake.extakeAlgae());

        L3_ALGAE.whileTrue(arm.moveTo(Positions.L3_ALGAE));
        L2_ALGAE.whileTrue(arm.moveTo(Positions.L2_ALGAE));
    }

    public static CommandXboxController driver = new CommandXboxController(0);

    /** If the driver pulls any of the sticks */
    public static Trigger DRIVER_WANTS_CONTROL = driver.axisGreaterThan(0, 0.1)
        .or(driver.axisGreaterThan(1, 0.1))
        .or(driver.axisGreaterThan(4, 0.1))
        .or(driver.axisGreaterThan(5, 0.1));

    public static CommandGenericHID operator = new CommandGenericHID(1);

    public static Trigger L1 = operator.button(6).and(DRIVER_WANTS_CONTROL.negate());
    public static Trigger L2 = operator.button(7).and(DRIVER_WANTS_CONTROL.negate());
    public static Trigger L3 = operator.button(8).and(DRIVER_WANTS_CONTROL.negate());
    public static Trigger L4 = operator.button(9).and(DRIVER_WANTS_CONTROL.negate());

    public static Trigger L3_ALGAE = operator.axisGreaterThan(0, 0.5).and(DRIVER_WANTS_CONTROL.negate());
    public static Trigger L2_ALGAE = operator.axisGreaterThan(1, 0.5).and(DRIVER_WANTS_CONTROL.negate());

    /** Button labeled "home" */
    public static Trigger STATION = operator.button(12).and(DRIVER_WANTS_CONTROL.negate());
    /** Button labeled "bottom right" */
    public static Trigger GROUND_INTAKE = operator.button(14);

    /** Button labeled "bottom left" */
    public static Trigger PROCESSOR = operator.button(15).and(DRIVER_WANTS_CONTROL.negate());
    /** Button labeled "top left" */
    public static Trigger BARGE = operator.button(16).and(DRIVER_WANTS_CONTROL.negate());

    public static Trigger CORAL_INTAKE = operator.button(11).and(DRIVER_WANTS_CONTROL.negate());
    public static Trigger CORAL_EXTAKE = operator.button(10).and(DRIVER_WANTS_CONTROL.negate());

    public static Trigger ALGAE_INTAKE = operator.button(4).and(DRIVER_WANTS_CONTROL.negate());
    public static Trigger ALGAE_EXTAKE = operator.button(5).and(DRIVER_WANTS_CONTROL.negate());

}
