package raidzero.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import raidzero.robot.subsystems.arm.Arm;
import raidzero.robot.subsystems.arm.ArmConstants.Positions;
import raidzero.robot.subsystems.arm.Intake;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.drivetrain.TunerConstants;

import static raidzero.robot.subsystems.Superstructure.*;

public class Bindings {
    private static final Swerve swerve = Swerve.system();
    private static final Arm arm = Arm.system();
    private static final Intake intake = Intake.system();

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static void applyButtonBinds() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        DRIVER_WANTS_CONTROL.whileTrue(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        arm.setDefaultCommand(arm.home());
        intake.setDefaultCommand(intake.stop());

        L1.onTrue(arm.moveTo(Positions.L1, Positions.L1_WRIST_ANGLE));
        L2.onTrue(arm.moveTo(Positions.L2, Positions.L2_WRIST_ANGLE));
        L3.onTrue(arm.moveTo(Positions.L3, Positions.L3_WRIST_ANGLE));
        L4.onTrue(arm.interpolateTo(Positions.L4_INTERPOLATION_PATH, Positions.L4_WRIST_ANGLE));

        STATION.whileTrue(arm.moveTo(Positions.STATION, Positions.STATION_WRIST_ANGLE));
        GROUND_INTAKE.whileTrue(arm.moveTo(Positions.GROUND_INTAKE, Positions.GROUND_INTAKE_WRIST_ANGLE));

        PROCESSOR.whileTrue(arm.moveTo(Positions.PROCESSOR, Positions.PROCESSOR_WRIST_ANGLE));
        BARGE.whileTrue(arm.moveTo(Positions.BARGE, Positions.BARGE_WRIST_ANGLE));

        CORAL_INTAKE.whileTrue(intake.intakeCoral());
        CORAL_EXTAKE.whileTrue(intake.extakeCoral());

        ALGAE_INTAKE.whileTrue(intake.intakeAlgae());
        ALGAE_EXTAKE.whileTrue(intake.extakeAlgae());

        L3_ALGAE.whileTrue(arm.moveTo(Positions.L3_ALGAE, Positions.L3_ALGAE_WRIST_ANGLE));
        L2_ALGAE.whileTrue(arm.moveTo(Positions.L2_ALGAE, Positions.L2_ALGAE_WRIST_ANGLE));

        // auto home when let go or obtain algae or coral
        intake.hasCoral().onTrue(arm.home());
        intake.hasAlgae().onTrue(arm.home());

        intake.hasCoral().onFalse(arm.home());
        intake.hasAlgae().onFalse(arm.home());

        L4.and(DRIVER_WANTS_CONTROL.negate()).and(READY_FOR_REEF)
            .whileTrue(swerve.goToNearestReef(4));

        L4.and(DRIVER_WANTS_CONTROL.negate()).and(AT_REEF_READY_ARM)
            .whileTrue(arm.interpolateTo(Positions.L4_INTERPOLATION_PATH, Positions.L4_WRIST_ANGLE));

        L4.and(DRIVER_WANTS_CONTROL.negate()).and(AT_REEF_READY_ROLLERS)
            .whileTrue(intake.extakeCoral());

        STATION.and(DRIVER_WANTS_CONTROL.negate()).and(WANTS_GAMEPEICE)
            .whileTrue(swerve.goToNearestStation());

        STATION.and(DRIVER_WANTS_CONTROL.negate()).and(AT_STATION_READY_ARM)
            .whileTrue(arm.moveTo(Positions.STATION, Positions.STATION_WRIST_ANGLE));

        STATION.and(DRIVER_WANTS_CONTROL.negate()).and(AT_STATION_READY_INTAKE)
            .whileTrue(intake.intakeCoral());

        // allow driver movment for ground intake
        GROUND_INTAKE.and(WANTS_GAMEPEICE)
            .whileTrue(arm.moveTo(Positions.GROUND_INTAKE, Positions.GROUND_INTAKE_WRIST_ANGLE));

        // TODO: Add autopath logic to ground intake
        GROUND_INTAKE.and(AT_GROUND_READY_INTAKE)
            .whileTrue(intake.intakeCoral());

        L2_ALGAE.and(DRIVER_WANTS_CONTROL.negate()).and(READY_FOR_REEF)
            .whileTrue(swerve.goToNearestL2Algae());

        L2_ALGAE.and(DRIVER_WANTS_CONTROL.negate()).and(AT_REEF_READY_ARM)
            .whileTrue(arm.moveTo(Positions.L2_ALGAE, Positions.L2_ALGAE_WRIST_ANGLE));

        L2_ALGAE.and(DRIVER_WANTS_CONTROL.negate()).and(AT_REEF_READY_ROLLERS)
            .whileTrue(intake.intakeAlgae());

        PROCESSOR.and(DRIVER_WANTS_CONTROL.negate()).and(HAS_ALGAE)
            .whileTrue(swerve.goToProcessor());

        PROCESSOR.and(DRIVER_WANTS_CONTROL.negate()).and(AT_PROCESSOR_READY_ARM)
            .whileTrue(arm.moveTo(Positions.PROCESSOR, Positions.PROCESSOR_WRIST_ANGLE));

        PROCESSOR.and(DRIVER_WANTS_CONTROL.negate()).and(AT_PROCESSOR_READY_ROLLERS)
            .whileTrue(intake.extakeAlgae());

    }

    public static final int DRIVER_PORT = 0;
    public static CommandXboxController driver = new CommandXboxController(0);

    /** If the driver pulls any of the sticks */
    public static Trigger DRIVER_WANTS_CONTROL = driver.axisGreaterThan(0, 0.1)
        .or(driver.axisGreaterThan(1, 0.1))
        .or(driver.axisGreaterThan(4, 0.1))
        .or(driver.axisGreaterThan(5, 0.1));

    public static final int OPERATOR_PORT = 1;
    public static CommandGenericHID operator = new CommandGenericHID(OPERATOR_PORT);

    public static Trigger L1 = operator.button(6);
    public static Trigger L2 = operator.button(7);
    public static Trigger L3 = operator.button(8);
    public static Trigger L4 = operator.button(9);

    public static Trigger L3_ALGAE = operator.axisGreaterThan(0, 0.5);
    public static Trigger L2_ALGAE = operator.axisGreaterThan(1, 0.5);

    /** Button labeled "home" */
    public static Trigger STATION = operator.button(12);
    /** Button labeled "bottom right" */
    public static Trigger GROUND_INTAKE = operator.button(14);

    /** Button labeled "bottom left" */
    public static Trigger PROCESSOR = operator.button(15);
    /** Button labeled "top left" */
    public static Trigger BARGE = operator.button(16);

    public static Trigger CORAL_INTAKE = operator.button(11);
    public static Trigger CORAL_EXTAKE = operator.button(10);

    public static Trigger ALGAE_INTAKE = operator.button(4);
    public static Trigger ALGAE_EXTAKE = operator.button(5);

}
