// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidzero.robot.Constants.Arm.Positions;
import raidzero.robot.subsystems.arm.Arm;
import raidzero.robot.subsystems.arm.Intake;
import raidzero.robot.subsystems.drivetrain.Limelight;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.drivetrain.TunerConstants;
import raidzero.robot.Constants.Swerve.Setpoints;

import static raidzero.robot.Constants.Bindings.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Swerve swerve = Swerve.system();
    public final Limelight limes = Limelight.system();

    public final Arm arm = Arm.system();
    public final Intake intake = Intake.system();

    public final SendableChooser<Command> autoChooser;

    /**
     * Constructs a {@link RobotContainer} instance
     */
    public RobotContainer() {
        registerPathplannerCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);

        configureBindings();
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Configures button bindings for the robot
     */
    private void configureBindings() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
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

        // has coral but not there yet
        L4.and(intake.hasCoral()).and(swerve.atReef().negate())
            .whileTrue(swerve.goToPose(Setpoints.RIGHT_REEF_WAYPOINTS.get(0)));

        // has coral and there but no arm
        L4.and(intake.hasCoral()).and(swerve.atReef()).and(arm.atHome().negate())
            .whileTrue(arm.interpolateTo(Positions.L4_INTERPOLATION_PATH, Positions.L4_WRIST_ANGLE));

        // has coral, there and arm
        L4.and(intake.hasCoral()).and(swerve.atReef()).and(arm.atL4())
            .whileTrue(intake.extakeCoral());

        // no coral but wants some
        STATION.and(intake.hasCoral().negate()).and(swerve.atStation().negate())
            .whileTrue(swerve.goToPose(Setpoints.STATION_WAYPOINTS.get(0)));
        
        // there but no arm
        STATION.and(intake.hasCoral().negate()).and(swerve.atStation()).and(arm.atStation().negate())
            .whileTrue(arm.moveTo(Positions.STATION, Positions.STATION_WRIST_ANGLE));
        
        // has arm
        STATION.and(intake.hasCoral().negate()).and(swerve.atStation()).and(arm.atStation())
            .whileTrue(intake.intakeCoral());

        swerve.registerTelemetry(logger::telemeterize);
    }

    /**
     * Registers PathPlanner commands
     */
    private void registerPathplannerCommands() {}

    /**
     * Returns the selected autonomous command
     *
     * @return A {@link Command} representing the selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}