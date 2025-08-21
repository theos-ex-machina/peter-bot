// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.arm.Arm;
import raidzero.robot.subsystems.arm.Intake;
import raidzero.robot.subsystems.drivetrain.Limelight;
import raidzero.robot.subsystems.drivetrain.Swerve;
import raidzero.robot.subsystems.drivetrain.TunerConstants;

public class RobotContainer {

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final Swerve swerve = Swerve.system();
    // public final Limelight limes = Limelight.system();
    // public final Arm arm = Arm.system();
    // public final Intake intake = Intake.system();

    public final SendableChooser<Command> autoChooser;

    public static boolean[][] reef = new boolean[16][4];

    /**
     * Constructs a {@link RobotContainer} instance
     */
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);
        PathfindingCommand.warmupCommand().schedule();

        Bindings.setDefaultCommands();
        Bindings.applyManualBindings();
        // Bindings.setButtonBinds();

        swerve.registerTelemetry(logger::telemeterize);
    }

    /**
     * Returns the selected autonomous command
     *
     * @return A {@link Command} representing the selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}