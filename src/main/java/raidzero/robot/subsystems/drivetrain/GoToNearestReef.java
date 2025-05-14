package raidzero.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.Constants.Swerve.Setpoints;
import raidzero.robot.RobotContainer;

/**
 * Checks goes to the nearest reef that does not have a coral at the specified level
 */
public class GoToNearestReef extends Command {
    private Pose2d nearestPose;
    private int level;

    private Swerve swerve;

    public GoToNearestReef(int level, Swerve swerve) {
        this.level = level;
        this.swerve = swerve;
        this.addRequirements(swerve);
    }

    @Override
    public void initialize() {
        List<Pose2d> avalibleReefs = new ArrayList<Pose2d>();

        for (int i = 0; i < Setpoints.ALL_REEFS.size(); i++) {
            if (RobotContainer.reef[i][level - 1] == false) {
                avalibleReefs.add(Setpoints.ALL_REEFS.get(i));
            }
        }

        nearestPose = swerve.getState().Pose.nearest(avalibleReefs);
    }

    @Override
    public void execute() {
        swerve.goToPose(nearestPose);
    }
}
