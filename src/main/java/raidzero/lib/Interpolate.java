package raidzero.lib;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Interpolate extends Command {
    private final List<Pose2d> path;
    private final double segmentTime;
    private Consumer<Pose2d> poseConsumer;

    private final Timer timer = new Timer();

    private int currentSegment = 0;
    private double segmentStartTime = 0.0;

    public Interpolate(List<Pose2d> path, double segmentTimeSeconds, Consumer<Pose2d> poseConsumer, Subsystem... systems) {
        this.path = path;
        this.segmentTime = segmentTimeSeconds;
        this.poseConsumer = poseConsumer;

        super.addRequirements(systems);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        currentSegment = 0;
        segmentStartTime = 0.0;
    }

    @Override
    public void execute() {
        if (currentSegment >= path.size() - 1)
            return;

        Pose2d start = path.get(currentSegment);
        Pose2d end = path.get(currentSegment + 1);

        double elapsed = timer.get() - segmentStartTime;
        double t = Math.min(elapsed / segmentTime, 1.0);

        double x = interpolate(start.getX(), end.getX(), t);
        double y = interpolate(start.getY(), end.getY(), t);

        Pose2d targetPose = new Pose2d(Meters.of(x), Meters.of(y), Rotation2d.kZero);

        poseConsumer.accept(targetPose);

        if (t >= 1.0) {
            currentSegment++;
            segmentStartTime = timer.get();
        }
    }

    @Override
    public boolean isFinished() {
        return currentSegment >= path.size() - 1;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        Pose2d finalPose = path.get(path.size() - 1);
        poseConsumer.accept(finalPose);
    }

    private double interpolate(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
