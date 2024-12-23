package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;

/**
 * This is a command that follows an autonomous path created in the
 * <a href="https://github.com/A05annex/SwervePathPlanning">Swerve Path Planning</a> app. The key capabilities
 * of this command is that it acts as a dynamic command group that orchestrates the robot path and:
 * <ul>
 *     <li>launches other commands that happen concurrently with path following</li>
 *     <li>stops the robot on the path, initiates a command (like aiming and shooting), and continues following
 *     the path when the initiated command completes.</li>
 * </ul>
 * Note that when run in a test environment the navX device is instantiated as a simulation device rather than a
 * real physical devise.
 */
@SuppressWarnings("unused")
public class AutonomousPathCommand extends Command {


    /**
     * This is a wrapper for a {@link KochanekBartelsSpline.PathPoint} that provides mirroring around Y for the path.
     * It provides getter functions for the parameters in the {@link KochanekBartelsSpline.PathPoint} and performs
     * the mirroring functionality as required.
     */
    protected static class PathPoint {

        /**
         * This is the {@link KochanekBartelsSpline.PathPoint} returned by the
         * {@link KochanekBartelsSpline.PathFollower}.
         */
        final KochanekBartelsSpline.PathPoint pathPoint;
        /**
         * {@code true} if the path should be mirrored, {@code false} if the path should be run as specified.
         */
        final boolean mirror;

        /**
         * Instantiate a {@code PathPoint}.
         * @param pathPoint The actual {@link KochanekBartelsSpline.PathPoint} returned by the
         *              {@link KochanekBartelsSpline.PathFollower}.
         * @param mirror {@code true} if the path should be mirrored, {@code false} if the path should be run
         *                           as specified.
         */
        PathPoint(KochanekBartelsSpline.PathPoint pathPoint, boolean mirror) {
            this.pathPoint = pathPoint;
            this.mirror = mirror;
        }
        double speedForward() { return pathPoint.speedForward; }
        double speedStrafe() { return mirror ? -pathPoint.speedStrafe : pathPoint.speedStrafe; }
        double speedRotation() { return mirror ? -pathPoint.speedRotation : pathPoint.speedRotation; }
        AngleConstantD fieldHeading() {
            return mirror ?
                    new AngleConstantD(AngleUnit.RADIANS,-pathPoint.fieldHeading.getRadians()) :
                    pathPoint.fieldHeading;
        }

        KochanekBartelsSpline.RobotAction action() { return pathPoint.action; }
    }
    /**
     * The swerve drive
     */
    private final ISwerveDrive swerveDrive;
    /**
     * The autonomous path description set by the switches.
     */
    private final A05Constants.AutonomousPath path;
    /**
     * The spline of the path - which includes descriptions of all commands that should be run while the
     * autonomous oath is being followed.
     */
    private final KochanekBartelsSpline spline;
    /**
     * The {@link KochanekBartelsSpline.PathFollower} which accepts a time from start of the path and returns
     * a {@link KochanekBartelsSpline.PathPoint}.
     */
    private KochanekBartelsSpline.PathFollower pathFollower;
    /**
     * The current {@link AutonomousPathCommand.PathPoint} for this call of {@link #execute()}
     */
    protected PathPoint pathPoint = null;
    /**
     * The last {@link AutonomousPathCommand.PathPoint} for the last call of {@link #execute()}
     */
    protected PathPoint lastPathPoint = null;
    /**
     * Whether this command is finished. Because there are other commands that can be launched by this command, this
     * command does not finish until the end of the path is reached, and until those launched commands have lso finished.
     */
    private boolean isFinished = false;
    /**
     * The start time for the path.
     */
    private long startTime;
    /**
     * The start time for the current {@link #stopAndRunCommand}. When the current {@link #stopAndRunCommand}
     * finishes, this is used to compute the duration of that command, which is added to {@link #stopAndRunDuration}
     */
    private long stopAndRunStartTime = 0;
    /**
     * The current <i>stop-and-run-command</i>, {@code null} if there is no current <i>stop-and-run-command</i>.
     */
    private Command stopAndRunCommand = null;
    /**
     * The time consumed by the <i>stop and run</i> commands. The time on the path is the<br>
     * {@link System#currentTimeMillis()} - {@link #startTime} - {@code stopAndRunDuration}
     */
    protected long stopAndRunDuration = 0;

    /**
     * {@code false} if the path should be followed as specified, {@code true} if X should be mirrored
     * around the Y axis.
     */
    protected boolean mirror = false;

    /**
     * Constructor for the {@code AutonomousPathCommand}.
     * @param path The path description.
     * @param swerveDrive The swerve drive subsystem.
     * @param additionalRequirements Additional required subsystems.
     */
    public AutonomousPathCommand(@NotNull A05Constants.AutonomousPath path, @NotNull ISwerveDrive swerveDrive,
                                 Subsystem... additionalRequirements) {
        addRequirements(swerveDrive.getDriveSubsystem());
        addRequirements(additionalRequirements);
        this.swerveDrive = swerveDrive;
        this.path = path;
        spline = this.path.getSpline();
        if (A05Constants.getPrintDebug()) {
            System.out.println("AutonomousPathCommand instantiated for path " + path.getName());
        }
    }

    /**
     * This should be called after {@link AutonomousPathCommand} instantiation, and before the command is initialized.
     * Normally this would be in your {@code RobotContainer} constructor.
     * @param mirror {@code false} if the path should be followed as specified, {@code true} if X should be mirrored
     *                            around the Y axis.
     */
    public void setMirror(boolean mirror) {
        this.mirror = mirror;
    }

    private PathPoint getPointAt(double time) {
        return new PathPoint(pathFollower.getPointAt(time), mirror);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pathFollower = spline.getPathFollower();
        startTime = System.currentTimeMillis();
        isFinished = false;
        initializeRobotForPath();
        if (A05Constants.getPrintDebug()) {
            System.out.println("AutonomousPathCommand.initialize() called for path '" + path.getName() + "'");
        }
    }
    /**
     * Initialize the robot to run this path. This initialization consists specifically of
     * <ul>
     * <li>making sure the NavX is aware of robot heading prior to starting along the path</li>
     * <li>assuring The serve modules are rotated to the correct orientation for the first
     * expected Forward, strafe, and rotate components that will be set for the path (eliminating
     * drift while the robot is trying to get all the modules to the correct orientation.</li>
     * </ul>
     */
    public void initializeRobotForPath() {
        pathPoint = getPointAt(0.0);
        if (pathPoint.pathPoint != null) {
            NavX.getInstance().initializeHeadingAndNav(pathPoint.fieldHeading());
            double forward = pathPoint.speedForward() / swerveDrive.getMaxMetersPerSec();
            double strafe = pathPoint.speedStrafe() / swerveDrive.getMaxMetersPerSec();
            double rotation = (pathPoint.speedRotation() / swerveDrive.getMaxRadiansPerSec());
            swerveDrive.prepareForDriveComponents(forward, strafe, rotation);
            startTime = System.currentTimeMillis();
            if ((null != pathPoint.action()) && (null != pathPoint.action().command) &&
                    (KochanekBartelsSpline.RobotActionType.STOP_AND_RUN_COMMAND == pathPoint.action().actionType)) {
                if (null != (stopAndRunCommand = instantiateActionCommand(pathPoint.action().command))) {
                    stopAndRunStartTime = System.currentTimeMillis();
                    stopAndRunCommand.initialize();
                }
            }
            lastPathPoint = pathPoint;
        }
        if (A05Constants.getPrintDebug()) {
            System.out.println("AutonomousPathCommand.initializeRobotForPath() called for path '" +
                    path.getName() + "'");
        }
    }

    /**
     * Instantiate the action command.
     * @param commandClassName The command class name, assumed to be in the {@code frc.robot.commands}
     *                         package, and has a no argument constructor.
     * @return Returns the instantiated command, or {@code null} if the command could not be instantiated.
     */
    private Command instantiateActionCommand(@NotNull String commandClassName) {
        String commandClass = "frc.robot.commands." + commandClassName;
        Object obj;
        Command command = null;
        try {
            obj = Class.forName(commandClass).getDeclaredConstructor().newInstance();
            if (obj instanceof Command) {
                command = (Command)obj;
            } else {
                System.out.printf("Class '%s' is not a command; continuing with path.%n", commandClass);

            }
        } catch (final Exception t) {
            System.out.printf("Could not instantiate command: class='%s'; continuing with path.%n",
                    commandClass);
        }
        return command;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (null != stopAndRunCommand) {
            // There is an active stop-and-run command. Take the next step in that command.
            stopAndRunCommand.execute();

        } else {
            // get the path time: path time is a time along the path as though there were no stop-and-run
            // commands. The duration of any stop-and-run commands is tracked and subtracted to get the
            // actual path time.
            double pathTime = (System.currentTimeMillis() - startTime - stopAndRunDuration) / 1000.0;
            pathPoint = getPointAt(pathTime);
            if (A05Constants.getPrintDebug()) {
                System.out.println("AutonomousPathCommand.execute() get point at time: " + pathTime);
            }
            if (pathPoint.pathPoint == null) {
                // We have reached the end of the path, stop the robot and finish this command.
                isFinished = true;
                swerveDrive.swerveDriveComponents(0.0, 0.0, 0.0);
            } else {
                // for 2022 Rapid React we have added scheduled actions and stop-and-run actions. This makes this
                // command very much like a wpilib CommandGroup action. The interesting thing about this action
                // that it gets all its sequencing from the path file - which was built without access to the
                // actual code and commands that may be scheduled or stop_and_run. These commands are instantiated
                // by reflection, so only the name of the command is required during path planning.
                Command command;
                if ((null != pathPoint.action()) && (null != pathPoint.action().command) &&
                        (null != (command = instantiateActionCommand(pathPoint.action().command)))) {
                    // OK, we've instantiated the command, now either schedule it, or run it inside this command.
                    if (KochanekBartelsSpline.RobotActionType.SCHEDULE_COMMAND == pathPoint.action().actionType) {
                        // this one is really simple - we just schedule the command, and it happens in
                        // parallel with path following.
                        CommandScheduler.getInstance().schedule(command);
                    } else if (KochanekBartelsSpline.RobotActionType.STOP_AND_RUN_COMMAND == pathPoint.action().actionType) {
                        // this is a bit more complicated, we are going to run the command inside this command,
                        // then resume path following when this command completes. So we assume the robot is stopped,
                        //that we know the start time of the command, and that the command is initialized.
                        stopAndRunCommand = command;
                        swerveDrive.swerveDriveComponents(0.0, 0.0, 0.0);
                        stopAndRunStartTime = System.currentTimeMillis();
                        stopAndRunCommand.initialize();
                        return;
                    }
                }

                double forward = pathPoint.speedForward() / swerveDrive.getMaxMetersPerSec();
                double strafe = pathPoint.speedStrafe() / swerveDrive.getMaxMetersPerSec();
                // The expected heading is included in the PathPoint. The path point is the instantaneous
                // speed and position that we want to be at when we go through the path point. So, we are
                // actually telling the swerve drive what to do to get from this path point to the next
                // path point. If the heading is not correct for this path point, then forward and strafe
                // speeds are not in the right direction to get to the next path point.
                //
                // So here we have a heading PID error correction to try and keep us on path. The error is:
                //     expected heading (pathPoint.fieldHeading()) -
                //         actual robot heading (NavX.getInstance().getHeading())
                // and we would like to correct this in several command cycles without introducing oscillation.
                // 1 cycle time is 20ms, or .02sec -- or initial guess was 3 command cycles for correction, but
                // that resulted in rotation oscillations typical of too high Kp in a PID loop, so we adjusted
                // the guess targeting for a 12 cycle correction, so
                //     error(radians) / (12 * .02sec) = radians/sec adjustment to the path rotation to
                // correct the error.
                double headingError = (pathPoint.fieldHeading().getRadians() -
                        NavX.getInstance().getHeading().getRadians());
                NavX.getInstance().setExpectedHeading(pathPoint.fieldHeading());
//                double headingCorrection = headingError / (12.0 * 0.02);
                double headingCorrection = headingError * A05Constants.getDriveOrientationKp();
                double rotation = Utl.clip((pathPoint.speedRotation() / swerveDrive.getMaxRadiansPerSec()) + headingCorrection, -1.0, 1.0);
                swerveDrive.swerveDriveComponents(forward, strafe, rotation);

                lastPathPoint = pathPoint;
            }
        }

    }


    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        if (null != stopAndRunCommand) {
            if (stopAndRunCommand.isFinished()) {
                // done with the stop and run, so end it and increment the stop and run duration.
                stopAndRunCommand.end(false);
                long now = System.currentTimeMillis();
                long duration = now - stopAndRunStartTime;
                stopAndRunDuration += duration;
                stopAndRunCommand = null;
                stopAndRunStartTime = 0;
                // I'm going to assume that if we stop to do something it may involve rotation to aim
                // for shooting, but, probably does not involve any translation.
                swerveDrive.setHeading(pathPoint.fieldHeading());
                try {
                    Thread.sleep(15);
                    double forward = pathPoint.speedForward() / swerveDrive.getMaxMetersPerSec();
                    double strafe = pathPoint.speedStrafe() / swerveDrive.getMaxMetersPerSec();
                    double rotation = (pathPoint.speedRotation() / swerveDrive.getMaxRadiansPerSec());
                    swerveDrive.prepareForDriveComponents(forward, strafe, rotation);
                } catch (InterruptedException e) {
                    // do nothing here, it means the sleep was interrupted.
                }
            }
        }
        if (A05Constants.getPrintDebug() && isFinished) {
            System.out.println("AutonomousPathCommand.isFinished() returns true for path " + path.getName());
        }
        return isFinished;
    }

    /**
     * The action to take when the command ends - in this case,  if there is a stop-and-run command active,
     * we end it, and then we stop the robot drive.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted && (null != stopAndRunCommand)) {
            stopAndRunCommand.end(true);
        }
        swerveDrive.swerveDriveComponents(0, 0, 0);
        if (A05Constants.getPrintDebug()) {
            System.out.println("AutonomousPathCommand.end() called for path '" + path.getName() + "'");
        }
    }
}
