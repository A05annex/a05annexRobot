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
import org.a05annex.util.geo2d.KochanekBartelsSpline.*;
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

    // --------------------------------------------------------------------------------------------
    // These are some static parameters set during the run of the command, and examined only for
    // testing, to assure the command has operated as expected.
    public static int invalidCommandCt = 0;

    static public void zeroCounts() {
        invalidCommandCt = 0;
    }
    // --------------------------------------------------------------------------------------------



    /**
     * This is a wrapper for a {@link PathPoint} that provides mirroring around Y for the path.
     * It provides getter functions for the parameters in the {@link PathPoint} and performs
     * the mirroring functionality as required.
     */
    protected static class LclPathPoint {

        /**
         * This is the {@link PathPoint} returned by the
         * {@link PathFollower}.
         */
        final PathPoint pathPoint;
        /**
         * {@code true} if the path should be mirrored, {@code false} if the path should be run as specified.
         */
        final boolean mirror;

        /**
         * Instantiate a {@code PathPoint}.
         * @param pathPoint The actual {@link PathPoint} returned by the
         *              {@link PathFollower}.
         * @param mirror {@code true} if the path should be mirrored, {@code false} if the path should be run
         *                           as specified.
         */
        LclPathPoint(PathPoint pathPoint, boolean mirror) {
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

        RobotAction action() { return pathPoint.action; }

        ControlPoint nextControlPt() { return pathPoint.nextControlPoint; }
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
     * The {@link PathFollower} which accepts a time from start of the path and returns
     * a {@link PathPoint}.
     */
    private PathFollower pathFollower;
    /**
     * The current {@link LclPathPoint} for this call of {@link #execute()}
     */
    protected LclPathPoint pathPoint = null;
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
     * The current {@link RobotActionType#STOP_AND_RUN_COMMAND}, {@code null} if there is no current <i>stop-and-run-command</i>.
     */
    private Command stopAndRunCommand = null;
    /**
     * The start time for the current {@link RobotActionType#STOP_AND_RUN_COMMAND}. When the current
     * {@link RobotActionType#STOP_AND_RUN_COMMAND} finishes, this is used to compute the duration of that
     * command, which is added to {@link #accumulatedStopDuration}
     */
    private long stopAndRunStartTime = 0;

    /**
     * The current {@link RobotActionType#RELINQUISH_DRIVE_TO_COMMAND}, {@code null} if there is no
     * current <i>relinquish-drive-to</i> command.
     */
    private Command takeDriveCommand = null;
    /**
     * Has the current {@link #takeDriveCommand} taken control of the drive.
     */
    private boolean takeDriveCmdHasDriveControl = false;
    private long takeDriveCmdStartTime = 0;

    /**
     * The path time the path following should restart after {@link RobotActionType#RELINQUISH_DRIVE_TO_COMMAND}
     * ends. if it does not take control of the drive (the robot should be in at the curve control point, which
     * is the expected location at the end of targeting), and path following should resume.
     */
    private double takeDriveCmdDefEndPathTime = 0.0;

    /**
     * The accumulated time that the robot has not been running the path because it is running other commands that
     * take control of the drive for some period, and the robot must wait before
     * path following resumes. The time on the path is the<br>
     * ({@link System#currentTimeMillis()} - {@link #startTime} - {@code accumulatedStopDuration}) / 1000.0
     */
    protected long accumulatedStopDuration = 0;

    /**
     * {@code false} if the path should be followed as specified, {@code true} if X should be mirrored
     * around the Y axis. This is used when the red and black fields are mirrors of reach other rather
     * that a 180&deg; rotation around field center.
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

    private LclPathPoint getPointAt(double time) {
        return new LclPathPoint(pathFollower.getPointAt(time), mirror);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pathFollower = spline.getPathFollower();
        startTime = System.currentTimeMillis();
        isFinished = false;
        if (A05Constants.getPrintDebug()) {
            System.out.println("**************************************************************************************");
            System.out.println("**** AutonomousPathCommand.initialize() called for path '" + path.getName() + "'");
            System.out.println("****   swerveDrive: " + swerveDrive.getClass().getCanonicalName());
        }
        initializeRobotForPath();
        if (A05Constants.getPrintDebug()) {
            System.out.println("**************************************************************************************");
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
        if (A05Constants.getPrintDebug()) {
            System.out.println("****   AutonomousPathCommand.initializeRobotForPath() called for path '" +
                    path.getName() + "'");
        }
        pathPoint = getPointAt(0.0);
        if (pathPoint.pathPoint != null) {
            NavX.getInstance().initializeHeadingAndNav(pathPoint.fieldHeading());
            double forward = pathPoint.speedForward() / swerveDrive.getMaxMetersPerSec();
            double strafe = pathPoint.speedStrafe() / swerveDrive.getMaxMetersPerSec();
            double rotation = (pathPoint.speedRotation() / swerveDrive.getMaxRadiansPerSec());
            swerveDrive.prepareForDriveComponents(forward, strafe, rotation);
            startTime = System.currentTimeMillis();
            RobotAction robotAction = pathPoint.action();
            if ((null != robotAction) && (RobotActionType.STOP_AND_RUN_COMMAND == robotAction.actionType)) {
                Command command;
                if (null != (command = instantiateActionCommand(robotAction))) {
                    stopAndRunCommand = command;
                    stopAndRunStartTime = System.currentTimeMillis();
                    stopAndRunCommand.initialize();
                }
            }
        }
        if (A05Constants.getPrintDebug()) {
            System.out.println("****   Field heading: " + pathPoint.fieldHeading().getDegrees() + "degrees");
        }
    }

    /**
     * Instantiate the action command.
     * @param robotAction The {@link RobotAction} description for the action to be performed. The command assumed to
     *                    be in the {@code frc.robot.commands} package, and has a constructor with an argument
     *                    constructor signature that matches that specified in the description.
     * @return Returns the instantiated command, or {@code null} if the command could not be instantiated.
     */
    private Command instantiateActionCommand(@NotNull RobotAction robotAction) {
        String commandClassName = robotAction.getCommand();
        if (null == commandClassName) {
            return null;
        }
        String commandClass = "frc.robot.commands." + commandClassName;
        if (A05Constants.getPrintDebug()) {
            System.out.println("**************************************************************************************");
            System.out.println("**** Attempting to instantiate robot action: " + commandClass);
            System.out.println("****   Robot action type: " + robotAction.actionType.toString());
        }
        Command command = Utl.instantiateObjectFromName(Command.class, commandClass,
                robotAction.getArgTypeArray(), robotAction.getArgValueArray());
        if (null != command) {
            if (RobotActionType.RELINQUISH_DRIVE_TO_COMMAND == robotAction.actionType) {
                if (!(command instanceof ICanTakeDrive)) {
                    System.out.println("**** ************************************************************************");
                    System.out.println("**** **** Command '" + commandClass);
                    System.out.println("**** ****   cannot be run because it does not implement ICanTakeDrive");
                    System.out.println("**** ************************************************************************");
                    command = null;
                }
            }
        }
        if (null == command) {
            invalidCommandCt++;
        }
        if (A05Constants.getPrintDebug()) {
            System.out.println("****   Instantiation " + ((null == command)?"failed":"successful"));
            System.out.println("**************************************************************************************");
        }
        return command;
    }

    /**
     *
     * @param interrupted
     */
    private void stopTakesDriveCommand(boolean interrupted) {
        if (A05Constants.getPrintDebug()) {
            System.out.println("**************************************************************************************");
            System.out.println("**** AutonomousPathCommand.stopTakesDriveCommand():");
            System.out.println("****   command: " + takeDriveCommand.getClass().getCanonicalName());
            System.out.println("****   " + (interrupted?"The command is being forced to end.":
                    "The command has reported it is finished."));
            System.out.println("****   " + (takeDriveCmdHasDriveControl?"The command has taken drive control":
                    "THIS COMMAND NEVER TOOK CONTROL OF THE DRIVE"));
            System.out.println("**************************************************************************************");
        }
        takeDriveCommand.end(interrupted);
        long now = System.currentTimeMillis();
//        accumulatedStopDuration += now - startTime - (long)(1000.0 * takeDriveCmdDefEndPathTime);
        accumulatedStopDuration = now - startTime - (long)(1000.0 * takeDriveCmdDefEndPathTime);
        // now set everything back to defaults for takesDriveAction
        takeDriveCommand = null;
        takeDriveCmdHasDriveControl = false;
        takeDriveCmdStartTime = 0;
        takeDriveCmdDefEndPathTime = 0.0;
        // assume targeting has the robot at the next control point location and
        // heading, so there is no more to do here.
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
            // actual path time. OK, we are doing this even if there is a takesDrive command because there
            // may be another scheduled action that needs to be queued, or pre emps te
            double pathTime = (System.currentTimeMillis() - startTime - accumulatedStopDuration) / 1000.0;
            if ((null != takeDriveCommand) && !takeDriveCmdHasDriveControl &&
                    (pathTime >= takeDriveCmdDefEndPathTime)) {
                // This is a worst-case scenario - the robot has driven to the control point after targeting started,
                // but the target has not been acquired. Force an end to the targeting command.
                stopTakesDriveCommand(true);
            }
            // if a command is potentially taking control of the drive, test whether it is ready.
            if ((null != takeDriveCommand) && !takeDriveCmdHasDriveControl) {
                takeDriveCmdHasDriveControl = ((ICanTakeDrive)takeDriveCommand).canTakeDrive();
                if (A05Constants.getPrintDebug() && takeDriveCmdHasDriveControl) {
                    System.out.println("**************************************************************************************");
                    System.out.println("**** AutonomousPathCommand.execute():");
                    System.out.println("****   command: " + takeDriveCommand.getClass().getCanonicalName());
                    System.out.println("****   This command has just TAKEN CONTROL of the drive");
                    System.out.println("**************************************************************************************");
                }
            }
            if (takeDriveCmdHasDriveControl && (pathTime >= takeDriveCmdDefEndPathTime)) {
                // The takeDriveCommand is still driving. We have run the path follower
                // to the next control point, and do not want to go any further in path
                // following, so we are just waiting for targeting to complete now.
                takeDriveCommand.execute();
                return;
            }

            // NOTE: we may be targeting, but we still need to path follow up to the control
            // point so that we pick up any scheduled commands that may be started during targeting.
            pathPoint = getPointAt(pathTime);
            if (A05Constants.getPrintDebug()) {
                System.out.println("AutonomousPathCommand.execute() get point at time: " + pathTime);
            }
            if (pathPoint.pathPoint == null) {
                // We have reached the end of the path, stop the robot and finish this command.
                isFinished = true;
                swerveDrive.swerveDriveComponents(0.0, 0.0, 0.0);
            } else {
                // for 2022 Rapid React, we have added scheduled actions and stop-and-run actions. This makes this
                // command very much like a wpilib CommandGroup action. The interesting thing about this action
                // that it gets all its sequencing from the path file - which was built without access to the
                // actual code and commands that may be scheduled or stop_and_run. These commands are instantiated
                // by reflection, so only the name of the command and the argument type/value list is required during
                // path planning.
                //
                // for 2025 REEFSCAPE, we have added scheduled actions that can take control of the swerve drive
                // for targeting. This is a bit more complicated because target acquisition can be significantly
                // affected by game venue conditions. So we need an implementation that will do something reasonable
                // if the target cannot be acquired, and provides leeway in when the target is acquired. These commands
                // must implement ICanTakeDrive.
                RobotAction robotAction = pathPoint.action();
                if (null != robotAction) {
                    Command command;
                    if (null != (command = instantiateActionCommand(robotAction))) {
                        // OK, we've instantiated the command, now either schedule it, or run it inside this command.
                        if (RobotActionType.SCHEDULE_COMMAND == pathPoint.action().actionType) {
                            // this one is really simple - we just schedule the command, and it happens in
                            // parallel with path following.
                            CommandScheduler.getInstance().schedule(command);
                        } else if (RobotActionType.STOP_AND_RUN_COMMAND == pathPoint.action().actionType) {
                            // we are concerned about the increment interval creating a situation
                            // this is a bit more complicated, we are going to run the command inside this command,
                            // then resume path following when this command completes. So we assume the robot is stopped,
                            //that we know the start time of the command, and that the command is initialized.
                            stopAndRunCommand = command;
                            swerveDrive.swerveDriveComponents(0.0, 0.0, 0.0);
                            stopAndRunStartTime = System.currentTimeMillis();
                            stopAndRunCommand.initialize();
                            // not path following again until this finishes.
                            return;
                        } else if (RobotActionType.RELINQUISH_DRIVE_TO_COMMAND == pathPoint.action().actionType) {
                            takeDriveCommand = command;
                            takeDriveCmdHasDriveControl = false;
                            takeDriveCmdStartTime = System.currentTimeMillis();
                            // set this just short of the control point so we get the control point and possibly
                            // stop and run command when path following restarts.
                            takeDriveCmdDefEndPathTime = pathPoint.nextControlPt().getTime();
                            takeDriveCommand.initialize();
                        }
                    }
                }

                if (takeDriveCmdHasDriveControl) {
                    // there is a targeting command that has drive control, run it.
                    takeDriveCommand.execute();
                } else {
                    // normal path following
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
//                    double headingCorrection = headingError / (12.0 * 0.02);
                    double headingCorrection = headingError * A05Constants.getDriveOrientationKp();
                    double rotation = Utl.clip((pathPoint.speedRotation() / swerveDrive.getMaxRadiansPerSec()) + headingCorrection, -1.0, 1.0);
                    swerveDrive.swerveDriveComponents(forward, strafe, rotation);
                }
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
                accumulatedStopDuration += duration;
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
        } else if ((null != takeDriveCommand) && takeDriveCmdHasDriveControl) {
            if (takeDriveCommand.isFinished()) {
                stopTakesDriveCommand(false);
            }
        }
        if (A05Constants.getPrintDebug() && isFinished) {
            if (A05Constants.getPrintDebug()) {
                System.out.println("**************************************************************************************");
                System.out.println("**** AutonomousPathCommand.isFinished() returns true for path '" + path.getName() + "'");
                System.out.println("**************************************************************************************");
            }
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
        if (null != stopAndRunCommand) {
            stopAndRunCommand.end(interrupted);
        } else if (null != takeDriveCommand) {
            takeDriveCommand.end(interrupted);
        }
        swerveDrive.swerveDriveComponents(0, 0, 0);
        if (A05Constants.getPrintDebug()) {
            System.out.println("**************************************************************************************");
            System.out.println("**** AutonomousPathCommand.end() called for path '" + path.getName() + "'");
            System.out.println("****   " + (interrupted?"The command is being forced to end.":
                    "The command has reported it is finished."));
            System.out.println("**************************************************************************************");
        }
    }
}
