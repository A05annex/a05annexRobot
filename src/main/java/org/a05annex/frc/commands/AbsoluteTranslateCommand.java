package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.a05annex.frc.subsystems.DriveSubsystem;

/**
 * This is a command that does an absolute translate with respect to the robot, no change in heading. It can
 * be used to position the robot relative to its final position after centering on a target like an
 * <i>april tag</i>. The absolute translation is expressed as meters forward and left strafe of the and
 * is set in the constructor.
 */
public class AbsoluteTranslateCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final double distanceForward;
    private final double distanceStrafe;
    /**
     * Construct a command to move the robot the specified forward and strafe distances.
     *
     * @param distanceForward The distance to move forward (negative is backwards) in meters.
     * @param distanceStrafe The distance to move right (negative is left) in meters.
     */
    public AbsoluteTranslateCommand(double distanceForward, double distanceStrafe) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
        this.distanceForward = distanceForward;
        this.distanceStrafe = distanceStrafe;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // This command sets the heading and target end position of all the modules.
        driveSubsystem.startAbsoluteTranslate(distanceForward,distanceStrafe);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // nothing to do here, just waiting for the drive to get to the requested
        // position in the isFinished() command.
    }

    /**
     * <p>
     * Test whether the robot has reached to specified end location, which finishes this command.
     * </p>
     *
     * @return {@code true} when the robot reaches the specified position, {@code false} otherwise.
     */
    @Override
    public boolean isFinished() {
        // test whether we have reached the final position
        return driveSubsystem.isAbsoluteTranslateDone();
    }

    /**
     * Called to end the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // nothing to do here - the command is ending, it will release control of the
        // drive subsystem, and the default drive command will probably take over.
    }
}
