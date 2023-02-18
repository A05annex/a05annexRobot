package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.a05annex.frc.subsystems.DriveSubsystem;

/**
 *
 */
public class AbsoluteTranslateCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    private final double distanceForward;
    private final double distanceStrafe;
    /**
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
        driveSubsystem.startAbsoluteTranslate(distanceForward,distanceStrafe);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // nothing to do here, just waiting for the drive to get to the requested
        // position
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
        // TODO: Make this return true when this Command no longer needs to run execute()
        return driveSubsystem.isAbsoluteTranslateDone();
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // nothing to do here
    }
}
