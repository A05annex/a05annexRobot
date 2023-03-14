package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleConstantD;

/**
 * This is a command that does an absolute translate with respect to the robot, no change in heading. It can
 * be used to position the robot relative to its final position after centering on a target like an
 * <i>april tag</i>. The absolute translation is expressed as meters forward and left strafe of the and
 * is set in the constructor.
 */
public class AbsoluteSmartTranslateCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final double distanceForward;
    private final double distanceStrafe;
    private final double maxSpeed;
    private final double maxAcceleration;
    private final AngleConstantD expectedHeading;
    private boolean restoreHeadingAtEnd = true;

    /**
     * Construct a command to move the robot the specified forward and strafe distances.
     *
     * @param distanceForward The distance to move forward (negative is backwards) in meters.
     * @param distanceStrafe The distance to move right (negative is left) in meters.
     */
    public AbsoluteSmartTranslateCommand(double distanceForward, double distanceStrafe) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
        this.distanceForward = distanceForward;
        this.distanceStrafe = distanceStrafe;
        this.maxSpeed = 1.0;
        this.maxAcceleration = 10000.0;
        this.expectedHeading = NavX.getInstance().getHeadingInfo().expectedHeading;
    }

    /**
     * Construct a command to move the robot the specified forward and strafe distances.
     *
     * @param distanceForward The distance to move forward (negative is backwards) in meters.
     * @param distanceStrafe  The distance to move right (negative is left) in meters.
     * @param maxSpeed        (double) This is the speed mapped to the range 0.0 to 1.0.
     * @param maxAcceleration (double) This is a REV Spark smart motion max RPM/sec -- so, 10000 gets the
     *                        robot to max speed in about .5 seconds.
     * @param restoreHeading  (boolean) {@code true} if the heading should be restored after the translation,
     *                        {@code false} if not. Note, if heading is restored, the wheels will be in
     *                        position to spin the robot when the command ends; otherwise the wheels will be positioned
     */
    public AbsoluteSmartTranslateCommand(double distanceForward, double distanceStrafe,
                                         double maxSpeed, double maxAcceleration, boolean restoreHeading) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
        this.distanceForward = distanceForward;
        this.distanceStrafe = distanceStrafe;
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.expectedHeading = NavX.getInstance().getHeadingInfo().expectedHeading;
        this.restoreHeadingAtEnd = restoreHeading;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // This command sets the heading and target end position of all the modules.
        driveSubsystem.startAbsoluteSmartTranslate(distanceForward,distanceStrafe,
                maxSpeed, maxAcceleration);
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
        // Fix the heading if there was skidding that turned the robot.
        if (restoreHeadingAtEnd) {
            driveSubsystem.setHeading(expectedHeading);
        }
        System.out.println("***********************************************************************");
        System.out.println("***********************************************************************");
        if (interrupted) {
            System.out.println("**** AbsoluteSmartTranslateCommand was interrupted                ****");
        } else {
            System.out.println("**** AbsoluteSmartTranslateCommand reached target                 ****");
         }
        System.out.println("***********************************************************************");
        System.out.println("***********************************************************************");
    }
}
