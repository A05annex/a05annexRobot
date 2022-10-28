package org.a05annex.frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

public class DummySwerveDriveSubsystem extends SubsystemBase implements ISwerveDrive {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * is the drive geometry set? Initially {@code false}, but sell be set to {@code true} when the
     * drive geometry and calibration has been initialized.
     */
    private boolean isDriveGeometrySet = false;
    private double DRIVE_LENGTH;
    private double DRIVE_WIDTH;

    // The maximum spin of the robot when only spinning. Since the robot drive centers are
    // rectangular, all wheels are aligned so their axis passes through the center of that rectangle, and all wheels
    // follow the same circular path at a radius of DRIVE_DIAGONAL/2.0 at MAX_METERS_PER_SEC. So this is
    // computed from DRIVE_DIAGONAL and MAX_METERS_PER_SEC:
    //     Max [radians/sec] = max speed [meters/sec] / (PI * radius) [meters/radian]
    //     Max [radians/sec] = MAX_METERS_PER_SEC / (Math.PI * DRIVE_DIAGONAL * 0.5)
    private double MAX_RADIANS_PER_SEC;
    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    private static double DRIVE_POS_TICS_PER_RADIAN;

    /**
     * The Singleton instance of this DummySwerveDriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DummySwerveDriveSubsystem INSTANCE = new DummySwerveDriveSubsystem();

    /**
     * Returns the Singleton instance of this DummySwerveDriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DummySwerveDriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DummySwerveDriveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this DummySwerveDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DummySwerveDriveSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    private void testGeometryIsSet() {
        if (!isDriveGeometrySet) {
            System.out.println();
            System.out.println("**********************************************************************");
            System.out.println("***** THE DRIVE GEOMETRY MUST BE SET BEFORE YOU TRY TO DRIVE !!! *****");
            System.out.println("**********************************************************************");
            System.out.println();
            throw new IllegalStateException("The drive geometry has not been set!");
        }
    }

    @Override
    public void setDriveGeometry(double driveLength, double driveWidth,
                                 double rfCalibration, double rrCalibration,
                                 double lfCalibration, double lrCalibration) {

        isDriveGeometrySet = true;
        DRIVE_LENGTH = driveLength;
        DRIVE_WIDTH = driveWidth;
        double driveDiagonal = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);
        MAX_RADIANS_PER_SEC = (377.0/360.0) * // tested
                ((Mk4NeoModule.MAX_METERS_PER_SEC * 2 * Math.PI) / (Math.PI * driveDiagonal));
        DRIVE_POS_TICS_PER_RADIAN = 10; //TODO: Really compute this number
    }

    @Override
    public double getDriveLength() {
        testGeometryIsSet();
        return DRIVE_LENGTH;
    }

    @Override
    public double getDriveWidth() {
        testGeometryIsSet();
        return DRIVE_WIDTH;
    }

    @Override
    public double getMaxMetersPerSec() {
        testGeometryIsSet();
        return DRIVE_POS_TICS_PER_RADIAN;
    }

    @Override
    public double getMaxRadiansPerSec() {
        testGeometryIsSet();
        return MAX_RADIANS_PER_SEC;
    }

    @Override
    public void setFieldPosition(double fieldX, double fieldY, AngleD heading) {
        testGeometryIsSet();
        System.out.printf("setFieldPosition:                      %10.3f %10.3f %10.3f%n",
                fieldX, fieldY, heading.getDegrees());
    }

    @Override
    public void swerveDriveComponents(double forward, double strafe, double rotation) {
        testGeometryIsSet();
        System.out.printf("swerveDriveComponents:      %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                forward, strafe, rotation);
    }

    @Override
    public void prepareForDriveComponents(double forward, double strafe, double rotation) {
        testGeometryIsSet();
        System.out.printf("prepareForDriveComponents:  %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                forward, strafe, rotation);
    }

    @Override
    public void swerveDrive(AngleConstantD chassisDirection, double speed, double rotation) {
        testGeometryIsSet();
        System.out.printf("swerveDrive:  %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                chassisDirection.getRadians(), speed, rotation);
    }

    @Override
    public void swerveDriveFieldRelative(AngleConstantD fieldDirection, double speed, double rotation) {
        testGeometryIsSet();
        System.out.printf("swerveDriveFieldRelative:  %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                fieldDirection.getRadians(), speed, rotation);
    }

    @Override
    public void setHeading(AngleConstantD targetHeading) {
        testGeometryIsSet();
        System.out.printf("setFieldHeading:        %10.3f%n", targetHeading.getRadians());
    }
}

