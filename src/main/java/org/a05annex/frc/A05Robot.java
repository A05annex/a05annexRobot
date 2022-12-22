package org.a05annex.frc;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.a05annex.frc.subsystems.DriveSubsystem;

/**
 * This is the basic A05annex Robot that does All the common stuff for a swerve drive base with NavX mounted
 * under the base of the robot, and switches for autonomous program and driver selection. This should free
 * the programmers to only worry about the attachments specific to the competition.
 */
public abstract class A05Robot extends TimedRobot {

    private A05RobotContainer a05RobotContainer;

    private Command autonomousCommand = null;

    /**
     * Set the robot container, which must be a superclass of {@link A05RobotContainer}, for this robot.
     *
     * @param container The robot container.
     */
    @SuppressWarnings("unused")
    protected void setRobotContainer(A05RobotContainer container) {
        a05RobotContainer = container;
    }

    /**
     * This method is called every robot command cycle, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit()
    {

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }

    }

    /**
     *  This is the initialization at the start of the autonomous period. If an autonomous command is obtained from
     *  the {@link A05RobotContainer}, then is scheduled to be run in autonomous. */
    @Override
    public void autonomousInit()
    {
        if (A05Constants.getPrintDebug()) {
            System.out.println("A05Robot.autonomousInit called");
        }

        autonomousCommand = a05RobotContainer.getAutonomousCommand();
        // if there is an autonomous command, schedule it.
        if (autonomousCommand != null)
        {
            // Autonomous paths ALWAYS assume FIELD_RELATIVE
            DriveSubsystem.getInstance().setDriveMode(DriveSubsystem.FIELD_RELATIVE);
            autonomousCommand.schedule();
            if (A05Constants.getPrintDebug()) {
                System.out.println("A05Robot.autonomousInit scheduled autonomousCommand");
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This next section is for telemetry in the default labview driver station dashboard, Which allows 10 telemetry
    // slots. If you are using the Shuffleboard dashboard, you will never use these.
    // -----------------------------------------------------------------------------------------------------------------
    Object[] lastLabviewTelemetry = {null, null, null, null, null, null, null, null, null, null};

    /**
     * Update telemetry feedback for a real number value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (double) The number to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, double var) {
        if ((lastLabviewTelemetry[port] == null) || (var != (Double)lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %10.6f", key, var));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Update telemetry feedback for an integer value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (int) The integer to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, int var) {
        if ((lastLabviewTelemetry[port] == null) || (var != (Integer)lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %d", key, var));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Update telemetry feedback for a string value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (String) The string to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, String var) {
        if ((lastLabviewTelemetry[port] == null) || ((var != lastLabviewTelemetry[port])) &&
                !var.equals(lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %s", key, var));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Update telemetry feedback for a boolean value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (boolean) The boolean to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, boolean var) {
        if ((lastLabviewTelemetry[port] == null) || (var != (Boolean)lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port),
                    String.format("%s: %s", key, var ? "on" : "off"));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Initialize the labview telemetry dashboard to empty entries.
     */
    protected void initLabviewTelemetry() {
        for (int i = 0; i < 10; i++) {
            SmartDashboard.putString(String.format("DB/String %d", i), " ");
        }

    }

}
