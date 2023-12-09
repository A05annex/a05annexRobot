package org.a05annex.frc.subsystems;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleUnit;
import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * This is a test class built to read data from april tag tracking runs with
 * the {@link org.a05annex.frc.subsystems.SpeedCachedSwerve} and help tune the prediction algorithms of the
 * {@link org.a05annex.frc.subsystems.SpeedCachedSwerve}
 */
public class TuneSpeedCache  extends JFrame implements ActionListener, WindowListener {

    public static void main(@NotNull final String[] args) {
        // Setup the commandline argument parser and parse any commandline arguments
        ArgumentParser parser = ArgumentParsers.newFor("TuneSpeedCache").build()
                .description("Speed cache tuner");
        parser.addArgument("-a", "--aprilTag")
                .type(String.class)
                .help("specify april tag log file");
        parser.addArgument("-s", "--swerve")
                .type(String.class)
                .help("specify a swerve log file");
        String aprilTagFile = null;
        String swerveFile = null;
        try {
            Namespace parsedArgs = parser.parseArgs(args);
            aprilTagFile = parsedArgs.get("aprilTag");
            swerveFile = parsedArgs.get("swerve");
        } catch (ArgumentParserException e) {
            parser.handleError(e);
        }
        // start display window
        try {
            final TuneSpeedCache tuneSpeedCache = new TuneSpeedCache(aprilTagFile, swerveFile);
            tuneSpeedCache.setVisible(true);
        } catch (final Throwable t) {
            t.printStackTrace();
            System.exit(0);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Creating a test SpeedCachedSwerve
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * These are the setting for the calibrated competition robot from '2023 Charged Up' used in testing - might
     * want to move these to a data file
     */
    static A05Constants.RobotSettings robotSettings = new A05Constants.RobotSettings(0, "Competition",
            0.5461, 0.5461, 2.700, 1.161,
            2.723, 2.448, 1.026,0.9650);

    /**
     * The length of the speed cache for this test - let's guess we will not collect more than 60 seconds of test data
     * a about 50 frames/sec (20ms per frame). 50fps * 60sec = 3000frames, use 1000 to give us a safety factor of 3.3
     */
    static private final int TEST_CACHE_LENGTH = 10000;

    static SpeedCachedSwerve getInitializedSCS() {
        SpeedCachedSwerve SCS = SpeedCachedSwerve.getInstance();
        A05Constants.RobotSettings cc = robotSettings;
        SCS.setDriveGeometry(cc.length, cc.width,
                cc.rf, cc.rr, cc.lf, cc.lr,
                cc.maxSpeedCalibration);
        SCS.setCacheLength(TEST_CACHE_LENGTH);
        return SCS;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This is the main TuneSpeedCache instance that controls the application window.
    // -----------------------------------------------------------------------------------------------------------------
    static class ParameterStats {
        double min = Double.NaN;
        double max = Double.NaN;
        double ave = 0.0;
        int ct = 0;
        ParameterStats() { }
        void add(double value) {
            if (Double.isNaN(min) || (value < min)) {
                min = value;
            }
            if (Double.isNaN(max) || (value > max)) {
                max = value;
            }
            ave += value;
            ct++;
        }
        void add(ParameterStats stats) {
            if (Double.isNaN(min) || (stats.min < min)) {
                min = stats.min;
            }
            if (Double.isNaN(max) || (stats.max > max)) {
                max = stats.max;
            }
            ave += stats.ave;
            ct++;
        }
        void doneAdding() {
            ave /= ct;
        }
    }
    static class AprilTagData {
        boolean hasTarget;
        double aprilTime;
        double aprilDistance;
        double aprilStrafe;
        double predictedDistance;
        double predictedStrafe;
        AngleConstantD headingDelta;
        AprilTagData(String csvLine, AprilTagTest aprilTagTest, SpeedCachedSwerve speedCachedSwerve) {
            String[] values = csvLine.split(",");
            hasTarget = Boolean.valueOf(values[0]);
            if (hasTarget) {
                aprilTime = Double.parseDouble(values[1]);
                aprilDistance = Double.parseDouble(values[2]);
                aprilTagTest.aprilDistanceStats.add(aprilDistance);
                aprilStrafe = Double.parseDouble(values[3]);
                aprilTagTest.aprilStrafeStats.add(aprilStrafe);
                predictedDistance = Double.parseDouble(values[4]);
                aprilTagTest.predictedDistanceStats.add(predictedDistance);
                predictedStrafe = Double.parseDouble(values[5]);
                aprilTagTest.predictedStrafeStats.add(predictedStrafe);
                headingDelta = speedCachedSwerve.getExpectedHeadingDeltaAt(aprilTime);
                if (null == headingDelta) {
                    headingDelta = AngleConstantD.ZERO;
                }
            }
            aprilTagTest.aprilPath.add(this);
        }
    }
    static class AprilTagTest {
        List<AprilTagData> aprilPath = new ArrayList<>();
        ParameterStats aprilDistanceStats = new ParameterStats();
        ParameterStats aprilStrafeStats = new ParameterStats();
        ParameterStats predictedDistanceStats = new ParameterStats();
        ParameterStats predictedStrafeStats = new ParameterStats();
        boolean doneAdding = false;

        void doneAdding() {
            if (!doneAdding) {
                aprilDistanceStats.doneAdding();
                aprilStrafeStats.doneAdding();
                predictedDistanceStats.doneAdding();
                predictedStrafeStats.doneAdding();
                doneAdding = true;
            }
        }
    }
    static class AprilTagTestRuns {
        List<AprilTagTest> aprilPath = new ArrayList<>();
        ParameterStats aprilDistanceStats = new ParameterStats();
        ParameterStats aprilStrafeStats = new ParameterStats();
        ParameterStats predictedDistanceStats = new ParameterStats();
        ParameterStats predictedStrafeStats = new ParameterStats();
        boolean doneAdding = false;

        void doneAdding() {
            if (!doneAdding) {
                aprilDistanceStats.doneAdding();
                aprilStrafeStats.doneAdding();
                predictedDistanceStats.doneAdding();
                predictedStrafeStats.doneAdding();
                doneAdding = true;
            }
        }
    }

    static class SpeedCacheData {
        double swerveTime;
        AngleConstantD actualHeading;
        AngleConstantD expectedHeading;
        double speed;
        double direction;
        double rotate;
        SpeedCacheData(String csvLine) {
            String[] values = csvLine.split(",");
            swerveTime = Double.parseDouble(values[0]);
            actualHeading = new AngleConstantD(AngleUnit.RADIANS, Double.parseDouble(values[1]));
            expectedHeading = new AngleConstantD(AngleUnit.RADIANS, Double.parseDouble(values[2]));
            speed = Double.parseDouble(values[3]);
            direction = Double.parseDouble(values[4]);
            rotate = Double.parseDouble(values[5]);
        }
    }
    static final int APRIL_TIME_INDEX = 0;
    static final int APRIL_TIME_DELTA_INDEX = 1;
    static final int APRIL_DISTANCE_INDEX = 2;
    static final int APRIL_STRAFE_INDEX = 3;
    static final int SPEED_CACHE_DISTANCE_INDEX = 4;
    static final int SPEED_CACHE_STRAFE_INDEX = 5;

    static final int SWERVE_TIME_INDEX = 0;
    static final int SWERVE_TIME_DELTA_INDEX = 1;
    static final int SWERVE_SPEED_INDEX = 2;
    static final int SWERVE_DIORECTION_INDEX = 3;
    static final int SWERVE_ROTATE_INDEX = 4;


    int nSizeX = 700;
    int nSizeY = 500;
    private final GraphicsConfiguration graphicsConfig;   // the graphics configuration of the window device

    AprilTagTestRuns aprilTagData = new AprilTagTestRuns();
    List<SpeedCacheData> speedCacheData = new ArrayList<>();

    SpeedCachedSwerve speedCachedSwerve = getInitializedSCS();

    private final TuneSpeedCacheCanvas canvas;
    private final TuneSpeedCacheControls controls;

    TuneSpeedCache( String aprilTagFile, String swerveFile) {
        //-----------------------------------------------------------------------------------------------
        // setup the window for drawing the field and paths
        //-----------------------------------------------------------------------------------------------
        // resize the default if it doesn't fit in the full screen
        graphicsConfig = getGraphicsConfiguration();
        final Rectangle boundsRect = graphicsConfig.getBounds();
        if (boundsRect.width < nSizeX) nSizeX = boundsRect.width;
        if (boundsRect.height < nSizeY) nSizeY = boundsRect.height;
        setSize(nSizeX, nSizeY);

        //-----------------------------------------------------------------------------------------------
        // Get the logged speed cache data and load the SpeedCacheSwerve
        //-----------------------------------------------------------------------------------------------
        try (BufferedReader br = new BufferedReader(new FileReader(swerveFile))) {
            boolean headerLine = true;
            String line;
            while ((line = br.readLine()) != null) {
                if (headerLine) {
                    headerLine = false;
                    continue;
                }
                SpeedCacheData thisLineData = new SpeedCacheData(line);
                speedCacheData.add(thisLineData);
                speedCachedSwerve.addControlRequest(thisLineData.swerveTime, thisLineData.actualHeading,
                        thisLineData.expectedHeading, thisLineData.speed, thisLineData.direction, thisLineData.rotate);
            }
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        //-----------------------------------------------------------------------------------------------
        // Get the logged april tag and speed cache data files, load the SpeedCacheSwerve
        //-----------------------------------------------------------------------------------------------
        AprilTagTest thisAprilSegment = null;
        try (BufferedReader br = new BufferedReader(new FileReader(aprilTagFile))) {
           boolean headerLine = true;
           String line;
            while ((line = br.readLine()) != null) {
                if (headerLine) {
                    headerLine = false;
                    continue;
                }
                if (0 == line.length()) {
                    // empty entry - the end of this test segment
                    thisAprilSegment = closeTestSegment(thisAprilSegment);
                    continue;
                }
                if (null == thisAprilSegment) {
                    thisAprilSegment = new AprilTagTest();
                    aprilTagData.aprilPath.add(thisAprilSegment);
                }
                new AprilTagData(line, thisAprilSegment, speedCachedSwerve);
            }
            if (null != thisAprilSegment) {
                thisAprilSegment.doneAdding();
            }
            aprilTagData.doneAdding();
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }


        // add a menu here when you get to it
        //-----------------------------------------------------------------------------------------------
        // create a canvas to draw on
        //-----------------------------------------------------------------------------------------------
        canvas = new TuneSpeedCacheCanvas(graphicsConfig, aprilTagData, speedCacheData, speedCachedSwerve);
        controls = new TuneSpeedCacheControls(canvas);

        add(canvas, BorderLayout.CENTER);
        add(controls, BorderLayout.LINE_END);

        // and right now everything is so simple that this is the window listener
        addWindowListener(this);

        //------------------------------------------------------------------
        // Setup the app menu
        //------------------------------------------------------------------
        try {
            Desktop desktop = Desktop.getDesktop();
            desktop.setQuitHandler((e, r) -> exitTuneSpeedCache());
        } catch (UnsupportedOperationException e) {
            System.out.println("No desktop quit handler setup, not supported by this platform.");
        }


    }

    AprilTagTest closeTestSegment(AprilTagTest aprilTagPath) {
        aprilTagPath.doneAdding();
        aprilTagData.aprilDistanceStats.add(aprilTagPath.aprilDistanceStats);
        aprilTagData.aprilStrafeStats.add(aprilTagPath.aprilStrafeStats);
        aprilTagData.predictedDistanceStats.add(aprilTagPath.predictedDistanceStats);
        aprilTagData.predictedStrafeStats.add(aprilTagPath.predictedStrafeStats);
        return null;
    }

    private void exitTuneSpeedCache() {
        dispose();
    }

    //------------------------------------------------------------------------------------------------------------------
    // ************** ActionListener **************
    @Override
    public void actionPerformed(ActionEvent e) {

    }

    @Override
    public void windowOpened(WindowEvent e) {

    }

    @Override
    public void windowClosing(WindowEvent e) {
        exitTuneSpeedCache();
    }

    @Override
    public void windowClosed(WindowEvent e) {
    }

    @Override
    public void windowIconified(WindowEvent e) {

    }

    @Override
    public void windowDeiconified(WindowEvent e) {

    }

    @Override
    public void windowActivated(WindowEvent e) {

    }

    @Override
    public void windowDeactivated(WindowEvent e) {

    }

}
