package org.a05annex.frc.subsystems;

import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.List;

import static org.a05annex.frc.subsystems.TuneSpeedCache.*;

/**
 * This is the canvas where we draw and compare april tag telemetry with speed cache predictions of robot position.
 */
public class TuneSpeedCacheCanvas extends Canvas implements ActionListener {


    // -----------------------------------------------------------------------------------------------------------------
    /**
     * This is a point along a {@link PlottedPath}. The point is represented by a tine, a field-relative point, a delta
     * between the target heading and the current heading, and a screen-relative point used for drawing the
     * point on the path.
     */
    static class PathPoint {
        
        double time;
        Point2D.Double fieldPt;
        Point2D.Double screenPt = new Point2D.Double();
        AngleConstantD deltaHeading;

        PathPoint(double time, double distance, double strafe, AngleConstantD deltaHeading) {
            this.time = time;
            fieldPt = new Point2D.Double(strafe, distance);
            this.deltaHeading = deltaHeading;
        }
        double getStrafe() {
            return fieldPt.getX();
        }
        double getDistance() {
            return fieldPt.getY();
        }
        void transform(AffineTransform xfm) {
            xfm.transform(fieldPt,screenPt);
        }
        public boolean testOverPathPoint(double fieldX, double fieldY, double tolerance) {
            double dx = this.fieldPt.getX() - fieldX;
            double dy = this.fieldPt.getY() - fieldY;
            return Math.sqrt(dx * dx + dy * dy) < tolerance;
        }

    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     *
     */
    static class PlottedPath extends ArrayList<PathPoint> {
        final int index;
        final String name;
        final Color color;
        boolean displayed;

        PlottedPath(int index, String name, Color color, boolean displayed) {
            super();
            this.index = index;
            this.name = name;
            this.color = color;
            this.displayed = displayed;
        }

        void transformPath(AffineTransform drawXfm) {
            for (PathPoint pathPt : this) {
                pathPt.transform(drawXfm);
            }
        }

        void paintPath(Graphics2D g2d) {
            if (displayed) {
                g2d.setPaint(color);
                Point2D.Double lastPt = null;
                for (PathPoint pathPt : this) {
                    Point2D.Double thisPt = pathPt.screenPt;
                    g2d.drawOval((int) thisPt.getX() - 2, (int) thisPt.getY() - 2, 4, 4);
                    if (null != lastPt) {
                        g2d.drawLine((int) lastPt.getX(), (int) lastPt.getY(),
                                (int) thisPt.getX(), (int) thisPt.getY());
                    }
                    lastPt = thisPt;
                }
            }
        }

        /**
         * Test whether a {@link PathPoint} on this path is <i>hit</i> by a screen point (usually the cursor
         * position). Return the hit {@link PathPoint}, or {@code null} if no {@link PathPoint} is hit.
         * @param testPt The test point, in screen coordinates.
         * @param tolerance The hit tolerance, in pixels.
         * @return The hit {@link PathPoint}, or {@code null} if no {@link PathPoint} is hit.
         */
        PathPoint hitTestPath(Point2D.Double testPt, double tolerance) {
            for (PathPoint pathPt : this) {
                Point2D.Double thisPt = pathPt.screenPt;
                if (Utl.inTolerance(thisPt.getX(), testPt.getX(), tolerance) &&
                        Utl.inTolerance(thisPt.getY(), testPt.getY(), tolerance)) {
                    return pathPt;
                }
            }
            return null;
        }

        PathPoint getPointAt (double time) {
            // are we asking for a point on the path
            if ((time > get(0).time) && (time <= get(size()-1).time)) {
                // find the position and interpolate between the points before and after this time
                PathPoint lastPt = null;
                PathPoint nextPt = null;
                for (PathPoint pathPt : this) {
                    lastPt = nextPt;
                    nextPt =  pathPt;
                    if (time < nextPt.time) {
                        // interpolate position between lastPt and nextPt
                        double position = (time - lastPt.time) / (nextPt.time - lastPt.time);
                        double distance = (position * nextPt.getDistance()) + ((1.0 -position) * lastPt.getDistance());
                        double strafe = (position * nextPt.getStrafe()) + ((1.0 - position) * lastPt.getStrafe());
                        AngleConstantD deltaHeading = new AngleConstantD(AngleUnit.RADIANS,
                                (position * nextPt.deltaHeading.getRadians()) +
                                        ((1.0 -position) * lastPt.getDistance()));
                        return new PathPoint(time, distance, strafe, deltaHeading);
                    }
                }
            }
            // nope, outside this path
            return null;
        }

    }

    /**
     * This is a path plotted from a specific point on the april tag path. It is a 0.5 second path that highlights
     * the 100ms (0.1 second) points on both the projected path and the april path being projected from.
     */
    static class PlottedPathFromPoint extends PlottedPath {
        PlottedTest startTest = null;
        PlottedPath startPath = null;
        PathPoint startPoint = null;
        ArrayList<PathPoint> pathKeyPoints = new ArrayList<>();
        ArrayList<PathPoint> refPathKeyPoints = new ArrayList<>();
        PlottedPathFromPoint(int index, String name, Color color, boolean displayed) {
            super(index, name, color, displayed);
        }

        void paintPath(Graphics2D g2d) {
            if (displayed) {
                if (null != startPath) {
                    super.paintPath(g2d);
                    g2d.setColor(startPath.color);
                    g2d.setStroke(highlightStroke);
                    for (PathPoint keyPt : refPathKeyPoints) {
                        Point2D.Double thisPt = keyPt.screenPt;
                        g2d.drawOval((int) thisPt.getX() - 3, (int) thisPt.getY() - 3, 6, 6);

                    }
                    g2d.setColor(color);
                    for (PathPoint keyPt : pathKeyPoints) {
                        Point2D.Double thisPt = keyPt.screenPt;
                        g2d.drawOval((int) thisPt.getX() - 3, (int) thisPt.getY() - 3, 6, 6);

                    }
                }
            }
        }
        void transformPath(AffineTransform drawXfm) {
            if (null != startPath) {
                super.transformPath((drawXfm));
                for (PathPoint keyPt : refPathKeyPoints) {
                    keyPt.transform(drawXfm);
                }
            }
        }
    }

        // -----------------------------------------------------------------------------------------------------------------
    // These are the paths we are plotting
    /**
     * This is the path reported by the Photonvision library as interpreted by our {@link PhotonCameraWrapper}
     */
    static public final int APRIL_TAG_PATH = 0;
    /**
     * This is a path derived from the {@link #APRIL_TAG_PATH} where the position has been corrected the cached
     * deltaHeading to determine the correct location for deltaHeading = 0.0
     */
    static public final int HEADING_CORRECTED_APRIL_TAG_PATH = 1;
    static public final int FILTERED_APRIL_TAG_PATH = 2;
    static public final int SWERVE_PATH = 3;
    static public final int SPEED_CACHE_PATH = 4;
    static public final int SPEED_CACHE_FROM_SELECTED = 5;

    static class PlottedTest {
        List<PlottedPath> plottedPaths = new ArrayList<>(Arrays.asList(
                new PlottedPath(APRIL_TAG_PATH, "April Tag Path", Color.WHITE, true),
                new PlottedPath(HEADING_CORRECTED_APRIL_TAG_PATH,"Nav Corrected Tag Path", Color.YELLOW, true),
                new PlottedPath(FILTERED_APRIL_TAG_PATH,"Filtered April Tag Path", Color.MAGENTA, false),
                new PlottedPath(SWERVE_PATH,"Projected Path at Test", Color.CYAN, false),
                new PlottedPath(SPEED_CACHE_PATH,"Cache Computed Path", Color.ORANGE, true),
                new PlottedPathFromPoint(SPEED_CACHE_FROM_SELECTED,"Cache Computed from Selected",
                        Color.GREEN, true)
        ));
        boolean isDisplayed = true;
        final int index;

        PlottedTest(int index) {
            this.index = index;
        }

    }

    List<PlottedTest> plottedPaths = new ArrayList<>();

    PlottedTest selectedTest = null;
    PlottedPath selectedPath = null;
    PathPoint selectedPoint = null;
    PlottedTest hitTest = null;
    PlottedPath hitPath = null;
    PathPoint hitPoint = null;
    static final Stroke normalStroke = new BasicStroke(1.0f);
    static final Stroke highlightStroke = new BasicStroke(3.0f);

    // This is the raw data
    AprilTagTestRuns aprilTagData;
    List<SpeedCacheData> swerveData;
    // This is the swerve cache loaded with the raw data
    SpeedCachedSwerve speedCachedSwerve;

    // the back buffer to support double buffering
    private int bufferWidth;
    private int bufferHeight;
    private Image bufferImage;
    private Graphics bufferGraphics;

    private Point2D.Double mouse = null;
    private PathPoint mouseOverPathPt = null;

    AffineTransform drawXfm = null;
    private AffineTransform mouseXfm = null;
    private double scale;

    /**
     * This is the handler for resizing. The main thing in resizing is that we scale the
     * drawing so the field fills the window but maintains the correct field aspect ratio.
     */
    private class ComponentHandler extends ComponentAdapter {
        public void componentResized(ComponentEvent e) {
            Component comp = e.getComponent();
            float width = comp.getWidth();
            float height = comp.getHeight();
            System.out.printf("Size Changed %d,%d%n", (int) width, (int) height);
            ((TuneSpeedCacheCanvas)comp).resetDisplayGeometry();
        }
    }
    // ------------------------------ start MouseHandler extends MouseAdapter ------------------------------------------
    /**
     * This is the handler for mouse actions on the path planning canvas. It handles highlighting
     * what the mouse is over, selection of editable targets (like control point location of tangent
     * and heading handles), and dragging things around on the game canvas.
     */
    private class MouseHandler extends MouseAdapter{

        @Override
        public void mouseClicked(MouseEvent e) {}

        @Override
        public void mousePressed(MouseEvent e) {
            if (null != hitPoint) {
                selectedTest = hitTest;
                selectedPath = hitPath;
                selectedPoint = hitPoint;
                loadPathFromSelectedPoint();
                repaint();
            }
        }

        @Override
        public void mouseReleased(MouseEvent e) {}

        @Override
        public void mouseEntered(MouseEvent e) {
            mouse = (Point2D.Double) mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
        }
        @Override
        public void mouseExited(MouseEvent e) {}
        @Override
        public void mouseWheelMoved(MouseWheelEvent e) {
            System.out.println("mouseWheel: " + e);
        }
        @Override
        public void mouseDragged(MouseEvent e) {
            Point2D pt = mouse = (Point2D.Double) mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
        }
        @Override
        public void mouseMoved(MouseEvent e) {
            Point2D pt = mouse = (Point2D.Double) mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
            testMouseOver(new Point2D.Double(e.getPoint().getX(),e.getPoint().getY()));
            repaint();
        }
        private void testMouseOver(@NotNull Point2D.Double pt) {
            // initially we will only test on the april, filtered april, and nav corrected paths
            int hitTestPaths[] = {APRIL_TAG_PATH, FILTERED_APRIL_TAG_PATH, HEADING_CORRECTED_APRIL_TAG_PATH};
            for (int pathIndex : hitTestPaths) {
                for (PlottedTest thisTest : plottedPaths) {
                    PlottedPath testPath = thisTest.plottedPaths.get(pathIndex);
                    if (testPath.displayed) {
                        hitPoint = testPath.hitTestPath(pt, 2.0);
                        if (null != hitPoint) {
                            hitTest = thisTest;
                            hitPath = testPath;
                            return;
                        }
                    }
                }
            }
            hitTest = null;
            hitPath = null;
            hitPoint = null;
        }
    }

    TuneSpeedCacheCanvas(@NotNull GraphicsConfiguration gc,
                         @NotNull AprilTagTestRuns aprilTagData,
                         @NotNull List<SpeedCacheData> speedCacheData,
                         @NotNull SpeedCachedSwerve speedCachedSwerve)
    {
        super(gc);

        // first thing, there may be multiple tests in this data set. So inspect the april tag data that has been
        // passed in, and figure out how many tests are in this data set. Then create a path data set for each of
        // the test data sets that was passed it.
        this.aprilTagData = aprilTagData;
        this.swerveData = speedCacheData;
        this.speedCachedSwerve = speedCachedSwerve;
        int testIndex = 0;
        for (AprilTagTest thisTest : aprilTagData.aprilPath) {
            // create the paths for this data set.
            PlottedTest plottedTest = new PlottedTest(testIndex);
            testIndex++;
            plottedPaths.add(plottedTest);
            // This is the stuff that is logged about the april tag and projected position at each command cycle
            // that the target is visible.
            double lastTime = 0.0;
            double lastLastTime = 0.0;
            Point2D.Double lastPt = null;
            Point2D.Double lastLastPt = null;
            for (AprilTagData aprilTag : thisTest.aprilPath) {
                if (aprilTag.hasTarget) {
                    plottedTest.plottedPaths.get(SWERVE_PATH).add(new PathPoint(aprilTag.aprilTime,
                            aprilTag.predictedDistance, -aprilTag.predictedStrafe, aprilTag.headingDelta));
                    PathPoint pathPt = new PathPoint(aprilTag.aprilTime,
                            aprilTag.aprilDistance, aprilTag.aprilStrafe, aprilTag.headingDelta);
                    plottedTest.plottedPaths.get(APRIL_TAG_PATH).add(pathPt);
                    // this is the computation for a 3 point weighted average filtering for the position value.
                    Point2D.Double thisPt = pathPt.fieldPt;
                    double thisTime = pathPt.time;
                    Point2D.Double thisAvePt;
                    double thisAveTime;
                    if ((null == lastLastPt) && (null == lastPt)) {
                        thisAvePt = new Point2D.Double(thisPt.getX(), thisPt.getY());
                        thisAveTime = thisTime;
                    } else if (null == lastLastPt) {
                        thisAvePt = new Point2D.Double((thisPt.getX() + lastPt.getX()) / 2.0,
                                (thisPt.getY() + lastPt.getY()) / 2.0);
                        thisAveTime = (thisTime + lastTime) / 2.0;
                    } else {
                        // This is an 0.2, 0.6, 0.2 filtering (a gaussian-like filter for running 3 points - looks
                        // gretty good in the path plots.
                        thisAvePt = new Point2D.Double(
                                (0.2 * thisPt.getX()) + (0.6 * lastPt.getX()) + (0.2 * lastLastPt.getX()),
                                (0.2 * thisPt.getY()) + (0.6 * lastPt.getY()) + (0.2 * lastLastPt.getY()));
                        thisAveTime = (0.2 * thisTime) + (0.6 * lastTime) + (0.2 * lastLastTime);
                    }
                    AngleConstantD headingDelta = speedCachedSwerve.getExpectedHeadingDeltaAt(thisAveTime);
                    if (null == headingDelta) {
                        headingDelta = AngleConstantD.ZERO;
                    }
                    plottedTest.plottedPaths.get(FILTERED_APRIL_TAG_PATH).add(new PathPoint(thisAveTime,
                            thisAvePt.getY(), thisAvePt.getX(), headingDelta));
                    lastLastPt = lastPt;
                    lastPt = thisPt;
                    lastLastTime = lastTime;
                    lastTime = thisTime;
                }

            }

            loadHeadingCorrectedAprilTagPath(speedCachedSwerve,
                    plottedTest.plottedPaths.get(APRIL_TAG_PATH),
                    plottedTest.plottedPaths.get(HEADING_CORRECTED_APRIL_TAG_PATH));

            loadCalculatedSpeedCachePath(speedCacheData, speedCachedSwerve,
                    plottedTest.plottedPaths.get(FILTERED_APRIL_TAG_PATH),
                    plottedTest.plottedPaths.get(SPEED_CACHE_PATH));
        }



        //------------------------------------------------------------------
        // setup the window for drawing the april target and the paths around it
        //------------------------------------------------------------------
        setBackground(Color.BLACK);
        MouseAdapter mouseHandler = new MouseHandler();
        addMouseListener(mouseHandler);
        addMouseMotionListener(mouseHandler);
        addComponentListener(new ComponentHandler());
        resetDisplayGeometry();

    }

    /**
     *
     * @param speedCachedSwerve
     * @param aprilPath
     * @param plottedPath
     */
    void loadHeadingCorrectedAprilTagPath(@NotNull SpeedCachedSwerve speedCachedSwerve,
                                          PlottedPath aprilPath, PlottedPath plottedPath) {
        plottedPath.clear();
        for (PathPoint aprilTag : aprilPath) {
            // this corrects the photonvision distance and strafe when the heading drifts from the target heading
            double time = aprilTag.time;    // the suspected time of the frame this tag is from
            AngleConstantD headingDelta = speedCachedSwerve.getExpectedHeadingDeltaAt(aprilTag.time);
            if (null == headingDelta) {
                headingDelta = AngleConstantD.ZERO;
            }
            double aprilDistance = Math.sqrt((aprilTag.getDistance() * aprilTag.getDistance()) +
                            (aprilTag.getStrafe() * aprilTag.getStrafe()));
            AngleD aprilAngle = new AngleD().atan2(aprilTag.getStrafe(),aprilTag.getDistance());
            aprilAngle.add(headingDelta);
            double strafe = aprilDistance * aprilAngle.sin();
            double distance = aprilDistance * aprilAngle.cos();
            plottedPath.add(new PathPoint(time,distance, strafe, AngleConstantD.ZERO));
        }
    }

    /**
     *
     * @param speedCacheData
     * @param speedCachedSwerve
     * @param aprilPath
     * @param plottedPath
     */
    void loadCalculatedSpeedCachePath(@NotNull List<SpeedCacheData> speedCacheData,
                                      @NotNull SpeedCachedSwerve speedCachedSwerve,
                                      PlottedPath aprilPath, PlottedPath plottedPath) {
        double speedCacheTime = speedCacheData.get(1).swerveTime;

        PathPoint aprilTagStart = null;
        PathPoint aprilTagEnd = aprilPath.get(aprilPath.size()-1);
        for (PathPoint aprilTag : aprilPath) {
            if (aprilTag.time > speedCacheTime) {
                aprilTagStart = aprilTag;
                break;
            }
        }
        double aprilTagStartTime = aprilTagStart.time;
        double aprilTagEndTime = aprilTagEnd.time;
        double aprilTagStartDistance = aprilTagStart.getDistance();
        double aprilTagStartStrafe = aprilTagStart.getStrafe();
        AngleConstantD aprilTagStartDelta = aprilTagStart.deltaHeading;

        plottedPath.clear();
        plottedPath.add(new PathPoint(aprilTagStartTime,
                aprilTagStartDistance,aprilTagStartStrafe,aprilTagStartDelta));
        for (SpeedCacheData swerveCommand : speedCacheData) {
            speedCacheTime = swerveCommand.swerveTime;
            if (speedCacheTime > aprilTagEndTime) {
                break;
            }
            if (speedCacheTime > aprilTagStartTime) {
                SpeedCachedSwerve.RobotRelativePosition relPosition =
                        speedCachedSwerve.getRobotRelativePositionSince(speedCacheTime,
                                aprilTagStartTime);
                plottedPath.add(new PathPoint(speedCacheTime,
                        aprilTagStartDistance - relPosition.forward,
                        aprilTagStartStrafe - relPosition.strafe,
                        new AngleD(swerveCommand.expectedHeading).subtract(swerveCommand.actualHeading)));
            }
        }
    }

    /**
     * The real question about future path projection is whether it is useful in the context of the completion
     * of competition tasks. In general, we either have full view of the target during positioning (accuracy during
     * camera latency is our only concern, typically less than 100ms); or, the desired final position is outside
     * the target viewing range - usually short, so let's guess about 500ms (0.5 sec) is the maximum time the
     * robot is outside the last seen target
     * method loads a path for the next 1000ms (1.0 second, about 50 command cycles) the idea being that if the april
     * image was lost at the selected point, how well would the speed cache project the future path.
     */
    void loadPathFromSelectedPoint() {
        if (null != selectedPoint) {
            PlottedPathFromPoint thisPath = (PlottedPathFromPoint)plottedPaths.get(selectedTest.index).plottedPaths.
                    get(SPEED_CACHE_FROM_SELECTED);
            thisPath.startTest = selectedTest;
            thisPath.startPath = selectedPath;
            thisPath.startPoint = selectedPoint;
            thisPath.clear();
            thisPath.pathKeyPoints.clear();
            thisPath.refPathKeyPoints.clear();

            double startTime = selectedPoint.time;
            double endTime = startTime + 1.0;
            if (speedCachedSwerve.getMostRecentControlRequest().timeStamp < endTime) {
                endTime = speedCachedSwerve.getMostRecentControlRequest().timeStamp;
            }
            thisPath.add(selectedPoint);
            int index = 0;
            for (double nextTime = startTime + 0.02; nextTime <= endTime; nextTime += 0.02) {
                SpeedCachedSwerve.RobotRelativePosition relPosition =
                        speedCachedSwerve.getRobotRelativePositionSince(nextTime - speedCachedSwerve.getLatencyOffset(), startTime);
                PathPoint thisPt = new PathPoint(nextTime,
                        selectedPoint.fieldPt.getY() - relPosition.forward,
                        selectedPoint.fieldPt.getX() - relPosition.strafe, AngleConstantD.ZERO);
                thisPt.transform(drawXfm);
                thisPath.add(thisPt);
                if (0 == index % 5) {
                    // highlight every 5th point (0.1 sec) to make it easier to compare the projected april tag path
                    // with the actual april tag path
                    thisPath.pathKeyPoints.add(thisPt);
                    PathPoint refKey = selectedPath.getPointAt(nextTime);
                    if (null != refKey) {
                        refKey.transform(drawXfm);
                        thisPath.refPathKeyPoints.add(refKey);
                    }
                }
                index++;
            }
         }
    }

    /**
     *
     * @param e the event to be processed
     */
    @Override
    public void actionPerformed(ActionEvent e) {
        System.out.println("action: " + e);

    }

    /**
     * This method resets the {@link #drawXfm} (field to window) and {@link #mouseXfm} (window to field), and should
     * be called whenever field geometry changes (a new calibration run is loaded), or, the window size changes.
     * Here we look at the geometry of the april tag and the april tag and predicted paths to guess at a good
     * scaling and offset that will display the geometry with room for trying modifications to the future path
     * prediction algorithms to see how they affect the projected path
     */
    public void resetDisplayGeometry() {
        float width = this.getWidth();
        float height = this.getHeight();
        if ((0 < width) && (0 < height)) {
            // The first step here - the april target is near the top of the screen at 0.0, positive distance is
            // down, positive strafe is to the right. We need a size of the display - assume starting Y at -0.10
            // (behind the target), and max at max (april dist or predicted dist) + 1m; for X, use min-max
            // (april dist or predicted dist) + .50m (.25m on each side) - this lets us compute the scale factor.
            double distMax = Utl.max(aprilTagData.aprilDistanceStats.max, aprilTagData.predictedDistanceStats.max);
            double scaleY = height / (distMax + 0.1 + 0.25);
            double strafeMin = Utl.min(aprilTagData.aprilStrafeStats.min, aprilTagData.predictedStrafeStats.min);
            double strafeMax = Utl.max(aprilTagData.aprilStrafeStats.max, aprilTagData.predictedStrafeStats.max);
            double scaleX = width / ((strafeMax - strafeMin) + 0.5);
            scale = Math.min(scaleX, scaleY);
            // second step - get the translation that moves the april target to 0.25m from the top of the window.
            // OK, what is happening here?? Magic - well, not really. The width/2.0 and height/2.0 bits of the
            // m02 and m12 shift the origin to the center of the screen window. For the default competition field
            // this is great because we adopted 0,0 as center field.
            drawXfm = new AffineTransform(scale, 0.0f, 0.0f, scale,
                    (width / 2.0) -
                            (scale * (((strafeMax - strafeMin) / 2.0) + strafeMin)),
                    (height / 2.0) +
                            (-scale * (((distMax - 0.1) / 2.0) + 0.1)));
            mouseXfm = new AffineTransform(drawXfm);
            try {
                mouseXfm.invert();
            } catch (NoninvertibleTransformException ex) {
                System.out.println("  -- can't invert draw transform");
            }

            // transform the paths
            for(PlottedTest plottedTest : plottedPaths) {
                for (PlottedPath path : plottedTest.plottedPaths) {
                    path.transformPath(drawXfm);
                }
            }
        }
    }

    /**
     * This override does not do anything other than call {@link #paint(Graphics)}. The overridden method assumed
     * the update was drawing to the displayed video buffer, so it cleared the buffer and then drew the new content,
     * which results in a lot of screen flashing for older video cards.
     *
     * @param g The graphic context to be updated (repainted)
     */
    @Override
    public void update(Graphics g) {
        paint(g);
    }

    /**
     * Paint the panel, which, in the double buffer context means:
     * <ul>
     *     <li>make sure there is a back buffer that is the size of the panel.</li>
     *     <li>clear the back buffer</li>
     *     <li>paint the current content into the back buffer</li>
     *     <li>copy the back buffer to the panel</li>
     * </ul>
     *
     * @param g The graphics context for drawing to tha back buffer.
     */
    @Override
    public void paint(Graphics g) {
        // make sure there is a back buffer that is the size of the onscreen panel
        if (bufferWidth != getSize().width ||
                bufferHeight != getSize().height ||
                bufferImage == null || bufferGraphics == null) {
            pkgResetBuffer();
        }
        if (bufferGraphics != null) {
            //this clears the back buffer
            bufferGraphics.clearRect(0, 0, bufferWidth, bufferHeight);

            // draw the content to the back buffer
            pkgPaintBuffer(bufferGraphics);

            // copy the back buffer into this displayed panel
            g.drawImage(bufferImage, 0, 0, this);
        }
    }

    /**
     * Create a back buffer that is the size of the panel.
     */
    private void pkgResetBuffer() {
        // always keep track of the image size
        bufferWidth = getSize().width;
        bufferHeight = getSize().height;

        //    clean up the previous image
        if (bufferGraphics != null) {
            bufferGraphics.dispose();
            bufferGraphics = null;
        }
        if (bufferImage != null) {
            bufferImage.flush();
            bufferImage = null;
        }

        //    create the new image with the size of the panel
        bufferImage = createImage(bufferWidth, bufferHeight);
        bufferGraphics = bufferImage.getGraphics();
    }

    /**
     * Paint everything we want to see to the back buffer. This happens while the user sees the font buffer, so there
     * is no effect on the user while the paint is happening. Once this paint finishes, the screen will flip to
     * display the content of the back buffer as the screen buffer - displaying everything painted here.
     *
     * @param g The graphics description for the back buffer.
     */
    public void pkgPaintBuffer(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setPaint(Color.WHITE);
        g2d.setStroke(normalStroke);


        // draw the target
        Point2D.Double leftPt = (Point2D.Double) drawXfm.transform(
                new Point2D.Double(-0.1016, 0.0), null);
        Point2D.Double rightPt = (Point2D.Double) drawXfm.transform(
                new Point2D.Double(0.1016, 0.0), null);
        g2d.drawLine((int) leftPt.getX(), (int) leftPt.getY(),
                (int) rightPt.getX(), (int) rightPt.getY());

        // draw the april path, the filtered path, and the path projected from start.
        for (PlottedTest plottedTest : plottedPaths) {
            if (plottedTest.isDisplayed) {
                for (PlottedPath path : plottedTest.plottedPaths) {
                    path.paintPath(g2d);
                }
            }
        }

        // Is there a mouse point over?
        if (null != hitPoint) {
            g2d.setPaint(Color.RED);
            g2d.setStroke(highlightStroke);
            Point2D.Double thisPt = hitPoint.screenPt;
            g2d.drawOval((int) thisPt.getX() - 3, (int) thisPt.getY() - 3, 6, 6);
        }


        // draw the mouse and tracking info
        // TODO: handle repositioning the text when the cursor gets to the edge of
        //  the window.
        if (null != mouse) {
            if (null != hitPoint) {
                g2d.setPaint(Color.RED);
                g2d.drawString(
                        String.format(" %.3fsec,(%.4f,%.4f,%.4f)", hitPoint.time, hitPoint.fieldPt.getX(),
                                hitPoint.fieldPt.getY(), hitPoint.deltaHeading.getRadians()),
                            (int) hitPoint.screenPt.getX() + 4, (int) hitPoint.screenPt.getY() - 4);
            } else {
                g2d.setPaint(Color.WHITE);
                Point2D screenMouse = drawXfm.transform(mouse, null);
                g2d.drawString(
                        String.format(" (%.4f,%.4f)", mouse.getX(), mouse.getY()),
                        (int) screenMouse.getX() + 4, (int) screenMouse.getY() - 4);
            }
        }
    }
}
