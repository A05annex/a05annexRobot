package org.a05annex.frc.subsystems;

import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
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
     *
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

        PathPoint hitTestPath(Point2D.Double pt, double tolerance) {
            for (PathPoint pathPt : this) {
                Point2D.Double thisPt = pathPt.screenPt;
                if (Utl.inTolerance(thisPt.getX(), pt.getX(), tolerance) &&
                        Utl.inTolerance(thisPt.getY(), pt.getY(), tolerance)) {
                    return pathPt;
                }
            }
            return null;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // These are the paths we are plotting
    static public final int APRIL_TAG_PATH = 0;
    static public final int HEADING_CORRECTED_APRIL_TAG_PATH = 1;
    static public final int FILTERED_APRIL_TAG_PATH = 2;
    static public final int SWERVE_PATH = 3;
    static public final int SPEED_CACHE_PATH = 4;
    static public final int SPEED_CACHE_FROM_SELECTED = 5;

    static class PlottedTest {
        List<PlottedPath> plottedPaths = new ArrayList<>(Arrays.asList(
                new PlottedPath(APRIL_TAG_PATH, "April Tag Path", Color.WHITE, true),
                new PlottedPath(HEADING_CORRECTED_APRIL_TAG_PATH,"Nav Corrected Tag Path", Color.YELLOW, true),
                new PlottedPath(FILTERED_APRIL_TAG_PATH,"Filtered April Tag Path", Color.MAGENTA, true),
                new PlottedPath(SWERVE_PATH,"Projected Path at Test", Color.CYAN, true),
                new PlottedPath(SPEED_CACHE_PATH,"Cache Computed Path", Color.ORANGE, true),
                new PlottedPath(SPEED_CACHE_FROM_SELECTED,"Cache Computed from Selected", Color.BLUE, true)
        ));
        boolean isDisplayed = true;
    }

    List<PlottedTest> plottedPaths = new ArrayList<>();

    PlottedPath hitPath = null;
    PathPoint hitPoint = null;
    private final Stroke normalStroke = new BasicStroke(1.0f);
    private final Stroke highlightStroke = new BasicStroke(3.0f);

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
        public void mousePressed(MouseEvent e) {}

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
                            hitPath = testPath;
                            return;
                        }
                    }
                }
            }
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
        for (AprilTagTest thisTest : aprilTagData.aprilPath) {
            // create the paths for this data set.
            PlottedTest plottedTest = new PlottedTest();
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
                            aprilTag.predictedDistance, aprilTag.predictedStrafe, aprilTag.headingDelta));
                    PathPoint pathPt = new PathPoint(aprilTag.aprilTime,
                            aprilTag.aprilDistance, aprilTag.aprilStrafe, aprilTag.headingDelta);
                    plottedTest.plottedPaths.get(APRIL_TAG_PATH).add(pathPt);
                    // this corrects the distance and strafe when the heading drifts
                    AngleConstantD headingDelta = speedCachedSwerve.getExpectedHeadingDeltaAt(aprilTag.aprilTime);
                    if (null == headingDelta) {
                        headingDelta = AngleConstantD.ZERO;
                    }
                    double actDist = (pathPt.fieldPt.getY() * headingDelta.cos()) -
                            (pathPt.fieldPt.getX() * headingDelta.sin());
                    double actStrafe = (pathPt.fieldPt.getY() * headingDelta.sin()) +
                            (pathPt.fieldPt.getX() * headingDelta.cos());
                    pathPt = new PathPoint(aprilTag.aprilTime,actDist, actStrafe, headingDelta);
                    plottedTest.plottedPaths.get(HEADING_CORRECTED_APRIL_TAG_PATH).add(pathPt);
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
//                      // This is an 0.2, 0.8, 0.2 filtering (a gaussian-like filter for running 3 points - looks
                        // gretty good in the path plots.
                        thisAvePt = new Point2D.Double(
                                (0.2 * thisPt.getX()) + (0.6 * lastPt.getX()) + (0.2 * lastLastPt.getX()),
                                (0.2 * thisPt.getY()) + (0.6 * lastPt.getY()) + (0.2 * lastLastPt.getY()));
                        thisAveTime = (0.2 * thisTime) + (0.6 * lastTime) + (0.2 * lastLastTime);
                    }
                    headingDelta = speedCachedSwerve.getExpectedHeadingDeltaAt(thisAveTime);
                    if (null == headingDelta) {
                        headingDelta = AngleConstantD.ZERO;
                    }
                    plottedTest.plottedPaths.get(FILTERED_APRIL_TAG_PATH).add(new PathPoint(thisAveTime,
                            thisAvePt.getY(), thisAvePt.getX(), headingDelta));
//                  lastLastPt = lastPt;
                    lastPt = thisPt;
                    lastLastTime = lastTime;
                    lastTime = thisTime;
                }

            }

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
