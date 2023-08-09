package org.a05annex.frc.subsystems;

import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import static org.a05annex.frc.subsystems.TuneSpeedCache.*;

/**
 * This is the canvas where we draw and compare april tag telemetry with speed cache predictions of robot position.
 */
public class TuneSpeedCacheCanvas extends Canvas implements ActionListener {

    private int m_nSizeX = 565;     // the initial X size of the app window.
    private int m_nSizeY = 574;     // the initial Y size of the app window.
    private final GraphicsConfiguration graphicsConfig = null;  // the graphics configuration of the window device

    static class PathPoint {
        double time;
        Point2D.Double fieldPt;
        Point2D.Double screenPt = new Point2D.Double();

        PathPoint(double time, double distance, double strafe) {
            this.time = time;
            fieldPt = new Point2D.Double(strafe, distance);
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

    List<List<Double>> aprilTagData;
    List<TuneSpeedCache.ColumnStats> aprilTagStats;
    List<PathPoint> aprilTagPath = new ArrayList<>();
    List<PathPoint> filteredAprilTagPath = new ArrayList<>();
    List<PathPoint> swervePath = new ArrayList<>();
    List<List<Double>> swerveData;
    List<TuneSpeedCache.ColumnStats> swerveStats;
    SpeedCachedSwerve speedCachedSwerve;
    List<PathPoint> speedCachePath = new ArrayList<>();

    // the back buffer to support double buffering
    private int bufferWidth;
    private int bufferHeight;
    private Image bufferImage;
    private Graphics bufferGraphics;

    private Point2D.Double mouse = null;
    private PathPoint mouseOverPathPt = null;

    private AffineTransform drawXfm = null;
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
    private class MouseHandler extends MouseAdapter {

        @Override
        public void mouseEntered(MouseEvent e) {
            mouse = (Point2D.Double) mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
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
            repaint();
        }
        private void testMouseOver(@NotNull Point2D pt) {

        }
    }

    TuneSpeedCacheCanvas(@NotNull GraphicsConfiguration gc,
                         @NotNull List<List<Double>> aprilTagData,
                         @NotNull List<TuneSpeedCache.ColumnStats> aprilTagStats,
                         @NotNull List<List<Double>> swerveData,
                         @NotNull List<TuneSpeedCache.ColumnStats> swerveStats,
                         @NotNull SpeedCachedSwerve speedCachedSwerve)
    {
        super(gc);

        // This is the stuff that is logged about the april tag and projected position at each command cycle
        // that the target is visible.
        this.aprilTagData = aprilTagData;
        this.aprilTagStats = aprilTagStats;
        double lastTime = 0.0;
        double lastLastTime = 0.0;
        Point2D.Double lastPt = null;
        Point2D.Double lastLastPt = null;
        for (List<Double> aprilTag : aprilTagData) {
            swervePath.add(new PathPoint(aprilTag.get(APRIL_TIME_INDEX),
                    aprilTag.get(SPEED_CACHE_DISTANCE_INDEX), aprilTag.get(SPEED_CACHE_STRAFE_INDEX)));
            PathPoint pathPt = new PathPoint(aprilTag.get(APRIL_TIME_INDEX),
                    aprilTag.get(APRIL_DISTANCE_INDEX),aprilTag.get(APRIL_STRAFE_INDEX));
            aprilTagPath.add(pathPt);
            // this is the computation for a 3 point running average filtering for the position value.
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
//                thisAvePt = new Point2D.Double((thisPt.getX() + lastPt.getX() + lastLastPt.getX()) / 3.0,
//                        (thisPt.getY() + lastPt.getY() + lastLastPt.getY()) / 3.0);
//                thisAveTime = (thisTime + lastTime + lastLastTime) / 3.0;
                thisAvePt = new Point2D.Double(
                        ((0.5 * thisPt.getX()) + lastPt.getX() + (0.5 * lastLastPt.getX())) / 2.0,
                        ((0.5 * thisPt.getY()) + lastPt.getY() + (0.5 * lastLastPt.getY())) / 2.0);
                thisAveTime = ((0.5 * thisTime) + lastTime + (0.5 * lastLastTime)) / 2.0;
            }
            filteredAprilTagPath.add(new PathPoint(thisAveTime,
                    thisAvePt.getY(),thisAvePt.getX()));
            lastLastPt = lastPt;
            lastPt = thisPt;
            lastLastTime = lastTime;
            lastTime = thisTime;

        }

        this.swerveData = swerveData;
        this.swerveStats = swerveStats;
        this.speedCachedSwerve = speedCachedSwerve;
        double speedCacheTime = swerveData.get(0).get(SWERVE_TIME_INDEX);
        List<Double> aprilTagStart = null;
        for (List<Double> aprilTag : aprilTagData) {
            if (aprilTag.get(APRIL_TIME_INDEX) > speedCacheTime) {
                aprilTagStart = aprilTag;
                break;
            }
        }
        double aprilTagStartTime = aprilTagStart.get(APRIL_TIME_INDEX);
        double aprilTagStartDistance = aprilTagStart.get(APRIL_DISTANCE_INDEX);
        double aprilTagStartStrafe = aprilTagStart.get(APRIL_STRAFE_INDEX);
        speedCachePath.add(new PathPoint(aprilTagStartTime,aprilTagStartDistance,aprilTagStartStrafe));
        for (List<Double> swerveCommand : swerveData) {
            speedCacheTime = swerveCommand.get(SWERVE_TIME_INDEX);
            if (speedCacheTime > aprilTagStartTime) {
                SpeedCachedSwerve.RobotRelativePosition relPosition =
                        speedCachedSwerve.getRobotRelativePositionSince( speedCacheTime, aprilTagStartTime);
                speedCachePath.add(new PathPoint(speedCacheTime,aprilTagStartDistance - relPosition.forward,
                        aprilTagStartStrafe - relPosition.strafe));
            }
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

    @Override
    public void actionPerformed(ActionEvent e) {

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
            // (april dist or predicted dist) + .50m (.25m on each side) - this let's us compute the scale factor.
            double distMax = Utl.max(aprilTagStats.get(TuneSpeedCache.APRIL_DISTANCE_INDEX).max,
                    aprilTagStats.get(TuneSpeedCache.SPEED_CACHE_DISTANCE_INDEX).max);
            double scaleX = width / (distMax + 0.1 + 0.25);
            double strafeMin = Utl.min(aprilTagStats.get(APRIL_STRAFE_INDEX).min,
                    aprilTagStats.get(SPEED_CACHE_STRAFE_INDEX).min);
            double strafeMax = Utl.max(aprilTagStats.get(APRIL_STRAFE_INDEX).max,
                    aprilTagStats.get(SPEED_CACHE_STRAFE_INDEX).max);
            double scaleY = height / ((strafeMax - strafeMin) + 0.5);
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
            transformPath(aprilTagPath);
            transformPath(filteredAprilTagPath);
            transformPath(swervePath);
            transformPath(speedCachePath);
        }
    }

    void transformPath(List<PathPoint> path) {
        for (PathPoint pathPt : path) {
            pathPt.transform(drawXfm);
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
     * Paint the panel, which in the double buffer context means:
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
     * display the content of the back buffer as the screen buffer - displaying everyting porinted here..
     *
     * @param g The graphics description for the back buffer.
     */
    public void pkgPaintBuffer(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setPaint(Color.WHITE);


        // draw the target
        Point2D.Double leftPt = (Point2D.Double) drawXfm.transform(
                new Point2D.Double(-0.1016, 0.0), null);
        Point2D.Double rightPt = (Point2D.Double) drawXfm.transform(
                new Point2D.Double(0.1016, 0.0), null);
        g2d.drawLine((int) leftPt.getX(), (int) leftPt.getY(),
                (int) rightPt.getX(), (int) rightPt.getY());

        // draw the april path, the filtered path, and the path projected from start.
        paintPath(g2d, aprilTagPath, Color.WHITE);
        paintPath(g2d, filteredAprilTagPath, Color.YELLOW);
        paintPath(g2d, swervePath, Color.CYAN);
        paintPath(g2d, speedCachePath, Color.ORANGE);

        // draw the mouse and tracking info
        // TODO: handle repositioning the text when the cursor gets to the edge of
        //  the window.
        if (null != mouse) {
            g2d.setPaint(Color.WHITE);
            Point2D screenMouse = drawXfm.transform(mouse, null);
            g2d.drawString(
                    String.format(" (%.4f,%.4f)", mouse.getX(), mouse.getY()),
                    (int) screenMouse.getX(), (int) screenMouse.getY());
        }
    }

    void paintPath(Graphics2D g2d, List<PathPoint> path, Color color) {
        g2d.setPaint(color);
        Point2D.Double lastPt = null;
        for (PathPoint pathPt : path) {
            Point2D.Double thisPt = pathPt.screenPt;
            g2d.drawOval((int) thisPt.getX() - 2, (int) thisPt.getY() - 2, 4, 4);
            if (null != lastPt) {
                g2d.drawLine((int) lastPt.getX(), (int) lastPt.getY(),
                        (int) thisPt.getX(), (int) thisPt.getY());
            }
            lastPt = thisPt;
        }
    }

    PathPoint hitTestPath(List<PathPoint> path, Point2D.Double pt, double tolerance) {
        for (PathPoint pathPt : path) {
            Point2D.Double thisPt = pathPt.screenPt;
            if (Utl.inTolerance(thisPt.getX(), pt.getX(), tolerance) &&
                    Utl.inTolerance(thisPt.getY(), pt.getY(), tolerance)) {
                return pathPt;
            }
        }
        return null;
    }

}
