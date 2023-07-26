package org.a05annex.frc.subsystems;

import org.a05annex.util.Utl;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
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

    List<List<Double>> aprilTagData;
    List<TuneSpeedCache.ColumnStats> aprilTagStats;
    List<List<Double>> swerveData;
    List<TuneSpeedCache.ColumnStats> swerveStats;
    SpeedCachedSwerve speedCachedSwerve;

    // the back buffer to support double buffering
    private int bufferWidth;
    private int bufferHeight;
    private Image bufferImage;
    private Graphics bufferGraphics;

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
            ((TuneSpeedCacheCanvas)comp).setDisplayGeometry();
        }
    }

    TuneSpeedCacheCanvas(@NotNull GraphicsConfiguration gc,
                         @NotNull List<List<Double>> aprilTagData, @NotNull List<TuneSpeedCache.ColumnStats> aprilTagStats,
                         @NotNull List<List<Double>> swerveData, @NotNull List<TuneSpeedCache.ColumnStats> swerveStats,
                         @NotNull SpeedCachedSwerve speedCachedSwerve)
    {
        super(gc);

        this.aprilTagData = aprilTagData;
        this.aprilTagStats = aprilTagStats;
        this.swerveData = swerveData;
        this.swerveStats = swerveStats;
        this.speedCachedSwerve = speedCachedSwerve;

        //------------------------------------------------------------------
        // setup the window for drawing the april target and the paths around it
        //------------------------------------------------------------------
        setBackground(Color.BLACK);
        addComponentListener(new ComponentHandler());
        setDisplayGeometry();

    }

    @Override
    public void actionPerformed(ActionEvent e) {

    }

    /**
     * Here we look at the geometry of the april tag and the april tag and predicted paths to guess at a good
     * scaling and offset that will display the geometry with room for trying modifications to the future path
     * prediction algorithms to see how they affect the projected path
     */
    public void setDisplayGeometry() {
        float width = this.getWidth();
        float height = this.getHeight();
        // The first step here - the april target is near the top of the screen at 0.0, positive distance is
        // down, positive strafe is to the right. We need a size of the display - assume starting Y at -0.25
        // (behind the target), and max at max (april dist or predicted dist) + 1m; for X, use min-max
        // (april dist or predicted dist) + 2m (1m on each side) - this let's us compute the scale factor.
        double distMax = Utl.max(aprilTagStats.get(TuneSpeedCache.APRIL_DISTANCE_INDEX).max,
                aprilTagStats.get(TuneSpeedCache.SPEED_CACHE_DISTANCE_INDEX).max);
        double scaleX = width / (distMax + 0.25);
        double strafeMin = Utl.min(aprilTagStats.get(APRIL_STRAFE_INDEX).min,
                aprilTagStats.get(SPEED_CACHE_STRAFE_INDEX).min);
        double strafeMax = Utl.max(aprilTagStats.get(APRIL_STRAFE_INDEX).max,
                aprilTagStats.get(SPEED_CACHE_STRAFE_INDEX).max);
        double scaleY = height / ((strafeMax - strafeMin) + 2.0);
        scale = Math.min(scaleX, scaleY);
        // second step - get the translation that moves the april target to 0.25m from the top of the window.
        // OK, what is happening here?? Magic - well, not really. The width/2.0 and height/2.0 bits of the
        // m02 and m12 shift the origin to the center of the screen window. For the default competition field
        // this is great because we adopted 0,0 as center field.
        drawXfm = new AffineTransform(scale, 0.0f, 0.0f, scale,
                (width / 2.0) -
                        (scale * (((strafeMax - strafeMin) / 2.0) + strafeMin)),
                (height / 2.0) +
                        (-scale * (((distMax - 0.25) / 2.0) + 0.25)));
        mouseXfm = new AffineTransform(drawXfm);
        try {
            mouseXfm.invert();
        } catch (NoninvertibleTransformException ex) {
            System.out.println("  -- can't invert draw transform");
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
     * Paint the current field, robot, and path to the back buffer.
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

        // draw the april path
        Point2D.Double lastPt = null;
        for (List<Double> aprilTag : aprilTagData) {
            Point2D.Double thisPt = (Point2D.Double) drawXfm.transform(
                    new Point2D.Double(aprilTag.get(APRIL_STRAFE_INDEX),
                            aprilTag.get(APRIL_DISTANCE_INDEX)), null);
            if (null != lastPt) {
                g2d.drawLine((int) lastPt.getX(), (int) lastPt.getY(),
                        (int) thisPt.getX(), (int) thisPt.getY());
            }
            lastPt = thisPt;
        }

        // draw the path projected by the cache from the first april target sighting
        g2d.setPaint(Color.CYAN);
        lastPt = null;
        for (List<Double> aprilTag : aprilTagData) {
            Point2D.Double thisPt = (Point2D.Double) drawXfm.transform(
                    new Point2D.Double(aprilTag.get(SPEED_CACHE_STRAFE_INDEX),
                            aprilTag.get(SPEED_CACHE_DISTANCE_INDEX)), null);
            if (null != lastPt) {
                g2d.drawLine((int) lastPt.getX(), (int) lastPt.getY(),
                        (int) thisPt.getX(), (int) thisPt.getY());
            }
            lastPt = thisPt;
        }

    }


}
