package org.a05annex.frc;

import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * This is a test of the {@link org.a05annex.frc.NavX.HeadingInfo} functionality. The
 * {@link org.a05annex.frc.NavX.HeadingInfo} is primarily a data structure for returning current heading/displacement
 * information. However, it does have functionality to determine from current heading the closest up-field, down-field,
 * of either heading for the robot. Additionally, there is functionality related to whether the
 */
@Suite
public class TestHeadingInfo {

    NavX.HeadingInfo newHeadingInfo(double headingDegrees, double fusedHeadingDegrees,
                                    float displavementX, float diaplacementY) {
        return new NavX.HeadingInfo(new AngleD(AngleUnit.DEGREES, headingDegrees),
                null, true, new AngleD(AngleUnit.DEGREES, fusedHeadingDegrees),
                0.0f, 0.0f);
    }

    /**
     *
     */
    @Test
    @DisplayName("Test getting the closest down-field heading")
    void test_closestDownField() {
        // test no full rotations
        assertEquals(0.0, newHeadingInfo(5.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(0.0, newHeadingInfo(-5.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(0.0, newHeadingInfo(175.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(0.0, newHeadingInfo(-175.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        // test positive rotations
        assertEquals(720.0, newHeadingInfo(725.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(720.0, newHeadingInfo(715.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(720.0, newHeadingInfo(895.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(720.0, newHeadingInfo(545.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        // test negative rotations
        assertEquals(-720.0, newHeadingInfo(-725.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-715.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-895.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-545.0, 0.0, 0.0f, 0.0f).
                getClosestDownField().getDegrees());
    }

    @Test
    @DisplayName("Test getting the closest up-field heading")
    void test_closestUpField() {
        // test no full rotations
        assertEquals(180.0, newHeadingInfo(5.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(-180.0, newHeadingInfo(-5.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(180.0, newHeadingInfo(175.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(-180.0, newHeadingInfo(-175.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        // test positive rotations
        assertEquals(900.0, newHeadingInfo(725.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(540.0, newHeadingInfo(715.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(900.0, newHeadingInfo(895.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(540.0, newHeadingInfo(545.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        // test negative rotations
        assertEquals(-900.0, newHeadingInfo(-725.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(-540.0, newHeadingInfo(-715.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(-900.0, newHeadingInfo(-895.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
        assertEquals(-540.0, newHeadingInfo(-545.0, 0.0, 0.0f, 0.0f).
                getClosestUpField().getDegrees());
    }

    @Test
    @DisplayName("Test getting the closest up-field heading")
    void test_closestDownOrUpField() {
        // test no full rotations
        assertEquals(0.0, newHeadingInfo(5.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(0.0, newHeadingInfo(-5.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(180.0, newHeadingInfo(175.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(-180.0, newHeadingInfo(-175.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        // test positive rotations
        assertEquals(720.0, newHeadingInfo(725.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(720.0, newHeadingInfo(715.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(900.0, newHeadingInfo(895.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(540.0, newHeadingInfo(545.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        // test negative rotations
        assertEquals(-720.0, newHeadingInfo(-725.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-715.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(-900.0, newHeadingInfo(-895.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
        assertEquals(-540.0, newHeadingInfo(-545.0, 0.0, 0.0f, 0.0f).
                getClosestDownOrUpField().getDegrees());
    }

}
