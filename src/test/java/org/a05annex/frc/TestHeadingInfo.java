package org.a05annex.frc;

import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import static org.junit.jupiter.api.Assertions.assertEquals;

@Suite
public class TestHeadingInfo {

    NavX.HeadingInfo newHeadingInfo(double degrees) {
        return new NavX.HeadingInfo(new AngleD(AngleUnit.DEGREES,degrees),
                new AngleD(AngleUnit.DEGREES,degrees), true);
    }

    /**
     *
     */
    @Test
    @DisplayName("Test getting the closest down-field heading")
    void test_closestDownField() {
        // test no full rotations
        assertEquals(0.0, newHeadingInfo(5.0).getClosestDownField().getDegrees());
        assertEquals(0.0, newHeadingInfo(-5.0).getClosestDownField().getDegrees());
        assertEquals(0.0, newHeadingInfo(175.0).getClosestDownField().getDegrees());
        assertEquals(0.0, newHeadingInfo(-175.0).getClosestDownField().getDegrees());
        // test positive rotations
        assertEquals(720.0, newHeadingInfo(725.0).getClosestDownField().getDegrees());
        assertEquals(720.0, newHeadingInfo(715.0).getClosestDownField().getDegrees());
        assertEquals(720.0, newHeadingInfo(895.0).getClosestDownField().getDegrees());
        assertEquals(720.0, newHeadingInfo(545.0).getClosestDownField().getDegrees());
        // test negative rotations
        assertEquals(-720.0, newHeadingInfo(-725.0).getClosestDownField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-715.0).getClosestDownField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-895.0).getClosestDownField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-545.0).getClosestDownField().getDegrees());
    }

    @Test
    @DisplayName("Test getting the closest up-field heading")
    void test_closestUpField() {
        // test no full rotations
        assertEquals(180.0, newHeadingInfo(5.0).getClosestUpField().getDegrees());
        assertEquals(-180.0, newHeadingInfo(-5.0).getClosestUpField().getDegrees());
        assertEquals(180.0, newHeadingInfo(175.0).getClosestUpField().getDegrees());
        assertEquals(-180.0, newHeadingInfo(-175.0).getClosestUpField().getDegrees());
        // test positive rotations
        assertEquals(900.0, newHeadingInfo(725.0).getClosestUpField().getDegrees());
        assertEquals(540.0, newHeadingInfo(715.0).getClosestUpField().getDegrees());
        assertEquals(900.0, newHeadingInfo(895.0).getClosestUpField().getDegrees());
        assertEquals(540.0, newHeadingInfo(545.0).getClosestUpField().getDegrees());
        // test negative rotations
        assertEquals(-900.0, newHeadingInfo(-725.0).getClosestUpField().getDegrees());
        assertEquals(-540.0, newHeadingInfo(-715.0).getClosestUpField().getDegrees());
        assertEquals(-900.0, newHeadingInfo(-895.0).getClosestUpField().getDegrees());
        assertEquals(-540.0, newHeadingInfo(-545.0).getClosestUpField().getDegrees());
    }

    @Test
    @DisplayName("Test getting the closest up-field heading")
    void test_closestDownOrUpField() {
        // test no full rotations
        assertEquals(0.0, newHeadingInfo(5.0).getClosestDownOrUpField().getDegrees());
        assertEquals(0.0, newHeadingInfo(-5.0).getClosestDownOrUpField().getDegrees());
        assertEquals(180.0, newHeadingInfo(175.0).getClosestDownOrUpField().getDegrees());
        assertEquals(-180.0, newHeadingInfo(-175.0).getClosestDownOrUpField().getDegrees());
        // test positive rotations
        assertEquals(720.0, newHeadingInfo(725.0).getClosestDownOrUpField().getDegrees());
        assertEquals(720.0, newHeadingInfo(715.0).getClosestDownOrUpField().getDegrees());
        assertEquals(900.0, newHeadingInfo(895.0).getClosestDownOrUpField().getDegrees());
        assertEquals(540.0, newHeadingInfo(545.0).getClosestDownOrUpField().getDegrees());
        // test negative rotations
        assertEquals(-720.0, newHeadingInfo(-725.0).getClosestDownOrUpField().getDegrees());
        assertEquals(-720.0, newHeadingInfo(-715.0).getClosestDownOrUpField().getDegrees());
        assertEquals(-900.0, newHeadingInfo(-895.0).getClosestDownOrUpField().getDegrees());
        assertEquals(-540.0, newHeadingInfo(-545.0).getClosestDownOrUpField().getDegrees());
    }

}
