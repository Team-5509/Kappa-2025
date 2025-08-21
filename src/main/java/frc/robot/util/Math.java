package frc.robot.util;

public class Math {

    /**
     * Converts inches to meters.
     *
     * @param meters the distance in inches
     * @return the distance in meters
     */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    
    /**
     * Converts inches to meters.
     */
    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    /**
     * Converts inches to meters.
     * This is an alias for inchesToMeters.
     * @param inches
     * @return the distance in meters
     */
    public static double in2m(double inches) {
        return inchesToMeters(inches);
    }


    
}
