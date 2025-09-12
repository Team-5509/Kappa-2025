package frc.robot.util;

public class PrettyDouble {


/** */
    public static String prettyDouble(double value, int precision) {
        return Math.round(value * Math.pow(10, precision)) / Math.pow(10, precision) + "";
      }
    
      public static String prettyDouble(double value) {
        return prettyDouble(value, 2);
      }
    
}
