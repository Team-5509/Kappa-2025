package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilDrive {
    private static final double DISTANCE_CLOSE_ENOUGH = 0.1; //in meters, TODO currently not calibrated
    private static final double ORBIT_CLOSE_ENOUGH = 0.122173; //in radians, TODO currently not calibrated
    private static final double SPIN_CLOSE_ENOUGH = 0.122173; //in radians, TODO currently not calibrated

    private static final double CAMERA_OFFSET = 1.0041; //in radians above level, TODO currently not verified
    private static final double CAMERA_HEIGHT = 0.56; //in meters above ground, TODO currently not verified

    /* double[][] TAG_INFO is a 2D array that holds the tag configs.
     * TAG_INFO[tagID - 1] gives a double[] with the information for each tag.
     * The values for each index in the array TAG_INFO[tagID - 1] are as follows:
     * 
     * 0: Height of tag off ground, in meters
     * 1: Desired distance to tag, in meters
     * 2: Desired angle from tag to robot in radians, such that straight on is 0 and moving left decreases the value
     * 3: Desired rotation of robot in radians, such that facing the tag is 0 and turning left is positive
     * 
     * Indexes 1 and 2 are r and theta in polar coordinates, respectively.
    */
    private static final double[][] TAG_INFO = { //TODO no info added yet
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
        {1, 1, 0, 0},
    };
    private static final PhotonCamera CAMERA = new PhotonCamera("center"); //TODO not tested yet

    /* As I do not have all the variables I need yet, I have left placeholders.
     *
     * Measured by camera:
     * tagRotation - the percieved yaw of the tag in radians, standing to left is -, straight on is 1/2 pi
     * tagAngle - the change in pitch to make the camera centered on the tag, in radians
     * tagX - the yaw of the robot in radians, tag to right of camera is positive, straight on is 0
     * tagID - the ID of the tag
     * 
     * Constant:
     * cameraOffset - the pitch of the camera in radians, up is +, level with ground is 0
     * cameraHeight - the height of the camera in meters
    */

    private static PhotonTrackedTarget getTarget() {
        PhotonPipelineResult p = CAMERA.getLatestResult();
        if (p.hasTargets()) { return p.getBestTarget(); }
        return null;
    }

    private static double degToRad(double angle) {
        return (angle * (Math.PI / 180));
    }

    /*public static double getLeftX() { //assuming positive return value moves right
        throw new Exception("Need to be able to get values, code disabled by developer.");
        PhotonTrackedTarget target = getTarget();
        if (target != null) {
            if (Math.abs(degToRad(target.getSkew()) - TAG_INFO[target.getFiducialId() - 1][2]) < ORBIT_CLOSE_ENOUGH) { return 0; }
            else if (degToRad(target.getSkew()) < TAG_INFO[target.getFiducialId() - 1][2]) { return (-1 * Math.abs(degToRad(target.getSkew()) - TAG_INFO[target.getFiducialId() - 1][2])); }
            else if (degToRad(target.getSkew()) > TAG_INFO[target.getFiducialId() - 1][2]) { return (1 * Math.abs(degToRad(target.getSkew()) - TAG_INFO[target.getFiducialId() - 1][2])); }
            else { return 0; }
        }
        else { return 0; }
    }*/

    public static double getLeftY() { //assuming positive return value moves forwards TODO verify that guess
        PhotonTrackedTarget target = getTarget();
        if (/*getLeftX() == 0 && */target != null) {
            double distance = (TAG_INFO[target.getFiducialId() - 1][0] - CAMERA_HEIGHT) / Math.tan(degToRad(target.getPitch()) + CAMERA_OFFSET);
            if (Math.abs(distance - TAG_INFO[target.getFiducialId() - 1][1]) < DISTANCE_CLOSE_ENOUGH) { return 0; }
            else if (distance > TAG_INFO[target.getFiducialId() - 1][1]) { return (1 * Math.abs(distance - TAG_INFO[target.getFiducialId() - 1][1])); }
            else if (distance < TAG_INFO[target.getFiducialId() - 1][1]) { return (-1 * Math.abs(distance - TAG_INFO[target.getFiducialId() - 1][1])); }
            else { return 0; }
        }
        else { return 0; }
    }

    public static double getRightX() { //assuming positive return value turns right TODO verify that guess
        PhotonTrackedTarget target = getTarget();
        if (target != null) {
            if (Math.abs(degToRad(target.getYaw()) - TAG_INFO[target.getFiducialId() - 1][3]) < SPIN_CLOSE_ENOUGH) { return 0; }
            else if (degToRad(target.getYaw()) > TAG_INFO[target.getFiducialId() - 1][3]) { return (1 * Math.abs(degToRad(target.getYaw()) - TAG_INFO[target.getFiducialId() - 1][3])); }
            else if (degToRad(target.getYaw()) < TAG_INFO[target.getFiducialId() - 1][3]) { return (-1 * Math.abs(degToRad(target.getYaw()) - TAG_INFO[target.getFiducialId() - 1][3])); }
            else { return 0; }
        }
        else { return 0; }
    }
    
}
