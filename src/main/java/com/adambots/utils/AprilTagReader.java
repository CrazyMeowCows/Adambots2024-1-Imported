package com.adambots.utils;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Detect and read April Tags. Here's how to use it:
 * 
 * try (AprilTagReader tagReader = new AprilTagReader()){
 *      // Get the UsbCamera from CameraServer
 *      UsbCamera camera = CameraServer.startAutomaticCapture();
 *      // Set the resolution
 *      camera.setResolution(640, 480);
 * 
 *      // Get a CvSink. This will capture Mats from the camera
 *      CvSink cvSink = CameraServer.getVideo();
 * 
 *      var mat = new Mat();
 *      if (cvSink.grabFrame(mat) != 0){
 *          tagReader.detect(mat);
 * 
 *          System.out.println("Number of tags detected: " + tagReader.count());
 * 
 *          var id = tagReader.getId(0);
 *          var id2 = tagReader.getId(1);
 * 
 *          var pose = tagReader.getPose(0);
 *          var rot = pose.getRotation();
 *          Log.infoF("x=%d, y=%d, z=%d, rX=%d, rY=%d, rZ=%d\n", pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ());
 *          
 *          var outlineColor = new Scalar(0, 255, 0);
 *          Point[] corners = tagReader.getBoundingBox(0);
 *          for (var i = 0; i <= 3; i++) {
 *              var j = (i + 1) % 4; 
 *              Imgproc.line(mat, corners[i], corners[j], outlineColor, 2);
 *          }
 *      }
 * }
 */
public class AprilTagReader implements AutoCloseable {

    private final AprilTagDetector detector = new AprilTagDetector();
    private AprilTagDetection[] detections;
    private AprilTagPoseEstimator estimator;

    public AprilTagReader() {
        detector.addFamily("tag16h5", 0);

        var poseEstConfig = new AprilTagPoseEstimator.Config(
                0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);

        estimator = new AprilTagPoseEstimator(poseEstConfig);
    }

    public void detect(Mat frame) {
        var grayMat = new Mat();

        // convert to grayscale
        Imgproc.cvtColor(frame, grayMat, Imgproc.COLOR_RGB2GRAY);

        detections = detector.detect(grayMat);
    }

    public int count() {
        if (detections == null)
            return 0;

        return detections.length;
    }

    public int getId(int tagNum) {
        if (hasNotExtracted(tagNum))
            return -1;

        return detections[tagNum].getId();
    }

    private boolean hasNotExtracted(int tagNum) {
        return detections == null || (detections != null && tagNum >= detections.length);
    }

    public Transform3d getPose(int tagNum) {
        if (hasNotExtracted(tagNum))
            return null;

        return estimator.estimate(detections[tagNum]);
    }

    public Point[] getBoundingBox(int tagNum){
        Point[] cornerPoints = new Point[4];

        if (hasNotExtracted(tagNum))
            return cornerPoints;

        var detection = detections[tagNum];

        for (var i = 0; i <= 3; i++) {
            var j = (i + 1) % 4;
            cornerPoints[i] = new Point(detection.getCornerX(i), detection.getCornerY(i));
            cornerPoints[j] = new Point(detection.getCornerX(j), detection.getCornerY(j));
        }

        return cornerPoints;
    }

    public double getCenterX(int tagNum){
        if (hasNotExtracted(tagNum))
            return -1;
        
        return detections[tagNum].getCenterX();
    }

    public double getCenterY(int tagNum){
        if (hasNotExtracted(tagNum))
            return -1;
        
        return detections[tagNum].getCenterY();
    }

    public AprilTagDetection getDetection(int tagNum){
        if (hasNotExtracted(tagNum))
            return null;

        return detections[tagNum];
    }

    /**
     * Indicates if the robot is centered based on the April Tag based on its Y axis. 
     * If the robot is centered, it returns 0.
     * If not it returns -1 or +1. Depending on this turn left or right until it is centered (returns 0)
     * @param tagNum
     * @return
     * 
     */
    public int AlignmentCheck(int tagNum){
        if (hasNotExtracted(tagNum))
            return 0;
        
        var detection = getDetection(tagNum);
        Transform3d pose = estimator.estimate(detection);

        Rotation3d rot = pose.getRotation();

        if (rot.getY() > 0.4)
            return -1;
        
        if (rot.getY() < -0.4)
            return +1;
        
        return 0;
    }

    @Override
    public void close() throws Exception {
        detector.close();

    }
}
