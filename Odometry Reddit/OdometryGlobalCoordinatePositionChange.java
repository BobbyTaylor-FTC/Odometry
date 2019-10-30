package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePositionChange implements Runnable{
    HardwareMecanum robot = new HardwareMecanum();
    DcMotor left_front, right_front, right_back,left_back;
    //Thead run condition
    private boolean isRunning = true;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // 20:1 encoder ticks
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     SLIPPAGE_FACTOR         = 1.0 ; // CHANGE
    static final double     LR                      = 16.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415*SLIPPAGE_FACTOR);
    //Position variables used for storage and calculations

    double encoderFrontLeft = 0, encoderFrontRight = 0, encoderBackLeft = 0, encoderBackRight = 0;
    double lastEncoderFrontLeft = 0, lastEncoderFrontRight = 0, lastEncoderBackLeft = 0, lastEncoderBackRight = 0;
    double delta_y_r, delta_x_r, delta_theta_r, delta_y, delta_x, delta_theta, theta;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;


    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;


    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
/**
    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
*/
    /**
     * Constructor for GlobalCoordinatePosition Thread
     * The stuff below is from the original build, kept because maybe useful? Probably not but wth.
     * //@param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * //@param verticalEncoderRight right odometry encoder, facing the vertical direction
     * //@param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePositionChange(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back, double COUNTS_PER_INCH, int threadSleepDelay){
        this.left_front = left_front;
        this.right_front = right_front;
        this.right_back = right_back;
        this.left_back = left_back;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){

      /**
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
       */
        encoderFrontLeft = robot.left_front.getCurrentPosition();
        encoderFrontRight = robot.right_front.getCurrentPosition();
        encoderBackLeft = robot.left_back.getCurrentPosition();
        encoderBackRight = robot.right_back.getCurrentPosition();
        double deltaFrontLeft=(encoderFrontLeft-lastEncoderFrontLeft)*COUNTS_PER_INCH;
        double deltaBackLeft=(encoderBackLeft-lastEncoderBackLeft)*COUNTS_PER_INCH;
        double deltaBackRight=(encoderBackRight-lastEncoderBackRight)*COUNTS_PER_INCH;
        double deltaFrontRight=(encoderFrontRight-lastEncoderFrontRight)*COUNTS_PER_INCH;

        lastEncoderBackLeft = encoderBackLeft;
        lastEncoderBackRight = encoderBackRight;
        lastEncoderFrontLeft = encoderFrontLeft;
        lastEncoderFrontRight = encoderFrontRight;

        delta_y_r = (deltaFrontRight+deltaFrontLeft+deltaBackLeft+deltaBackRight)/4.0;
        delta_x_r = (-deltaFrontRight+deltaFrontLeft-deltaBackLeft+deltaBackRight)/4.0;
        delta_theta_r = (deltaFrontRight-deltaFrontLeft-deltaBackLeft+deltaBackRight)/(4.0*LR); //lr is track width. Could probably use gyro

        delta_y = (delta_x_r*Math.sin(-theta)+delta_y_r*Math.cos(-theta));
        delta_x = (delta_x_r*Math.cos(-theta)-delta_y_r*Math.sin(-theta));
        delta_theta = delta_theta_r;
        robotGlobalXCoordinatePosition += delta_x;
        robotGlobalYCoordinatePosition += delta_y;
        robotOrientationRadians += delta_theta;
        if(theta>360){
            theta-=360;
        }
        if(theta<0){
            theta+=360;
        }
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){
        return robotGlobalXCoordinatePosition;
    }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){
        return robotGlobalYCoordinatePosition;
    }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){
        return Math.toDegrees(robotOrientationRadians) % 360;
    }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }
/**    public void reverseLeftEncoder(){
 if(verticalLeftEncoderPositionMultiplier == 1){
 verticalLeftEncoderPositionMultiplier = -1;
 }else{
 verticalLeftEncoderPositionMultiplier = 1;
 }
 }

 public void reverseRightEncoder(){
 if(verticalRightEncoderPositionMultiplier == 1){
 verticalRightEncoderPositionMultiplier = -1;
 }else{
 verticalRightEncoderPositionMultiplier = 1;
 }
 }

 public void reverseNormalEncoder(){
 if(normalEncoderPositionMultiplier == 1){
 normalEncoderPositionMultiplier = -1;
 }else{
 normalEncoderPositionMultiplier = 1;
 }
 }

**/
    /**
     * Runs the thread
     */
    @Override
    public void run() {

        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
