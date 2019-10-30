package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePositionChange;
import org.firstinspires.ftc.teamcode.HardwareMecanum;


/**
 * Created by Sarthak on 10/4/2019.
 * Gutted and made worse by Bobby on 10/14/2019
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // 20:1 encoder ticks
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     SLIPPAGE_FACTOR         = 1.0 ; // CHANGE
    static final double     LR                      = 16.0;
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415*SLIPPAGE_FACTOR);


    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY


    OdometryGlobalCoordinatePositionChange globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        HardwareMecanum robot = new HardwareMecanum();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePositionChange(left_front, right_front, left_back, right_back, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //globalPositionUpdate.reverseRightEncoder();
        //globalPositionUpdate.reverseNormalEncoder();

        //Make path below this
        goToPosition(0*COUNTS_PER_INCH,0*COUNTS_PER_INCH,.5*COUNTS_PER_INCH,
                90*COUNTS_PER_INCH,.4*COUNTS_PER_INCH);


        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Front left encoder position",robot.left_front.getCurrentPosition());
            telemetry.addData("Front right encoder position",robot.right_front.getCurrentPosition());
            telemetry.addData("Back left encoder position", robot.left_back.getCurrentPosition());
            telemetry.addData("Back right encoder position", robot.right_back.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotSpeed, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToXTarget = targetXPosition-globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition-globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);

        while(opModeIsActive()&&distance>allowableDistanceError){
            distanceToXTarget = targetXPosition-globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition-globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToYTarget,distanceToXTarget));

//use the below 3 variables to send power to motors
            double robot_movement_x_component = calculateX(robotMovementAngle,robotSpeed);
            double robot_movement_y_component = calculateY(robotMovementAngle,robotSpeed);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();


        }

     /**   This code is from positionFinder that I wrote.
      * General idea is to translate into this class's stuff but I don't like it. Want more updated values?
      *
      * double d_x = goalX-x;
        double d_y = goalY-y;
        double pythag = Math.pow(d_x,2)+Math.pow(d_y,2);
        distance = Math.sqrt(pythag);
        double diffAngle=Math.asin(d_y/distance);
        if(d_y>0){
            if(d_x>0){
            }
            else
                diffAngle = 180-diffAngle;
        }
        else{
            if(d_x>0){
                diffAngle = 360-diffAngle;
            }
            else
                diffAngle = 180+diffAngle;

        }**/
    }
//I have no idea wtf the method below does but I'm guessing it's important so for now it stays

    /**
    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
*/
    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
}
