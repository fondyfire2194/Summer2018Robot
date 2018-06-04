package org.usfirst.frc.team2194.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class AllCameras {
	public static UsbCamera cubeCamera;
	public static CubePipeline cubePipeline;
	private final Object imgLock = new Object();
	private VisionThread cubeVisionThread;
	private Thread visionThreadAddTargeting;

	public static final int IMG_WIDTH = 320;
	public static final int IMG_HEIGHT = 240;
	public static int cubeNumberImages;// amt of images KW
	public boolean cubeVisionTurnedOn = true;// looking for cubes in auto
	public static int cubeThreadCounter;
	private static int cubeImagesAllowed = 4;

	public static boolean cubeVisionTargetingOn = true;// show cubes with targeting lines

	public static Scalar yellow = new Scalar(60, 100, 100, 0);
	public static Scalar red = new Scalar(0, 0, 255, 0);
	public static Scalar green = new Scalar(0, 255, 0, 0);
	public static Scalar blue = new Scalar(255, 0, 0, 0);
	public int boxAddition = 4;
	public int xVisionTarget = 0;
	Rect[] rect = new Rect[cubeImagesAllowed - 1];
	public static int x;
	public static int y;
	public static int height;
	public static int width;
	private int largestArea;
	private int imageCounter;
	private int noImageCounter;
	private int imageStateCountLimit = 2;
	public int targetOffsetMultiplier = 5;
	public boolean visionTargetNotFound;

	public AllCameras() {

		cubeCamera = CameraServer.getInstance().startAutomaticCapture("CubeCam", 0);
		cubeCamera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		cubeCamera.setBrightness(50);

		cubeCamera.setFPS(30);
		cubeCamera.setExposureAuto();
		cubeCamera.setWhiteBalanceAuto();

		cubeVisionThread = new VisionThread(cubeCamera, new CubePipeline(), cubePipeline -> {

			cubeNumberImages = cubePipeline.filterContoursOutput().size();

			cubeThreadCounter += 1;

			while (!cubeVisionTurnedOn && !Thread.currentThread().isInterrupted())
				try {
					Thread.sleep(10000);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			if (cubeVisionTurnedOn) {
				cubeNumberImages = cubePipeline.filterContoursOutput().size();
				largestArea = 0;
				if (cubeNumberImages > 0 && cubeNumberImages <= (cubeImagesAllowed - 1)) {
					if (noImageCounter > 0)
						noImageCounter--;
					if (imageCounter < imageStateCountLimit)
						imageCounter++;
					for (int i = 0; i < cubeNumberImages; i++) {
						rect[i] = Imgproc.boundingRect(cubePipeline.filterContoursOutput().get(i));
						synchronized (imgLock) {
							/*
							 * get largest of multiple images if any to eliminate any small spurious false
							 * indications
							 * 
							 */
							if (rect[i].height * rect[i].width > largestArea) {
								height = rect[i].height;
								width = rect[i].width;
								largestArea = height * width;
								x = rect[i].x;
								y = rect[i].y;
							}
						}
					}
				} else {
					if (imageCounter > 0)
						imageCounter--;
					if (noImageCounter < imageStateCountLimit)
						noImageCounter++;
				}
			}
		}
		// }
		);
		cubeVisionThread.setDaemon(true);
		cubeVisionThread.start();

		visionThreadAddTargeting = new Thread(() -> {
			CvSink cvSink = CameraServer.getInstance().getVideo(cubeCamera);
			// // Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("CubeTarget", IMG_WIDTH, IMG_HEIGHT);
			// // Mats are very memory expensive. Lets reuse this Mat.

			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted() && cubeVisionTargetingOn) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Draw on the image

				// image vertical center line
				Imgproc.line(mat, new Point(IMG_WIDTH / 2, 0), new Point(IMG_WIDTH / 2, IMG_HEIGHT - 1), green, 2);

				// target line on cube(s)
				Imgproc.line(mat, new Point(getXOffsetTarget(), 0), new Point(getXOffsetTarget(), IMG_HEIGHT - 1), blue,
						2);

				int x1Val = 0;
				int y1Val = 0;
				int x2Val = 0;
				int y2Val = 0;

				if (cubeVisionTurnedOn) {

					// target box 1
					// make sure not to write outside screen
					x1Val = Range.ensure(getX(), 0, IMG_WIDTH - 1);
					y1Val = Range.ensure(getY(), 0, IMG_HEIGHT - 1);
					x2Val = Range.ensure(getX() + getWidth(), x1Val + 1, IMG_WIDTH - 1);
					y2Val = Range.ensure(getY() + getHeight(), y1Val + 1, IMG_HEIGHT - 1);

					Imgproc.rectangle(mat, new Point(x1Val, y1Val), new Point(x2Val, y2Val), red, 2);
					// target center

				}
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		if (cubeVisionTargetingOn) {
			visionThreadAddTargeting.setDaemon(true);
			visionThreadAddTargeting.start();
		}
	}

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	public int getWidth() {
		return width;
	}

	public int getHeight() {
		return height;
	}

	/*
	 * x value starts at left of screen and counts up to IMG_WIDTH For targeting
	 * purposes, make it +/- IMG_WIDTH/2 from screen center This is named XNormal
	 * For single cubes such as when turning from the scale or in teleop if the
	 * driver wants to use it, aim should be for target center. For the cube stack
	 * and for the line of cubes from the side, it will likely be better to aim for
	 * some percentage to the left or right of center. Left aim for right switch
	 * from center and left switch from left. Right aim for left switch from center
	 * and right switch from right. Offset is a value between 0 and 10. 0 is shift
	 * to left cube edge and 10 is the right cube edge
	 * 
	 */
	public int getXNormal() {
		return getX() - IMG_WIDTH / 2;
	}

	public int getXCenter() {
		return getXNormal() + (getWidth() / 2);
	}

	/*
	 * look into using this instead of getXCenter if needed defaults to same as
	 * getxCenter with multiplier set to 5
	 */
	public int getXOffsetTarget() {
		return getXNormal() + (getWidth() * targetOffsetMultiplier) / 10;
	}

	public int getYCenter() {
		return getY() + (getHeight() / 2);
	}

	public double getVisionPositionStraightComp() {
		Robot.driveTrainCanBus.visionStraightKp = Robot.prefs.getDouble("VisionStraightKp", 0.05);
		return Robot.driveTrainCanBus.visionStraightKp * getVisionError();
	}

	public int getVisionError() {
		return xVisionTarget - getXOffsetTarget();
	}

	public boolean targetsPresent() {
		return imageCounter == imageStateCountLimit && noImageCounter == 0;
	}

	public void updateStatus() {

		if (cubeVisionTurnedOn && cubeVisionThread.getState() == Thread.State.TIMED_WAITING)
			cubeVisionThread.interrupt();

		SD.putN("CubeThreadCtr", cubeThreadCounter);
		SD.putN0("Camera X", getX());
		SD.putN0("Camera Y", getY());
		SD.putN0("Camera Width", getWidth());
		SD.putN0("Camera Height", getHeight());
		SD.putN0("XPixelTarget", xVisionTarget);
		SD.putN0("Camera Error", getVisionError());
		SD.putN0("Cube NumImg", cubeNumberImages);
		SD.putN0("Camera X Normal", getXNormal());
		SmartDashboard.putBoolean("TargetsPresent", targetsPresent());
		SmartDashboard.putBoolean("Cube Vision Targetting On", cubeVisionTargetingOn);
		SmartDashboard.putBoolean("Cube Vision On", cubeVisionTurnedOn);

	}
}
