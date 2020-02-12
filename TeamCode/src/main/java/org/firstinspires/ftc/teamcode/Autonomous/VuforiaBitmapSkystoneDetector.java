package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.renderscript.Int2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Helpers.Vector2;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * <p>
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class VuforiaBitmapSkystoneDetector {
    private static final String VUFORIA_KEY = "AQMfl/L/////AAABmTblKFiFfUXdnoB7Ocz4UQNgHjSNJaBwlaDm9EpX0UI5ISx2EH+5IoEmxxd/FG8c31He17kM5vtS0jyAoD2ev5mXBiITmx4N8AduU/iAw/XMC5MiEB1YBgw5oSO1qd4jvCOgbzy/HcOpN3KoVVnYqKhTLc8n6/IIFGy+qyF7b8WkzscJpybOSAT5wtaZumdBu0K3lHV6n+fqGJDMvkQ5xrCS6HiBtpZScAoekd7iP3IxUik2rMFq5hqMsOYW+qlxKp0cj+x4K9CIOYEP4xZsCBt66UxtDSiNqaiC1DyONtFz4oHJf/4J5aYRjMNwC2BpsVJ/R91WIcC0H0dpP9gtL/09J0bIMjm3plo+ac+OM0H3";

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName camera = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    //Define the skystone state
    public SkystoneState lastState = SkystoneState.CENTER;

    //Where the stones are relitive to the front of the bot
    public enum SkystoneState {
        PORT,
        CENTER,
        STARBOARD
    }

    public void Start(OpMode op) {
        //Find the camera
        camera = op.hardwareMap.get(WebcamName.class, RobotConfiguration.camera);
        op.telemetry.addData("Camera Found.", "");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        op.telemetry.addData("Camera ID Found.", "");

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = camera;
        op.telemetry.addData("Camera assigned.", "");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        op.telemetry.addData("Vuforia instansiated.", "");

        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();
    }

    VuforiaLocalizer.CloseableFrame frame;

    Bitmap image;


    public void Update(OpMode op) {
        try {
            //Fetch the latest frame
            frame = vuforia.getFrameQueue().take();

            //Convert it to a bitmap
            image = vuforia.convertFrameToBitmap(frame);


//            op.telemetry.addData("yeeet", "");
            op.telemetry.addData("first third color ", getAlphaFromBitmap(image, 0, 1));
//            op.telemetry.addData("second third color ", getColorFromBitmap(image, (1 / 3) * image.getWidth(), 1 / 3));
//            op.telemetry.addData("third third color ", getColorFromBitmap(image, 2 / 3 * image.getWidth(), 1 / 3));
            op.telemetry.addData("image x", image.getWidth());
            op.telemetry.addData("image y", image.getHeight());
            op.telemetry.addData("image y", image.getHeight());
            op.telemetry.update();

            frame.close();
        } catch (InterruptedException e) {
            op.telemetry.addData("yoiiiiink", "");
            op.telemetry.update();
        }
    }

    Int2 imageScale = new Int2(0, 0);

    //Returns an average color from a portion of a bitmap
    private long getAlphaFromBitmap(Bitmap bitmap, int xPixelOffset, double xRange) {

        imageScale.x = bitmap.getWidth();
        imageScale.y = bitmap.getHeight();

        int xPixelCount = (int) (xRange * imageScale.x);

        int color = 0;

        long totalAlpha = 0L;


        for (int x = xPixelOffset; x < xPixelCount; x++) {

            for (int y = 0; y < imageScale.y; y++) {
                color = bitmap.getPixel(x, y);

                totalAlpha += Color.red(color) + Color.blue(color) + Color.green(color);

            }
        }

//        redTotal /= xPixelCount * imageScale.y;
//        greenTotal /= xPixelCount * imageScale.y;
//        blueTotal /= xPixelCount * imageScale.y;


//        bitmap.getPixel()
//        return toColor(redTotal, greenTotal, blueTotal);
        return totalAlpha;
    }

    public int toColor(int r, int g, int b) {
        int rgb = r;
        rgb = (rgb << 8) + g;
        rgb = (rgb << 8) + b;
        return rgb;
    }

}
