package net.na4zagin3.ev3linetracer;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.WindowManager;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import ev3command.ev3.Motor;
import ev3command.ev3.comm.AndroidComm;
import ev3command.ev3.comm.EV3Command;

import net.na4zagin3.ev3linetracer.util.AbstractPeriodicTask;


public class MainActivity extends Activity  implements CameraBridgeViewBase.CvCameraViewListener2, SensorEventListener {
    private BluetoothAdapter mBtAdapter = null;
    private Motor mLeftMotor = Motor.B;
    private Motor mRightMotor = Motor.C;

    private SensorManager sensorManager;
    private boolean isActivatedMagneticSensor;
    private boolean isActivatedAccelerationSensor;
    private boolean isActivatedGravitySensor;

    private final static boolean USE_OPENCV = true;
    private final static boolean USE_BLUETOOTH = true;
    private final static boolean USE_SENSORS = false;

    private final static int CONTROL_INTERVAL = 1000;
    final static int SPEED_CONST = 100;

    private double MoveX = 0;
    private double MoveY = 0;

    private PeriodicCarControl carControl = null;

    private void setupSensor() {
        sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
    }

    private void registerSensors() {
        super.onResume();

        // センサの取得
        List<Sensor> sensors = sensorManager.getSensorList(Sensor.TYPE_ALL);

        if (USE_SENSORS) {
            // センサマネージャへリスナーを登録(implements SensorEventListenerにより、thisで登録する)
            for (Sensor sensor : sensors) {

                if (sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI);
                    isActivatedMagneticSensor = true;
                }

                if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                    sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI);
                    isActivatedAccelerationSensor = true;
                }

                if (sensor.getType() == Sensor.TYPE_GRAVITY) {
                    sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI);
                    isActivatedGravitySensor = true;
                }
            }
        }
    }

    private void deregisterSensors() {
        if (isActivatedAccelerationSensor || isActivatedMagneticSensor || isActivatedGravitySensor) {
            sensorManager.unregisterListener(this);
            isActivatedAccelerationSensor = false;
            isActivatedMagneticSensor = false;
            isActivatedGravitySensor = false;
        }
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // TODO 後でセンサ精度の変更に対応する
    }

    private static final int MATRIX_SIZE = 16;
    // センサーの値
    float[] orientationValues   = new float[3];
    float[] magneticValues      = new float[3];
    float[] accelerometerValues = new float[3];
    float[] gravityValues       = new float[3];

    float[]  inR = new float[MATRIX_SIZE];
    float[] outR = new float[MATRIX_SIZE];
    float[]    I = new float[MATRIX_SIZE];

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) return;

        switch (event.sensor.getType()) {
            case Sensor.TYPE_MAGNETIC_FIELD:
                magneticValues = event.values.clone();
                break;
            case Sensor.TYPE_ACCELEROMETER:
                accelerometerValues = event.values.clone();
                break;
            case Sensor.TYPE_GRAVITY:
                gravityValues = event.values.clone();
                break;
        }

        if (magneticValues != null && accelerometerValues != null) {

            SensorManager.getRotationMatrix(inR, I, accelerometerValues, magneticValues);

            //Activityの表示が縦固定の場合。横向きになる場合、修正が必要です
            SensorManager.remapCoordinateSystem(inR, SensorManager.AXIS_X, SensorManager.AXIS_Z, outR);
            SensorManager.getOrientation(outR, orientationValues);

            Log.v("Orientation",
                    String.valueOf(Math.toDegrees(orientationValues[0])) + ", " + //Z軸方向,azimuth
                            String.valueOf(Math.toDegrees(orientationValues[1])) + ", " + //X軸方向,pitch
                            String.valueOf(Math.toDegrees(orientationValues[2])));       //Y軸方向,roll
        }
    }

    public class PeriodicCarControl extends AbstractPeriodicTask {
        public PeriodicCarControl(long period, boolean isDaemon) {
            super(period, isDaemon);
        }

        @Override
        protected void invokersMethod() {
            /*
            if (gravityValues == null) return;
            double x = gravityValues[0] / SensorManager.GRAVITY_EARTH;
            double y = gravityValues[1] / SensorManager.GRAVITY_EARTH;
            */
            setSpeed(MoveX, MoveY);
        }

    }

    private void setSpeed(double x, double y) {
        double l = y - x;
        double r = y + x;
        if (l > 1)  l = 1;
        if (l < -1) l = -1;
        if (r > 1)  r = 1;
        if (r < -1) r = -1;

        Log.v(TAG, "Motor: left:" + String.valueOf(l) + " right:" + String.valueOf(r));
        setMotorSpeed(l, r);
    }

    private void setMotorSpeed(double l, double r) {
        boolean forwardL = l > 0;
        boolean forwardR = r > 0;
        int moveL = Math.min(SPEED_CONST - 1, (int) (Math.abs(l) * (double)SPEED_CONST));
        int moveR = Math.min(SPEED_CONST - 1, (int) (Math.abs(r) * (double)SPEED_CONST));

        Log.v(TAG, "Move: " + String.valueOf(moveL) + ":" + String.valueOf(moveR));
        if (USE_BLUETOOTH) {
            mLeftMotor.setSpeed(moveL);
            mRightMotor.setSpeed(moveR);
            if (l > 0) {
                mLeftMotor.forward();
            } else if (l < 0) {
                mLeftMotor.backward();
            } else {
                mLeftMotor.stop();
            }

            if (r > 0) {
                mRightMotor.forward();
            } else if (r < 0) {
                mRightMotor.backward();
            } else {
                mRightMotor.stop();
            }
        }
    }

    private void setupCarControl() {
        carControl = new PeriodicCarControl(CONTROL_INTERVAL, false);
    }

    private void registerCarControl() {
        if (carControl != null) {
            carControl.execute();
        }
    }

    private void deregisterCarControl() {
        if (carControl != null) {
            carControl.cancel();
        }
    }

    protected String getEV3MACAddress() {
        return "00:16:53:43:E5:EC";
    }

    protected void failedCreatingConnection() {
    }


    protected void prepareCommunication(){
        if (USE_BLUETOOTH) {
// Get default adapter
            mBtAdapter = BluetoothAdapter.getDefaultAdapter();


// Get the device MAC address
            String address = getEV3MACAddress();
// Get the BluetoothDevice object
            BluetoothDevice device = mBtAdapter.getRemoteDevice(address);

            AndroidComm.getInstance().setDevice(device); // Set device

// Connect to EV3
            try {
                EV3Command.open();
            } catch (Exception e) {
                // This exception also occurs when this device hasn’t
                // finished paring
                failedCreatingConnection();
            }
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
        prepareCommunication();

        openCVOnCreate();
        setupSensor();
        setupCarControl();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        // Close the connection
        try {
            EV3Command.close();
        }
        catch (RuntimeException e) {
            Log.v(TAG, "EV3Command.close() failed");
        }

        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    private static final String TAG = "EV3LineTracer";

    private CameraBridgeViewBase mOpenCvCameraView;

    private void openCVOnCreate() {
        if (USE_OPENCV) {
            Log.i(TAG, "called onCreate");
            mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.HelloOpenCvView);
            mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
            mOpenCvCameraView.setCvCameraViewListener(this);
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        if (USE_OPENCV) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_6, this, mLoaderCallback);
        }
        registerSensors();
        registerCarControl();
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        deregisterSensors();
        deregisterCarControl();
    }

    public void onCameraViewStarted(int width, int height) {
    }

    public void onCameraViewStopped() {
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat inputGray = inputFrame.gray();
        Mat inputRgba = inputFrame.rgba();
        Mat thresholdImage = new Mat(inputGray.size(), CvType.CV_8UC1);
        Imgproc.Canny(inputGray, thresholdImage, 80, 100);
        Mat lines = new Mat();

        int threshold = 100;
        //int threshold = 50;
        int minLineSize = 50;
        int lineGap = 50;

        // Imgproc.HoughLinesP(thresholdImage, lines, 2, Math.PI/120, threshold, minLineSize, lineGap);
        // Imgproc.HoughLines(thresholdImage, lines, 2, Math.PI/120, threshold, minLineSize, lineGap);
        Imgproc.HoughLines(thresholdImage, lines, 2, Math.PI / 120, threshold);

        Log.v(TAG, String.format("detected %d lines", lines.cols()));
        List<Double> rhos = new ArrayList<Double>(lines.cols());
        List<Double> thetas = new ArrayList<Double>(lines.cols());
        for (int i = 0; i < lines.cols(); i++) {
            double[] vec = lines.get(0, i);
            if (!isEdgeOfImage(vec[0], vec[1], thresholdImage.width(), thresholdImage.height())) {
                rhos.add(vec[0]);
                thetas.add(vec[1]);
            }
        }

        Double theta = null, rho = null;

        if (rhos.size() > 0) {
            rho = medianD(rhos);
            theta = medianD(thetas);

            double a = Math.cos(theta),
                    b = Math.sin(theta);
            double x0 = rho * a,
                    y0 = rho * b;

            double x1 = x0 - 1000 * b,
                    y1 = y0 + 1000 * a,
                    x2 = x0 + 1000 * b,
                    y2 = y0 - 1000 * a;

            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            Log.v(TAG, "Line: rho:" + rho + " theta:" + theta);
            Core.line(inputRgba, start, end, new Scalar(0, 255, 255, 0), 4);
        } else {
        }

        setHead(inputRgba, rho, theta, inputRgba.width(), inputRgba.height());

        Mat outputImage = inputRgba.t();
        Core.flip(outputImage, outputImage, 1);
        Imgproc.resize(outputImage, outputImage, inputRgba.size());

        return outputImage;
    }

    final static double IMAGE_EDGE_ANGLE_ACCURATE = Math.PI / 100;
    final static double IMAGE_EDGE_DISTANCE_ACCURATE = 5;
    private boolean isEdgeOfImage(double rho, double theta, int width, int height) {
        if (theta < IMAGE_EDGE_ANGLE_ACCURATE || theta > 2 * Math.PI - IMAGE_EDGE_ANGLE_ACCURATE) {
            if (rho < IMAGE_EDGE_DISTANCE_ACCURATE || rho > width - IMAGE_EDGE_DISTANCE_ACCURATE)
                return true;
        }
        if (Math.abs(theta - Math.PI / 2) < IMAGE_EDGE_ANGLE_ACCURATE
                || Math.abs(theta - 3 * Math.PI / 2) < IMAGE_EDGE_ANGLE_ACCURATE) {
            if (rho < IMAGE_EDGE_DISTANCE_ACCURATE || rho > height - IMAGE_EDGE_DISTANCE_ACCURATE)
                return true;
        }
        return false;
    }

    private Double medianD(List<Double> array) {
        if (array.size() == 0)
            return null;

        Collections.sort(array);
        double median;
        if (array.size() % 2 == 0)
            median = (array.get(array.size() / 2) + array.get(array.size() / 2 - 1)) / 2;
        else
            median = array.get(array.size() / 2);
        return median;
    }

    private void setHead(Mat imageRgba, Double rho, Double theta, int xSize, int ySize) {
        if (rho == null || theta == null) {
            MoveX = 0;
            MoveY = 0;
        } else {
            double hX = xSize / 2.0;
            double hY = ySize / 2.0;
            double c = Math.cos(theta);
            double s = Math.sin(theta);

            double rhoN = rho - c * hX - s * hY;
            double rhoC = Math.abs(rhoN);
            double thetaC = rhoN < 0 ? (theta + Math.PI) % (2 * Math.PI) : theta;

            double x = rhoC * Math.cos(thetaC);
            double y = rhoC * Math.sin(thetaC);

            Point start = new Point(hX, hY);
            Point end = new Point(x + hX, y + hY);

            Core.line(imageRgba, start, end, new Scalar(0, 255, 255, 0), 4);

            MoveX = y / hY;
            MoveY = 1;
        }
    }

    private void drawLinesForHough(Mat inputRgba, Mat lines) {
        for (int i = 0; i < lines.cols(); i++) {
            double[] vec = lines.get(0, i);

            double a = Math.cos(vec[1]),
                    b = Math.sin(vec[1]);
            double x0 = vec[0] * a,
                    y0 = vec[0] * b;

            double x1 = x0 - 1000 * b,
                    y1 = y0 + 1000 * a,
                    x2 = x0 + 1000 * b,
                    y2 = y0 - 1000 * a;

            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            Core.line(inputRgba, start, end, new Scalar(0, 255, 255, 0), 4);
        }
    }
    private void drawLinesForHoughP(Mat inputRgba, Mat lines) {
        for (int i = 0; i < lines.cols(); i++) {
            double[] vec = lines.get(0, i);
            double x1 = vec[0],
                    y1 = vec[1],
                    x2 = vec[2],
                    y2 = vec[3];

            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            Core.line(inputRgba, start, end, new Scalar(0, 255, 255, 0), 4);
        }
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    break;

                default:
                    super.onManagerConnected(status);
                    break;
            }
        }
    };
}
