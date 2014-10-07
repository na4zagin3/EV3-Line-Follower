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
import org.opencv.core.Mat;

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

    private final static boolean USE_OPENCV = false;
    private final static boolean USE_BLUETOOTH = true;

    private PeriodicCarControl carControl = null;

    private void setupSensor() {
        sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
    }

    private void registerSensors() {
        super.onResume();

        // センサの取得
        List<Sensor> sensors = sensorManager.getSensorList(Sensor.TYPE_ALL);

        // センサマネージャへリスナーを登録(implements SensorEventListenerにより、thisで登録する)
        for (Sensor sensor : sensors) {

            if( sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
                sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI);
                isActivatedMagneticSensor = true;
            }

            if( sensor.getType() == Sensor.TYPE_ACCELEROMETER){
                sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI);
                isActivatedAccelerationSensor = true;
            }

            if( sensor.getType() == Sensor.TYPE_GRAVITY){
                sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI);
                isActivatedGravitySensor = true;
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
            setSpeed();
        }

    }

    private void setSpeed() {
        if (gravityValues == null) return;
        double x = gravityValues[0] / SensorManager.GRAVITY_EARTH;
        double y = gravityValues[1] / SensorManager.GRAVITY_EARTH;
        double l = y - x;
        double r = y + x;
        if (l > 1)  l = 1;
        if (l < -1) l = -1;
        if (r > 1)  r = 1;
        if (r < -1) r = -1;

        Log.v(TAG, "Motor: left:" + String.valueOf(l) + " right:" + String.valueOf(r));
        setSpeed(l, r);
    }

    private void setSpeed(double l, double r) {
        boolean forwardL = l > 0;
        boolean forwardR = r > 0;
        if (USE_BLUETOOTH) {
            mLeftMotor.setSpeed(Math.min(99, (int) (Math.abs(l) * 100.0)));
            mRightMotor.setSpeed(Math.min(99, (int) (Math.abs(r) * 100.0)));
            if (forwardL) {
                mLeftMotor.forward();
            } else {
                mLeftMotor.backward();
            }

            if (forwardR) {
                mRightMotor.forward();
            } else {
                mRightMotor.backward();
            }
        }
    }

    private void setupCarControl() {
        carControl = new PeriodicCarControl(100, false);
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
        // getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
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
        return inputFrame.rgba();
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
