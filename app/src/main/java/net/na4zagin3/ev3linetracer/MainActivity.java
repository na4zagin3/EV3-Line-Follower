package net.na4zagin3.ev3linetracer;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;

import ev3command.ev3.Motor;
import ev3command.ev3.comm.AndroidComm;
import ev3command.ev3.comm.EV3Command;


public class MainActivity extends Activity {
    private BluetoothAdapter mBtAdapter = null;
    private Motor mLeftMotor = Motor.B;
    private Motor mRightMotor = Motor.C;


    protected String getEV3MACAddress() {
        return "00:16:53:43:E5:EC";
    }

    protected void failedCreatingConnection() {
    }


    protected void prepareCommunication(){
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
        }
        catch (Exception e) {
            // This exception also occurs when this device hasn’t
            // finished paring
            failedCreatingConnection();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        prepareCommunication();

        mLeftMotor.backward();
    }

    @Override
    protected void onDestroy() {
        // Close the connection
        try {
            EV3Command.close();
        }
        catch (RuntimeException e) {
        }
        super.onDestroy();
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
}
