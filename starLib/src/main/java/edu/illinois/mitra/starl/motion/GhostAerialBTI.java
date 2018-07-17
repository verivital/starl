package edu.illinois.mitra.starl.motion;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.os.IBinder;
import android.util.Log;
import android.widget.Toast;

import com.ehang.coptersdk.CopterControl;
import com.ehang.coptersdk.connection.OnConnectionListener;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.objects.HandlerMessage;


/**
 * Created by wangcs2 on 6/1/2017.
 */

public class GhostAerialBTI implements DroneBTI, DeviceControllerListener {


    private static String TAG = "GADroneBTI";

    /*
    private ServiceConnection bluetoothServiceConnection;
    private Service bluetoothService;
    private ComponentName serviceName;
    */
    public IBinder bluetoothServiceBinder;
    private BluetoothDevice GADrone;
    private GhostAerialDeviceController deviceController;
    private boolean bound;

    //private DeviceFinder devicesListUpdatedReceiver;

    private BluetoothAdapter BTAdapter;

    private Context context;
    private String mac;
    private GlobalVarHolder gvh;
    private OnConnectionListener GBoxListener;

    //private ConnectTask task;
    public GhostAerialBTI(GlobalVarHolder gvh, Context context, String mac) {
        this.context = context;
        this.mac = mac;
        this.gvh = gvh;
        myConnect();
    }

    /*public String getMac(){
        return this.mac;
    }*/

    private void myConnect() {
        bound = false;
        //BTAdapter = BluetoothAdapter.getDefaultAdapter();
        initReceivers();
        CopterControl.getInstance().getConnection().connect(mac, GBoxListener);


        //registerReceivers();
        //startSearch();

        /* initServices();
        initServiceConnection();*/
        //task = new ConnectTask();
        //task.execute();
    }

    /*private void startSearch() {
       BTAdapter.startDiscovery();
    }*/

    public void stopScan() {
        unregisterReceivers();
        //closeServices();
    }

    // sets pitch to val percent of max angle
    // positive value moves forward, negative backward
    public void setPitch(double val) {
        if (deviceController != null) {
            deviceController.setPitch(val);
        }
    }

    // sets roll to val percent of max angle
    // positive value moves right, negative left
    public void setRoll(double val) {
        if (deviceController != null) {
            deviceController.setRoll(val);
        }
    }

    // sets yaw to val percent of max angular rotation
    // positive value turns right (clockwise from above), negative turns left
    public void setYaw(double val) {
        if (deviceController != null) {
            if (val > 1) {
                deviceController.setYaw(1);
            } else if (val < -1) {
                deviceController.setYaw(-1);
            } else {
                deviceController.setYaw(val);
            }

        }
    }

    // moves the drone up (+ val) or down (- val)
    public void setThrottle(double val) {
        int tmp = (int) val;
        if (deviceController != null) {
            deviceController.setGaz(tmp);
        }
    }

    public void setControlInput(double yaw, double pitch, double roll, double gaz) {
        setYaw(yaw);
        setPitch(pitch);
        setRoll(roll);
        setThrottle(gaz);
    }

    public void sendLanding() {
        if (deviceController != null) {
            deviceController.sendLanding();
        }
    }

    public void sendTakeoff() {
        if (deviceController != null) {
            deviceController.sendTakeoff();
        }
    }

    // make the drone stop and fall to the ground
    public void sendEmergency() {
        if (deviceController != null) {
            deviceController.sendEmergency();
        }
    }

    public void hover() {
        // setting this flag to 0 causes the drone to ignore roll/pitch commands and attempt to hover
        deviceController.setFlag(true);
    }

    public void setMaxTilt(float maxTilt) {
        deviceController.sendMaxTilt(maxTilt);
    }

    public void disconnect() {}


/*

        private void initServices() {
            if (bluetoothServiceBinder == null) {
                Intent i = new Intent(context, GhostAerialBTI.class);
                context.bindService(i, bluetoothServiceConnection, Context.BIND_AUTO_CREATE);
            }

        }

        private void closeServices() {
            Log.d(TAG, "closeServices ...");

            if (bound) {
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        bluetoothService.stopSelf();
                        context.unbindService(bluetoothServiceConnection);
                        bound = false;
                        bluetoothServiceBinder = null;
                        bluetoothService = null;
                    }
                }).start();
            }
        }
*/

    private void initReceivers() {
        final Context context = this.context;
        GBoxListener = new OnConnectionListener() {
            @Override
            public void onSuccess() {
                Toast.makeText(context, "Bluetooth Connected", Toast.LENGTH_SHORT).show();
                Log.i(TAG, "connected " + CopterControl.getInstance().isCopterConnected());
                gvh.plat.sendMainMsg(HandlerMessage.COPTER_CONNECTED, CopterControl.getInstance().isCopterConnected());
                setupController();
            }

            @Override
            public void onFailure() {
                Toast.makeText(context, "Bluetooth Disconnected", Toast.LENGTH_SHORT).show();
            }
        };
        //devicesListUpdatedReceiver = new DeviceFinder();

    }

    private void setupController() {
        Log.d(TAG, "Setup Controller");
        Log.i(TAG, "connected " + CopterControl.getInstance().isCopterConnected());
        deviceController = new GhostAerialDeviceController(context);
        deviceController.setListener(this);
        // start the device controller
        gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_BLUETOOTH, HandlerMessage.BLUETOOTH_CONNECTED);
        startDeviceController();
        //stopScan();

    }
/*

        private void initServiceConnection() {
            bluetoothServiceConnection = new ServiceConnection() {
                @Override
                public void onServiceConnected(ComponentName name, IBinder service) {
                    bluetoothServiceBinder = service;
                    serviceName = name;
                    bound = true;
                }

                @Override
                public void onServiceDisconnected(ComponentName name) {
                    bluetoothService = null;
                    bound = false;
                }
            };
        }
*/

    private void registerReceivers() {
        //IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        //this.context.registerReceiver(devicesListUpdatedReceiver, filter);

    }

    private void unregisterReceivers() {
        //this.context.unregisterReceiver(devicesListUpdatedReceiver);
    }

    private void startDeviceController() {
        Log.d(TAG, "staring device controller");

        if (deviceController != null) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    //TODO: Assumes connection; Change A.
                    deviceController.start();
                }
            }).start();
        }

    }

    private void stopDeviceController() {
        if (deviceController != null) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    deviceController.stop();
                }
            }).start();
        }
    }

    @Override
    public void onUpdateBattery(final byte percent) {
        // this method has to be implemented
        // maybe make a toast if battery is getting low
    }


    @Override
    public void onDisconnect() {
        stopDeviceController();
        CopterControl.getInstance().getConnection().disconnect();
        // context.unbindService(ardiscoveryServiceConnection);
    }

       /* private class DeviceFinder extends BroadcastReceiver {

            @Override
            public void onReceive(Context context, Intent intent) {
                String action = intent.getAction();
                if (GADrone.ACTION_FOUND.equals(action)) {
                    // create a device controller
                    BluetoothDevice tmpDev = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                    String name = tmpDev.getName();
                    String deviceMAC = tmpDev.getAddress();
                    String mac = getMac();
                    if(deviceMAC == mac)
                    {
                        //CopterControl.getInstance().getConnection().connect(mac,GBoxListener);
                    }

                }
            }
        }*/

}

