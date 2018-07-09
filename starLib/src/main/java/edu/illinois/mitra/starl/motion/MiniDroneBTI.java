package edu.illinois.mitra.starl.motion;

/**
 * Created by VerivitalLab on 2/19/2016.
 */

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.net.wifi.WifiManager;
import android.os.IBinder;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Log;

import com.parrot.arsdk.ardiscovery.ARDiscoveryDeviceBLEService;
import com.parrot.arsdk.ardiscovery.ARDiscoveryDeviceService;
import com.parrot.arsdk.ardiscovery.ARDiscoveryService;
import com.parrot.arsdk.ardiscovery.receivers.ARDiscoveryServicesDevicesListUpdatedReceiver;
import com.parrot.arsdk.ardiscovery.receivers.ARDiscoveryServicesDevicesListUpdatedReceiverDelegate;
import com.parrot.arsdk.arsal.ARSALPrint;
import com.parrot.arsdk.arsal.ARSAL_PRINT_LEVEL_ENUM;

import java.sql.Date;
import java.util.List;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.objects.HandlerMessage;

public class MiniDroneBTI implements DroneBTI, ARDiscoveryServicesDevicesListUpdatedReceiverDelegate, DeviceControllerListener {

    private static String TAG = "MiniDroneBTI";


    static
    {
        try
        {
            System.loadLibrary("arsal");
            System.loadLibrary("arsal_android");
            System.loadLibrary("arnetworkal");
            System.loadLibrary("arnetworkal_android");
            System.loadLibrary("arnetwork");
            System.loadLibrary("arnetwork_android");
            System.loadLibrary("arcommands");
            System.loadLibrary("arcommands_android");
            System.loadLibrary("ardiscovery");
            System.loadLibrary("ardiscovery_android");

            ARSALPrint.setMinimumLogLevel(ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_INFO);

        }
        catch (Exception e)
        {
            Log.e(TAG, "Oops (LoadLibrary)", e);
        }
    }

    private ARDiscoveryService ardiscoveryService;
    private boolean ardiscoveryServiceBound = false;
    private ServiceConnection ardiscoveryServiceConnection;
    public IBinder discoveryServiceBinder;
    public DeviceController deviceController;

    private BroadcastReceiver ardiscoveryServicesDevicesListUpdatedReceiver;

    private Context context;
    private String mac;
    private GlobalVarHolder gvh;
    boolean connected = false;
    private WifiManager.MulticastLock multicastLock;
    //private ConnectTask task;
    public MiniDroneBTI(GlobalVarHolder gvh, Context context, String mac) {
        this.context = context;
        this.mac = mac;
        this.gvh = gvh;
        connect();
    }


    private void connect() {

        initBroadcastReceiver();
        initServiceConnection();
        onServicesDevicesListUpdated();
        registerReceivers();
        initServices();
        //task = new ConnectTask();
        //task.execute();
    }

    private void stopScan() {
        unregisterReceivers();
        closeServices();
    }

    // sets pitch to val percent of max angle
    // positive value moves forward, negative backward
    public void setPitch(double val) {
        if (deviceController != null)
        {
            deviceController.setPitch((byte) val);
            deviceController.setFlag((byte)1);
        }
    }

    // sets roll to val percent of max angle
    // positive value moves right, negative left
    public void setRoll(double val) {
        if (deviceController != null)
        {
            deviceController.setRoll((byte) val);
            deviceController.setFlag((byte)1);
        }
    }

    // sets yaw to val percent of max angular rotation
    // positive value turns right (clockwise from above), negative turns left
    public void setYaw(double val) {
        if (deviceController != null)
        {
            deviceController.setYaw((byte) val);
        }
    }

    // moves the drone up (+ val) or down (- val)
    public void setThrottle(double val) {
        if (deviceController != null)
        {
            deviceController.setGaz((byte) val);
        }
    }

    public void setControlInput(double yaw, double pitch, double roll, double gaz) {
        setYaw(yaw);
        setPitch(pitch);
        setRoll(roll);
        setThrottle(gaz);
    }

    public void sendLanding() {
        if (deviceController != null)
        {
            deviceController.sendLanding();
        }
    }

    public void sendTakeoff() {
        if (deviceController != null)
        {
            deviceController.sendTakeoff();
        }
    }

    // make the drone stop and fall to the ground
    public void sendEmergency() {
        if (deviceController != null)
        {
            deviceController.sendEmergency();
        }
    }

    public void hover() {
        // setting this flag to 0 causes the drone to ignore roll/pitch commands and attempt to hover
        deviceController.setFlag((byte)0);
    }

    public void setMaxTilt(float maxTilt) {
        deviceController.sendMaxTilt(maxTilt);
    }

    public void disconnect() {}


    // The following methods come from the Parrot SDK and allow connection to the drone
    private void initServices()
    {
        if (discoveryServiceBinder == null)
        {
            Intent i = new Intent(context, ARDiscoveryService.class);
            context.bindService(i, ardiscoveryServiceConnection, Context.BIND_AUTO_CREATE);
        }
        else
        {
            ardiscoveryService = ((ARDiscoveryService.LocalBinder) discoveryServiceBinder).getService();
            ardiscoveryServiceBound = true;

            ardiscoveryService.start();
        }
    }

    private void closeServices()
    {
        Log.d(TAG, "closeServices ...");

        if (ardiscoveryServiceBound)
        {
            new Thread(new Runnable() {
                @Override
                public void run()
                {
                    ardiscoveryService.stop();
                    context.unbindService(ardiscoveryServiceConnection);
                    ardiscoveryServiceBound = false;
                    discoveryServiceBinder = null;
                    ardiscoveryService = null;
                }
            }).start();
        }
    }

    private void initBroadcastReceiver()
    {
        ardiscoveryServicesDevicesListUpdatedReceiver = new ARDiscoveryServicesDevicesListUpdatedReceiver(this);
    }

    private void initServiceConnection()
    {
        ardiscoveryServiceConnection = new ServiceConnection()
        {
            @Override
            public void onServiceConnected(ComponentName name, IBinder service)
            {
                discoveryServiceBinder = service;
                ardiscoveryService = ((ARDiscoveryService.LocalBinder) service).getService();
                ardiscoveryServiceBound = true;
                ardiscoveryService.start();
            }

            @Override
            public void onServiceDisconnected(ComponentName name)
            {
                ardiscoveryService = null;
                ardiscoveryServiceBound = false;
            }
        };
    }

    private void registerReceivers()
    {
        LocalBroadcastManager localBroadcastMgr = LocalBroadcastManager.getInstance(context);
        localBroadcastMgr.registerReceiver(ardiscoveryServicesDevicesListUpdatedReceiver, new IntentFilter(ARDiscoveryService.kARDiscoveryServiceNotificationServicesDevicesListUpdated));

    }

    private void unregisterReceivers()
    {
        LocalBroadcastManager localBroadcastMgr = LocalBroadcastManager.getInstance(context);
        localBroadcastMgr.unregisterReceiver(ardiscoveryServicesDevicesListUpdatedReceiver);
    }

    @Override
    public void onServicesDevicesListUpdated()
    {
        Log.d(TAG, "onServicesDevicesListUpdated ...");

        List<ARDiscoveryDeviceService> list;

        if (ardiscoveryService != null)
        {
            list = ardiscoveryService.getDeviceServicesArray();
            if(list != null)
            {
                for (ARDiscoveryDeviceService service : list)
                {
                    Log.d(TAG, "service :  "+ service);
                    if (service.getDevice() instanceof ARDiscoveryDeviceBLEService)
                    {
                        if (service.getName().equals(mac)) {
                            // create a device controller
                            deviceController = new DeviceController(context, service);
                            deviceController.setListener(this);
                            // start the device controller
                            gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_BLUETOOTH, HandlerMessage.BLUETOOTH_CONNECTING);
                            stopScan();
                            startDeviceController();

                        }
                    }
                }
            }
        }
    }

    private void startDeviceController() {
        Log.d(TAG, "staring device controller");
        if(deviceController != null) {
            new Thread(new Runnable() {
                @Override
                public void run()
                {
                    boolean failed = false;

                    failed = deviceController.start();

                    if (failed)
                    {
                        // maybe add a while loop here to keep trying to connect
                    }
                    else
                    {
                        //only with RollingSpider in version 1.97 : date and time must be sent to permit a reconnection
                        Date currentDate = new Date(System.currentTimeMillis());
                        deviceController.sendDate(currentDate);
                        deviceController.sendTime(currentDate);
                        // put message to GUI in StarL here
                        connected = true;
                        gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_BLUETOOTH, HandlerMessage.BLUETOOTH_CONNECTED);
                        // it seems the Parrot code takes this lock away at some point, so even though this is in the main RobotsActivity you have to put it here again
                        WifiManager wifi = (WifiManager) context.getSystemService(Context.WIFI_SERVICE);
                        multicastLock = wifi.createMulticastLock("multicastLock");
                        multicastLock.setReferenceCounted(true);
                        multicastLock.acquire();
                    }
                }
            }).start();
        }

    }

    private void stopDeviceController() {
        if(deviceController != null) {
            new Thread(new Runnable() {
                @Override
                public void run()
                {
                    deviceController.stop();
                }
            }).start();
        }
    }

    @Override
    public void onUpdateBattery(final byte percent)
    {
        // this method has to be implemented
        // maybe make a toast if battery is getting low
    }


    @Override
    public void onDisconnect() {
        stopDeviceController();
        context.unbindService(ardiscoveryServiceConnection);
    }


}
