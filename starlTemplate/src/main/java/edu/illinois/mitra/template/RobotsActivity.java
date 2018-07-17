package edu.illinois.mitra.template;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.net.wifi.WifiManager;
import android.preference.PreferenceManager;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.CheckBox;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;
import android.os.Bundle;
import android.os.StrictMode;

import edu.illinois.mitra.starl.comms.MessageContents;
import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.gvh.RealGlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.interfaces.MessageListener;
import edu.illinois.mitra.starl.models.Model_Drone;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.HandlerMessage;

import edu.illinois.mitra.demo.follow.FollowApp;
//import edu.illinois.mitra.demo.race.RaceApp;

public class RobotsActivity extends Activity implements MessageListener {
	private static final String TAG = "RobotsActivity";
	private static final String ERR = "Critical Error";

	private static final String IDENTITY_FILE_URL = "https://dl.dropbox.com/s/dwfqdhbf5vdtz18/robots.rif?dl=1";
	private static final String[] ERROR_PARTICIPANTS = {"ERROR"};

	private static final boolean ENABLE_TRACING = false;

	private GlobalVarHolder gvh = null;
	public boolean launched = false;

	// SharedPreferences variables
	private static final String PREF_SELECTED_ROBOT = "SELECTED_ROBOT";
	private int selectedRobot = 0;

	// Logic thread executor
	private ExecutorService executor = Executors.newFixedThreadPool(1);
	private Future<List<Object>> results;
	private LogicThread runThread;
	private MainHandler mainHandler;
	private WifiManager.MulticastLock multicastLock;

	private int numRobots;
	private BotInfoSelector[] botInfo;
	private int i;


	@Override
	protected void onCreate(Bundle savedInstanceState) {
		//testing TURN STRICT MODE OFF
		StrictMode.ThreadPolicy tp = StrictMode.ThreadPolicy.LAX;
		StrictMode.setThreadPolicy(tp);

		//StrictMode.ThreadPolicy()
		super.onCreate(savedInstanceState);
		this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
		setContentView(R.layout.main);

		// this code allows the MotoE (and probably other phones) to receive broadcast udp packets
		// they don't accept broadcast messages by default to save battery
		WifiManager wifi = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE); //changed tim to prevent memory leak added getApplicationContext()
		multicastLock = wifi.createMulticastLock("multicastLock");
		multicastLock.setReferenceCounted(true);
		multicastLock.acquire();

		// Load the participants
		//participants = IdentityLoader.loadIdentities(IDENTITY_FILE_URL);
		// Put number of robots being used here
		numRobots = 1;
		botInfo = new BotInfoSelector[numRobots];
		// add color, robot type, and device type for each robot here
		//botInfo[0] = new BotInfoSelector("red", Common.MAVIC, Common.NEXUS7);
		//botInfo[1] = new BotInfoSelector("green", Common.IROBOT, Common.MOTOE);
		//botInfo[2] = new BotInfoSelector("blue", Common.IROBOT, Common.NEXUS7);
		// botInfo[3] = new BotInfoSelector("white", Common.IROBOT, Common.NEXUS7);
		botInfo[0] = new BotInfoSelector("blue", Common.PHANTOM, Common.NEXUS7);

		// Row 0 = names
		// Row 1 = IPs
		String[] participantNames = new String[numRobots];
		String[] participantIPs = new String[numRobots];
		for (i = 0; i < numRobots; i++) {
			participantNames[i] = botInfo[i].name;
			participantIPs[i] = botInfo[i].ip;
		}


		// Initialize preferences holder
		SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
		selectedRobot = prefs.getInt(PREF_SELECTED_ROBOT, 0);

		if (selectedRobot >= participantNames.length) {
			Toast.makeText(this, "Identity error! Reselect robot identity", Toast.LENGTH_LONG).show();
			selectedRobot = 0;
		}

		// Set up the GUI
		setupGUI(participantNames); // names

		// Create the main handler
		mainHandler = new MainHandler(this, pbBluetooth, pbBattery, cbGPS, cbBluetooth, cbRunning, txtDebug, cbRegistered);

		//if (participantNames == null || participantIPs == null) {
		//	Toast.makeText(this, "Error loading identity file!", Toast.LENGTH_LONG).show();
		//	participantNames = ERROR_PARTICIPANTS;
		//	participantIPs = ERROR_PARTICIPANTS;
		//	selectedRobot = 0;
		//}

		// Create the global variable holder
		HashMap<String, String> hm_participants = new HashMap<String, String>();
		for (int i = 0; i < participantNames.length; i++) {
			hm_participants.put(participantNames[i], participantIPs[i]);
		}
		gvh = new RealGlobalVarHolder(participantNames[selectedRobot], hm_participants, botInfo[selectedRobot].type, mainHandler, this);
		mainHandler.setGvh(gvh);

		//Connect
		connect();

		createAppInstance(gvh);
	}

	public void createAppInstance(GlobalVarHolder gvh) {
		//runThread = new FollowApp(gvh);    // Instantiate your application here!
		runThread = new FollowApp(gvh);
		// Example: runThread = new LightPaintActivity(gvh);
	}

	public void launch(int numWaypoints, int runNum) {
		if (!launched) {
			if (gvh.gps.getWaypointPositions().getNumPositions() == numWaypoints) {
				if (ENABLE_TRACING)
					gvh.trace.traceStart(runNum);
				launched = true;
				cbRunning.setChecked(true);

				gvh.trace.traceSync("APPLICATION LAUNCH", gvh.time());

				RobotMessage informLaunch = new RobotMessage("ALL", gvh.id.getName(), Common.MSG_ACTIVITYLAUNCH, new MessageContents(Common.intsToStrings(numWaypoints, runNum)));
				gvh.comms.addOutgoingMessage(informLaunch);
				results = executor.submit(runThread);
			} else {
				gvh.plat.sendMainToast("Should have " + numWaypoints + " waypoints, but I have " + gvh.gps.getWaypointPositions().getNumPositions());
			}
		}
	}

	public void abort() {
		runThread.cancel();
		results.cancel(true);
		executor.shutdownNow();
		executor = Executors.newSingleThreadExecutor();
		createAppInstance(gvh);
	}

	public void connect() {
		// Update GUI
		gvh.log.d(TAG, gvh.id.getName());

		// Begin persistent background threads
		gvh.comms.startComms();
		gvh.gps.startGps();

		// Register this as a listener
		gvh.comms.addMsgListener(this, Common.MSG_ACTIVITYLAUNCH, Common.MSG_ACTIVITYABORT);
	}

	public void disconnect() {
		gvh.log.i(TAG, "Disconnecting and stopping all background threads");

		// Shut down the logic thread if it was running
		if (launched) {
			runThread.cancel();
			executor.shutdownNow();
		}
		launched = false;

		// Shut down persistent threads
		gvh.comms.stopComms();
		gvh.gps.stopGps();
		gvh.plat.moat.cancel();
	}

	private TextView txtRobotName;
	private TextView txtDebug;
	private TextView txtDestination;
	private ProgressBar pbBluetooth;
	private CheckBox cbGPS;
	private CheckBox cbBluetooth;
	private CheckBox cbRunning;
	private CheckBox cbRegistered; //status of DJI SDK registration
	private ProgressBar pbBattery;


	private void setupGUI(final String[] participantNames) {
		final SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());

		txtRobotName = (TextView) findViewById(R.id.txtRobotName);
		cbGPS = (CheckBox) findViewById(R.id.cbGPS);
		cbBluetooth = (CheckBox) findViewById(R.id.cbBluetooth);
		cbRunning = (CheckBox) findViewById(R.id.cbRunning);
		txtDebug = (TextView) findViewById(R.id.tv_debug);
		pbBluetooth = (ProgressBar) findViewById(R.id.pb_bluetooth);
		pbBattery = (ProgressBar) findViewById(R.id.pbBattery);
		pbBattery.setMax(100);
		cbRegistered = (CheckBox) findViewById(R.id.cbRegistered);

		if (!(botInfo[selectedRobot].type instanceof Model_Drone)) {
			cbRegistered.setVisibility(View.GONE);
		} else {
			cbBluetooth.setText("Drone Connected");
		}

		txtRobotName.setText(participantNames[selectedRobot]);
		txtRobotName.setOnClickListener(new OnClickListener() {
			public void onClick(View arg0) {
				AlertDialog.Builder sel_robot_builder = new AlertDialog.Builder(RobotsActivity.this);
				sel_robot_builder.setTitle("Who Am I?");
				sel_robot_builder.setItems(participantNames, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int item) {
						selectedRobot = item;
						txtRobotName.setText(participantNames[selectedRobot]);
						SharedPreferences.Editor spe = prefs.edit();
						spe.putInt(PREF_SELECTED_ROBOT, selectedRobot);
						spe.commit();

						// Restart the application
						Intent restart = getIntent();
						disconnect();
						finish();
						startActivity(restart);
					}
				});
				AlertDialog sel_robot = sel_robot_builder.create();
				sel_robot.show();
			}
		});

	}

	@Override
	public void onBackPressed() {
		gvh.log.e(TAG, "Exiting application");
		disconnect();
		finish();
		if (multicastLock != null) {
			multicastLock.release();
			multicastLock = null;
		}
		return;
	}

	@Override
	public void messageReceived(RobotMessage m) {
		switch (m.getMID()) {
			case Common.MSG_ACTIVITYLAUNCH:
				gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_LAUNCH, Integer.parseInt(m.getContents(0)), Integer.parseInt(m.getContents(1)));
				break;
			case Common.MSG_ACTIVITYABORT:
				gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_ABORT, null);
				break;
		}
	}

}