package com.juma.stsensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

import android.annotation.SuppressLint;
import android.app.Dialog;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.support.v4.content.LocalBroadcastManager;
import android.view.View;
import android.view.Window;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.Button;
import android.widget.ListView;

public class CustomDialog extends Dialog implements android.view.View.OnClickListener,OnItemClickListener{

	private ListView lvDevice = null;

	private Button negativeButton = null;

	private Context context = null;

	private Callback scanCallback = null;

	private CustomListViewAdapter lvDeviceAdapter = null;

	private List<HashMap<String, Object>> deviceInfo = null;

	private android.view.View.OnClickListener negativeButtonClickListener = null;

	public CustomDialog(Context context) {
		super(context);
		this.context = context;
	}
	public CustomDialog(Context context, int theme){
		super(context, theme);
		this.context = context;
	}


	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		Window win = getWindow();
		win.requestFeature(Window.FEATURE_NO_TITLE);
		this.setContentView(R.layout.dialog_scan);

		initView();

	}

	@Override
	public void show() {
		super.show();
	}

	@Override
	public void dismiss() {
		scanCallback.onDismiss();
		super.dismiss();
	}

	private void initView(){

		lvDevice = (ListView) findViewById(R.id.lvDevice);
		negativeButton = (Button) findViewById(R.id.btnCancelScan);

		deviceInfo = new ArrayList<HashMap<String,Object>>();

		lvDeviceAdapter = new CustomListViewAdapter(context, deviceInfo);

		lvDevice.setAdapter(lvDeviceAdapter);

		lvDevice.setOnItemClickListener(this);

		if(negativeButtonClickListener != null)
			negativeButton.setOnClickListener(negativeButtonClickListener);
		else
			negativeButton.setOnClickListener(this);

	}

	public void setScanCallback(Callback scanCallback){

		this.scanCallback = scanCallback;
	}

	@Override
	protected void onStart() {
		super.onStart();
		LocalBroadcastManager.getInstance(context).registerReceiver(receiver, getIntentFilter());
	}

	private IntentFilter getIntentFilter(){
		IntentFilter filter = new IntentFilter();
		filter.addAction(STSensor.ACTION_DEVICE_DISCOVERED);
		return filter;
	}

	@Override
	protected void onStop() {
		super.onStop();
		LocalBroadcastManager.getInstance(context).unregisterReceiver(receiver);
	}


	public void setNegativeButton(android.view.View.OnClickListener listener){

		negativeButtonClickListener = listener;

	}

	@Override
	public void onClick(View v) {
		switch (v.getId()) {
			case R.id.btnCancelScan:
				dismiss();
				break;
		}
	}

	@SuppressWarnings("unchecked")
	@Override
	public void onItemClick(AdapterView<?> arg0, View arg1, int arg2, long arg3) {

		HashMap<String, Object> map = (HashMap<String, Object>) lvDeviceAdapter.getItem(arg2);

		String name = (String) map.get("name");

		UUID uuid = UUID.fromString((String)map.get("uuid"));

		if(scanCallback != null)
			scanCallback.onDevice(uuid, name);
		dismiss();
	}


	public interface Callback{
		void onDevice(UUID uuid, String name);
		void onDismiss();
	}

	private BroadcastReceiver receiver = new BroadcastReceiver() {

		@Override
		public void onReceive(Context context, Intent intent) {
			String uuid = intent.getStringExtra("uuid");
			String name = intent.getStringExtra("name");
			int rssi = intent.getIntExtra("rssi", 0);
			addDeviceInfo(name, uuid, rssi);

		}
	};
	private boolean addDevice = true;
	private void addDeviceInfo(String name, String uuid, int rssi){

		if(deviceInfo != null && lvDeviceAdapter != null){
			HashMap<String , Object> map = new HashMap<String, Object>();
			map.put("name", name);
			map.put("uuid", uuid);
			map.put("rssi", rssi);

			for(int i=0;i<deviceInfo.size();i++){
				if(deviceInfo.get(i).get("uuid").equals(map.get("uuid"))){
					deviceInfo.add(i+1, map);
					deviceInfo.remove(i);
					addDevice = false;
					break;
				}
			}
			if(addDevice){
				deviceInfo.add(map);
			}
			addDevice = true;

			lvDeviceAdapter.notifyDataSetChanged();
		}
	}

	@SuppressLint("UseValueOf")
	public static final byte[] hexToByte(String hex)throws IllegalArgumentException {
		if (hex.length() % 2 != 0) {
			throw new IllegalArgumentException();
		}
		char[] arr = hex.toCharArray();
		byte[] b = new byte[hex.length() / 2];
		for (int i = 0, j = 0, l = hex.length(); i < l; i++, j++) {
			String swap = "" + arr[i++] + arr[i];
			int byteint = Integer.parseInt(swap, 16) & 0xFF;
			b[j] = new Integer(byteint).byteValue();
		}
		return b;
	}
}