package com.juma.stsensor;

import java.util.HashMap;
import java.util.List;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.TextView;

public class CustomListViewAdapter extends BaseAdapter {
	
	public static final String NAME_STR = "name";
	public static final String UUID_STR = "uuid";
	public static final String RSSI_STR = "rssi";
	
	private List<HashMap<String, Object>> deviceInfo = null;;
	
	private LayoutInflater inflater = null;;

	public CustomListViewAdapter(Context context,  List<HashMap<String, Object>> deviceInfo) {
		inflater = LayoutInflater.from(context);
		this.deviceInfo = deviceInfo;

	}

	@Override
	public int getCount() {
		return deviceInfo.size();
	}

	@Override
	public Object getItem(int position) {
		return deviceInfo.get(position);
	}

	@Override
	public long getItemId(int position) {
		return position;
	}

	@Override
	public View getView(int position, View convertView, ViewGroup parent) {
		ViewGroup vg;

		if (convertView != null) {
			vg = (ViewGroup) convertView;
		} else {
			vg = (ViewGroup) inflater.inflate(R.layout.devcie_list_item, null);
		}

		TextView tvName = ((TextView) vg.findViewById(R.id.tvName));
		TextView tvUuid = (TextView) vg.findViewById(R.id.tvUuid);
		TextView tvRssi = (TextView) vg.findViewById(R.id.tvRssi);;

		tvName.setText((String)deviceInfo.get(position).get(NAME_STR));
		tvUuid.setText((String)deviceInfo.get(position).get(UUID_STR));
		tvRssi.setText((Integer) deviceInfo.get(position).get(RSSI_STR)+"");
		
		return vg;
	}
}
