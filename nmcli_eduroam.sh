nmcli con add type wifi con-name "eduroam" ifname wlan0 \
	ssid "eduroam" wifi-sec.key-mgmt wpa-eap 802-1x.identity "robotics@brandeis.edu"\
 	802-1x.password "robot life good" 802-1x.system-ca-certs yes 802-1x.eap "peap" 802-1x.phase2-auth mschapv2
 