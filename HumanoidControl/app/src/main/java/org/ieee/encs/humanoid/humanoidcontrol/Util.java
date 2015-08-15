package org.ieee.encs.humanoid.humanoidcontrol;


import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;
import java.util.List;

public class Util {

	public static String getIpAddress(String subnetFilter) {
		try {
			Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
			while (interfaces.hasMoreElements()) {
				NetworkInterface networkInterface = interfaces.nextElement();
				List<InterfaceAddress> interfaceAddresses = networkInterface.getInterfaceAddresses();
				for (InterfaceAddress address : interfaceAddresses) {
					String hostAddress = address.getAddress().getHostAddress();
					if (hostAddress.startsWith(subnetFilter))
						return hostAddress;
				}
			}
		} catch (Exception e) {
			throw new RuntimeException("Failed to determine IP address for subnet " + subnetFilter, e);
		}
	
		throw new RuntimeException("Failed to determine IP address: No network interfaces found for subnet " + subnetFilter);
	}

	public static void sleepQuietly(int millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			// intentionally empty
		}
	}

}
