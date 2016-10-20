/**
 * 
 */
package org.usfirst.frc199.OffseasonBot2016;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * Will be able to:
 * - display a value on smartdashboard with particular key and object value, 
 * - modify key so that it can be used by dashboard widget
 * - get a value of particular type, accessing SmartDashboard as would be expected except with a different key
 * - two methods for each kind of getting, with and without default values
 * 
 */

public interface DashboardInterface {
	
	/**
	 * Puts all desired data on SmartDashboard
	 */
	public void displayData();
	
	
	/*
	 * Methods for displaying values with modified keys
	 */
	default void putNumber(String key, double value) {
		SmartDashboard.putNumber(getKey(key), value);
	}
	default void putBoolean(String key, boolean value) {
		SmartDashboard.putBoolean(getKey(key), value);
	}
	default void putString(String key, String value) {
		SmartDashboard.putString(getKey(key), value);
	}
	default void putSendable(String key, Sendable value) {
		SmartDashboard.putData(getKey(key), value);
	}
	
	/*
	 * Methods for reading numbers, without specifying
	 * the modified key
	 */
	default double getNumber(String key) {
		return SmartDashboard.getNumber(key);
	}
	default double getNumber(String key, double defaultValue) {
		return SmartDashboard.getNumber(key, defaultValue);
	}
	default boolean getBoolean(String key) {
		return SmartDashboard.getBoolean(key);
	}
	default boolean getBoolean(String key, boolean defaultValue) {
		return SmartDashboard.getBoolean(key, defaultValue);
	}
	default String getString(String key) {
		return SmartDashboard.getString(key);
	}
	default String getString(String key, String defaultValue) {
		return SmartDashboard.getString(key, defaultValue);
	}
	
	/**
	 * Converts the specified display key into one with its subsystem name
	 * appended as a prefix, to be compatible with the Subsystem widget on
	 * SmartDashboard for organizational purposes
	 * 
	 * @param key - The name of the original key
	 * @return A modified key with prefix subsystem name
	 */
	default String getKey(String key) {
		return getClass().getSimpleName() + "/" + key;
	}
}
