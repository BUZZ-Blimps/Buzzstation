// Battery.tsx

// React
import React, { useEffect } from 'react';

// React Native
import { Pressable, Text, StyleSheet, View } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb } from '../../Constants/Constants';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState, AppDispatch } from '../../Redux/Store';
import { setBatteryColors, setBatteryTexts } from '../../Redux/States';

// Battery Button Props
interface BatteryButtonProps {
  blimpName: string;
  buttonKey: string;
  buttonColor: string;
  buttonText: string;
  onPress: () => void;
}

// Battery Component with Style and onPress function
export const useBattery = () => {

  // Redux Dispatch
  const dispatch: AppDispatch = useDispatch();

  // Update Blimp Battery
  useEffect(() => {
 
    // Event handler for 'update_button_value'
    const handleUpdateButtonValue = (val: { [key: string]: string }) => {
      const receivedName: string = val['name'];
      const receivedButtonKey: string = val['key'];
      const receivedValue: string = val['value'];

      if (receivedButtonKey === 'battery_status') {
        
        // Parse battery data: "voltage,percentage"
        const batteryData = receivedValue.split(',');
        
        if (batteryData.length === 2) {
          const voltage = parseFloat(batteryData[0]);
          const percentage = parseFloat(batteryData[1]);
          
          // Format text: "Battery: 7.4V (30%)"
          const batteryText = `Battery: ${voltage.toFixed(1)}V (${percentage.toFixed(0)}%)`;
          
          // Update battery text for the specific blimp
          dispatch(setBatteryTexts({ [receivedName]: batteryText }));
          
          // Update battery color based on percentage
          let batteryColor = 'green';
          if (percentage <= 20) {
            batteryColor = '#E11C1C'; // Red for critically low battery
          } else if (percentage <= 40) {
            batteryColor = '#EDA90C'; // Yellow/orange for low battery
          }

          dispatch(setBatteryColors({ [receivedName]: batteryColor }));
        }
      }
    };

    if (socket) {
      // Listen for 'update_button_value' events
      socket.on('update_button_value', handleUpdateButtonValue);

      // Request battery status specifically
      socket.emit('request_battery_status');


      // Cleanup to remove the listener when the component is unmounted
      return () => {
        socket.off('update_button_value', handleUpdateButtonValue);

      };
    }

  }, [socket]);

  // Handle Battery Click - no specific action needed
  const handleBatteryClick = (name: string) => {
    // Can add functionality in the future if needed
  };

  return {
    BatteryButton,
    handleBatteryClick,
  };
};

// Battery Button Component
const BatteryButton: React.FC<BatteryButtonProps> = ({ blimpName, buttonKey, buttonColor, buttonText, onPress }) => {
  // Split text into label and value parts
  const textParts = buttonText.split(':');
  const label = textParts[0];
  const value = textParts.length > 1 ? textParts[1] : '';
  
  return (
    <Pressable
        style={[styles.button, { backgroundColor: buttonColor }]}
        onPress={onPress}
        role='button'
        accessible={true}
        android_disableSound={true}
        android_ripple={{ color: 'transparent' }}
      >
      <View style={styles.textContainer}>
          <Text style={[styles.buttonText, styles.buttonTextLabel, { userSelect: 'none' }]}>
            {label}:
          </Text>
          <Text style={[styles.buttonText, styles.buttonTextValue, { userSelect: 'none' }]}>
            {value}
          </Text>
      </View>
    </Pressable>
  );
};

const styles = StyleSheet.create({
  button: {
    width: 235,
    height: 40,
    justifyContent: 'center',
    alignItems: 'center',
    marginTop: 10,
    marginBottom: 10,
    borderRadius: 5,
    borderWidth: 2,
    borderColor: 'black',
  },
  textContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    width: '100%',
  },
  buttonText: {
    fontWeight: 'bold',
    fontSize: isAndroid || isIOS ? 14 : 16,
    textShadowColor: 'black',
    textShadowOffset: { width: 0, height: 0 },
    textShadowRadius: isAndroid || isIOS ? 2 : 2,
  },
  buttonTextLabel: {
    color: 'white',
    marginRight: 5,
  },
  buttonTextValue: {
    color: 'white',
  }
});

export default BatteryButton; 