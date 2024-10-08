// TopButtons.tsx
import React from 'react';
import { SafeAreaView, Image, Pressable, StyleSheet, Platform } from 'react-native';
import Button from '../../Components/Button';
import Controller from '../../Controller/Controller';
import { handleLogoPress } from '../Functions/HandleLogoPress';
import { useBarometer } from '../../AllBlimps/Barometer';
import { isAndroid, isIOS } from '../../Constants/Constants';

interface TopButtonsProps {
  setOverlayImage: React.Dispatch<React.SetStateAction<boolean>>; // Function to toggle overlay image
}

const TopButtons: React.FC<TopButtonsProps> = ({ setOverlayImage }) => {
  
  const {
    BarometerButtonStyle,
    buttonColor: barometerButtonColor,
    buttonText: barometerButtonText,
    handleClick: handleBarometerClick,
  } = useBarometer('#E11C1C', 'Barometer: Disconnected');

  return (
    <SafeAreaView style={styles.container}>

      {/* Controller Input Display */}
      <Controller
        blimpName='none'
        buttonKey='controller'
        buttonColor='black'
        buttonStyle={{
          ...BarometerButtonStyle.button,
          width: isAndroid || isIOS ? 200 : 235,
          height: isAndroid || isIOS ? 40 : 100,
          marginTop: 5,
          marginRight: 20,
          borderColor: 'white',
        }}
        onPress={() => setOverlayImage(prev => !prev)} // Toggle overlay image on controller button press
      />

      {/* Buzz Blimps Logo */}
      <Pressable onPress={handleLogoPress}>
        <Image
          source={require('../../../assets/buzz_blimps_logo.png')}
          resizeMode='contain'
          style={Platform.OS === 'ios' || Platform.OS === 'android' ? styles.imageHalfSize : styles.image}
        />
      </Pressable>

      {/* Barometer Button */}
      <Button
        blimpName='none'
        buttonKey='barometer'
        buttonColor={barometerButtonColor}
        buttonText={barometerButtonText}
        buttonStyle={BarometerButtonStyle}
        onPress={handleBarometerClick}
      />
      
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
  },
  image: {
  },
  imageHalfSize: {
    width: 624 / 2.6,
    height: 150 / 2.6,
  },
});

export default TopButtons;
