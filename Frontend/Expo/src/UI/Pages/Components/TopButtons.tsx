// TopButtons.tsx

import React from 'react';
import { SafeAreaView, Image, Pressable, StyleSheet, Platform } from 'react-native';
import Button from '../../Components/Button';
import Controller from '../../Controller/Controller';
import { handleLogoPress } from '../Functions/HandleLogoPress';
import { useBarometer } from '../../AllBlimps/Barometer';
import { isAndroid, isIOS } from '../../Constants/Constants';
import { getUserID } from '../../Users/UserManager';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../../Redux/Store';
import { setOverlayImageState } from '../../Redux/States';


const TopButtons = () => {
  
  const dispatch = useDispatch();

  // Overlay Image State
  const overlayImageState = useSelector((state: RootState) => state.app.overlayImageState);

  // User ID
  const { userID } = getUserID();

  // Barometer Color
  const barometerColor = useSelector((state: RootState) => state.app.barometerColor);

  // Barometer Text
  const barometerText = useSelector((state: RootState) => state.app.barometerText);

  // Barometer Button Style and Click Function
  const {
    BarometerButtonStyle,
    handleClick: handleBarometerClick,
  } = useBarometer('#E11C1C', 'Barometer: Disconnected');

  return (
    <SafeAreaView style={styles.container}>

      {/* Controller Input Display */}
      <Controller
        blimpName='none'
        buttonKey='controller'
        buttonColor='black'
        onPress={() => dispatch(setOverlayImageState(!overlayImageState))} // Toggle overlay image on controller button press
      />

      {/* Buzz Blimps Logo */}
      <Pressable onPress={() => handleLogoPress(userID)}>
        <Image
          source={require('../../../assets/buzz_blimps_logo.png')}
          resizeMode='contain'
          style={isAndroid || isIOS ? styles.imageHalfSize : styles.image}
        />
      </Pressable>

      {/* Barometer Button */}
      <Button
        blimpName='none'
        buttonKey='barometer'
        buttonColor={barometerColor}
        buttonText={barometerText}
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
    marginBottom: '1%',
  },
  image: {
  },
  imageHalfSize: {
    width: 624 / 2.6,
    height: 150 / 2.6,
    left: isAndroid || isIOS ? '8%' : '0%',
  },
});

export default TopButtons;
