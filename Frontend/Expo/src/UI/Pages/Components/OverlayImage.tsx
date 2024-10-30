// OverlayImage.tsx

// React
import React from 'react';

// React Native
import { Image, StyleSheet } from 'react-native';

// Constants
import {isIOS, isAndroid, isWeb} from '../../Constants/Constants';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../../Redux/Store';

const OverlayImage = () => {

  // Overlay Image State
  const overlayImageState = useSelector((state: RootState) => state.app.overlayImageState);
  
  return (
    overlayImageState && (
      <Image
        source={require('../../../assets/controller_mapping.png')}
        style={styles.overlayImage}
        resizeMode="contain"
      />
    )
  );
};

const styles = StyleSheet.create({
  overlayImage: {
    position: 'absolute',
    zIndex: 1, // Put behind buttons
    width: '100%', // Adjust width
    height: '100%', // Adjust height
    top: '0%', // Center vertically
    left: isAndroid || isIOS ? '8%' : '0%', // Center horizontally
    pointerEvents: 'none', // Make the image non-clickable
    opacity: 0.75,
  },
});

export default OverlayImage;
