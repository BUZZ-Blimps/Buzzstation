// OverlayImage.tsx

// React
import React from 'react';

// React Native
import { Image, StyleSheet } from 'react-native';

// Constants
import {isIOS, isAndroid, isWeb} from '../../Constants/Constants';

interface OverlayImageProps {
  isOverlayImage: boolean; // Define the type for the prop
}

const OverlayImage: React.FC<OverlayImageProps> = ({ isOverlayImage }) => {
  return (
    isOverlayImage && (
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
    width: '100%', // Adjust width to 75% of the parent
    height: '100%', // Adjust height to 75% of the parent
    top: '0%', // Center vertically
    left: isAndroid || isIOS ? '8%' : '0%', // Center horizontally
    pointerEvents: 'none', // Make the image non-clickable
    opacity: 0.75,
  },
});

export default OverlayImage;
