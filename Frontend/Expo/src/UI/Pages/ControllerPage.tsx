// Controller Page //

// React
import React, { useState } from 'react';

// React Native
import { SafeAreaView, StyleSheet, Text, Image, ScrollView } from 'react-native';

// React Navigation
import { useFocusEffect } from '@react-navigation/native';

// Components
import SidebarMenu from './Components/SidebarMenu';
import TopButtons from './Components/TopButtons';
import OverlayImage from './Components/OverlayImage';

// Functions
import { disableStyleWarning } from './Functions/DisableStyleWarning';

// Constants
import {isIOS, isAndroid, isWeb} from '../Constants/Constants';

const ControllerPage: React.FC = () => {
    
  disableStyleWarning();

  // Show Components
  const [showComponents, setShowComponents] = useState(true);

  // Only Mount Components on Current Page
  useFocusEffect(
    React.useCallback(() => {
      // When focused, show components
      setShowComponents(true);
      return () => {
        // Cleanup when navigating away
        setShowComponents(false); // This will effectively unmount the components
      };
    }, [])
  );

  return (
    <SafeAreaView style={styles.container}>

        {showComponents && (
        <>
          <SidebarMenu />

          <TopButtons />

          <Text style={styles.text}>
            Controller Mapping
          </Text>

          <ScrollView
            style={styles.scrollView}
            contentContainerStyle={styles.scrollContent}
          >

            <SafeAreaView style={styles.mainContainer}>

            <Image
              source={require('../../assets/controller_mapping.png')}
              resizeMode="contain"
              style={styles.image}
            />

            </SafeAreaView>

          </ScrollView>

          <OverlayImage />
        </>
      )}

    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingTop: 20,
    backgroundColor: 'black',
  },
  mainContainer: {
    flexDirection: 'row',
    justifyContent: 'center', // Center the content horizontally
  },
  text: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: isAndroid || isIOS ? 35 : 50, // Text size
    textAlign: 'center', // Center the text
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
    userSelect: 'none',
    marginTop: isAndroid || isIOS ? -10 : -25,
    marginBottom: isAndroid || isIOS ? '0.5%' : '0.5%',
  },
  image: {
    position: 'relative',
    zIndex: 1,
    width: isAndroid || isIOS ? 400 : 700, // Adjust width
    height: isAndroid || isIOS ? 400 : 700, // Adjust height
    top: isAndroid || isIOS ? '-4%' : '-7%', // Center vertically
    pointerEvents: 'none', // Make the image non-clickable
  },
  scrollView: {
    maxHeight: '100%', // Set maximum scroll height (adjust as needed)
    width: '100%', // Adjust width as necessary
  },
  scrollContent: {
    alignItems: 'center',
    paddingTop: 5,
  },
});

export default ControllerPage;
