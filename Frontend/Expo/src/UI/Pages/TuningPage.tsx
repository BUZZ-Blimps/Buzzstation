// Tuning Page //

// React
import React, { useState } from 'react';

// React Native
import { SafeAreaView, StyleSheet, Text, ScrollView } from 'react-native';

// React Navigation
import { useFocusEffect } from '@react-navigation/native';

// Components
import SidebarMenu from './Components/SidebarMenu';
import TopButtons from './Components/TopButtons';
import BlimpButtons from './Components/BlimpButtons';
import AllBlimpsButtons from './Components/AllBlimpsButtons';
import OverlayImage from './Components/OverlayImage';

// Functions
import { disableStyleWarning } from './Functions/DisableStyleWarning';

// Constants
import {isIOS, isAndroid, isWeb} from '../Constants/Constants';

const TuningPage: React.FC = () => {
    
  // Disable Style Warning
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
              Tuning
            </Text>

            <ScrollView
              style={styles.scrollView}
              contentContainerStyle={styles.scrollContent}
            >

              <SafeAreaView style={styles.mainContainer}>

                {/* <BlimpButtons />

                <AllBlimpsButtons /> */}

              </SafeAreaView>

              <Text style={styles.middleText}>
                  Coming Soon...
              </Text>

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
    justifyContent: 'center',
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
  middleText: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: 50, // Text size
    marginTop: '1%',
    textAlign: 'center', // Center the text
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
    userSelect: 'none',
  },
  scrollView: {
    maxHeight: '100%', // Set maximum scroll height (adjust as needed)
    width: '100%', // Adjust width as necessary
  },
  scrollContent: {
    alignItems: 'center',
    paddingBottom: 16,
  },
});

export default TuningPage;
