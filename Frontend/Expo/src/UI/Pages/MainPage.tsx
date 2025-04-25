// Main Page //

// React
import React, { useState } from 'react';

// React Native
import { SafeAreaView, StyleSheet } from 'react-native';

// React Navigation
import { useFocusEffect, useRoute } from '@react-navigation/native';

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

const MainPage: React.FC = () => {

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

          <SafeAreaView style={styles.mainContainer}>

            <BlimpButtons />

            <AllBlimpsButtons />

          </SafeAreaView>

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
    position : "relative"
  },
  mainContainer: {
    flexDirection: 'row',
    justifyContent: 'center',
    marginTop: -10,
    position: "relative"
  },
});

export default MainPage;
