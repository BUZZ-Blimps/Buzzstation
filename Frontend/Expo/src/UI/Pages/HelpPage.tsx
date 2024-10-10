// Help Page //

// React
import React, { useState, useEffect } from 'react';

// React Native
import { SafeAreaView, StyleSheet, Text } from 'react-native';

// Components
import SidebarMenu from './Components/SidebarMenu';
import TopButtons from './Components/TopButtons';
import AllBlimpsButtons from './Components/AllBlimpsButtons';
import OverlayImage from './Components/OverlayImage';

// Functions
import { disableStyleWarning } from './Functions/DisableStyleWarning';

// Constants
import {isIOS, isAndroid, isWeb} from '../Constants/Constants';

const HelpPage: React.FC = () => {
    
  disableStyleWarning();

  const [isOverlayImage, setOverlayImage] = useState(false);

  return (
    <SafeAreaView style={styles.container}>

       <SidebarMenu />

       <TopButtons setOverlayImage={setOverlayImage} />

      <SafeAreaView style={styles.mainContainer}>

        <Text style={styles.text}>
          Help Page
        </Text>

      </SafeAreaView>

      <Text style={styles.middleText}>
          Coming Soon...
      </Text>

      <OverlayImage isOverlayImage={isOverlayImage} />

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
    fontSize: 50, // Text size
    textAlign: 'center', // Center the text
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
    userSelect: 'none',
  },
  middleText: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: 50, // Text size
    marginTop: '10%',
    textAlign: 'center', // Center the text
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
    userSelect: 'none',
  },
});

export default HelpPage;
