// Main Page //

// React
import React, { useState, useEffect } from 'react';

// React Native
import { SafeAreaView, StyleSheet } from 'react-native';

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
    
  disableStyleWarning();

  const [isOverlayImage, setOverlayImage] = useState(false);

  return (
    <SafeAreaView style={styles.container}>

       <SidebarMenu />

       <TopButtons setOverlayImage={setOverlayImage} />

      <SafeAreaView style={styles.mainContainer}>

        <BlimpButtons />

        <AllBlimpsButtons />

      </SafeAreaView>

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
});

export default MainPage;
