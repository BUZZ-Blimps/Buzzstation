// Main Page //

// React
import React, { useState, useRef } from 'react';

// React Native
import { SafeAreaView, StyleSheet } from 'react-native';

// React Navigation
import { useFocusEffect, useRoute } from '@react-navigation/native';

// React hls live stream
import ReactHlsPlayer from 'react-hls-player';

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


const CameraStream = () => {
  const playerRef = useRef(null);
  return (
    <div>
      <ReactHlsPlayer
        playerRef={playerRef}
        src="http://192.168.0.200:8888/cam6/video1_stream.m3u8"  // Ensure your URL is a valid m3u8 stream
        autoPlay={true}
        controls={true}
        width="100%"
        height="auto"
        hlsConfig={{}}
      />
    </div>
  );
};

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

            <CameraStream/>

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
  },
  mainContainer: {
    flexDirection: 'row',
    justifyContent: 'center',
    marginTop: -10,
  },
});

export default MainPage;
