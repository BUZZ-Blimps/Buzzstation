// BlimpButtonContainer.tsx

// React
import React from 'react';

// React Native
import { SafeAreaView, StyleSheet } from 'react-native';

// Blimps Container
import BlimpsContainer from '../../Blimps/BlimpsContainer';

// Constants
import {isIOS, isAndroid, isWeb} from '../../Constants/Constants';

const BlimpButtonContainer: React.FC = () => {
  return (
    <SafeAreaView style={styles.blimpContainer}>
      {/* Blimps */}
      <BlimpsContainer />
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  blimpContainer: {
    justifyContent: 'flex-start',
    alignItems: 'center',
    marginTop: '1.75%',
    marginLeft: isAndroid || isIOS ? '10%' : 100,
    right: isAndroid || isIOS ? '4%' : '0%', // Center horizontally
  },
});

export default BlimpButtonContainer;
