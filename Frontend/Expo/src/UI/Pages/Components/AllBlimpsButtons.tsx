// AllBlimpsButtons.tsx

// React
import React from 'react';

// React Native
import { SafeAreaView, StyleSheet } from 'react-native';

// Components
import Button from '../../Components/Button';

// Functions
import { useTargetButtonColor } from '../../AllBlimps/TargetButtonColor'; 
import { useAllMode } from '../../AllBlimps/AllMode';

// Constants
import {isIOS, isAndroid, isWeb} from '../../Constants/Constants';

const AllBlimpsButtons = () => {
  const { 
    buttonColor: goalColorButtonColor, 
    handleClick: handleGoalColorClick 
  } = useTargetButtonColor('#FF5D01', 'goal_color');

  const { 
    buttonColor: enemyColorButtonColor, 
    handleClick: handleEnemyColorClick 
  } = useTargetButtonColor('#0044A6', 'enemy_color');

  const { 
    handleClick: handleAllAutoClick 
  } = useAllMode('green', 'all_auto');

  const { 
    handleClick: handleAllManualClick 
  } = useAllMode('#E11C1C', 'all_manual');

  return (
    <SafeAreaView style={styles.allBlimpsButtonContainer}>
      <Button 
        blimpName='none' 
        buttonKey='goal_color' 
        buttonColor={goalColorButtonColor} 
        buttonText='Goal' 
        buttonStyle={TargetButtonStyle} 
        onPress={() => handleGoalColorClick('goal_color')} 
      />
      <Button 
        blimpName='none' 
        buttonKey='enemy_color' 
        buttonColor={enemyColorButtonColor} 
        buttonText='Enemy' 
        buttonStyle={TargetButtonStyle} 
        onPress={() => handleEnemyColorClick('enemy_color')} 
      />
      <Button 
        blimpName='none' 
        buttonKey='all_auto' 
        buttonColor='green' 
        buttonText='All Auto' 
        buttonStyle={AllModeButtonStyle} 
        onPress={() => handleAllAutoClick('all_auto')} 
      />
      <Button 
        blimpName='none' 
        buttonKey='all_manual' 
        buttonColor='#E11C1C' 
        buttonText='All Manual' 
        buttonStyle={AllModeButtonStyle} 
        onPress={() => handleAllManualClick('all_manual')} 
      />
    </SafeAreaView>
  );
};

// Target Button Style
const TargetButtonStyle = StyleSheet.create({
  button: {
    width: 100,
    height: 50,
    alignItems: 'center',
    justifyContent: 'center',
    marginVertical: 12,
    borderRadius: 5,
    borderWidth: 2,
    borderColor: 'black',
  },
  buttonText: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: 18, // Text size
    textAlign: 'center', // Center the text horizontally
    verticalAlign: 'middle', // Center the text vertically
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 0, height: 0 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 2 : 2.5, // Spread of the shadow
  },
}); 

// All Mode Button Style
const AllModeButtonStyle = StyleSheet.create({
  button: {
    width: 100,
    height: 50,
    alignItems: 'center',
    justifyContent: 'center',
    marginVertical: 12,
    borderRadius: 5,
    borderWidth: 2,
    borderColor: 'black',
  },
  buttonText: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: 15, // Text size
    textAlign: 'center', // Center the text horizontally
    verticalAlign: 'middle', // Center the text vertically
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 0, height: 0 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 2 : 2.5, // Spread of the shadow
  },
}); 

const styles = StyleSheet.create({
  allBlimpsButtonContainer: {
    flexDirection: 'column', // Arranges buttons horizontally
    justifyContent: 'flex-end',
    left: isAndroid || isIOS ? '3%' : '0%',
    marginLeft: 5,
  },
});

export default AllBlimpsButtons;
