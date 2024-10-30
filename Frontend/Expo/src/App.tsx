// Main File //

// React
import React, { useEffect, useState } from 'react';

// React Native Screens
import { enableScreens } from 'react-native-screens';

// React Navigation
import { NavigationContainer, useNavigation, NavigationProp } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';

// Pages
import MainPage from './UI/Pages/MainPage';
import TuningPage from './UI/Pages/TuningPage';
import TelemetryPage from './UI/Pages/TelemetryPage';
import LogsPage from './UI/Pages/LogsPage';
import ControllerPage from './UI/Pages/ControllerPage';
import HelpPage from './UI/Pages/HelpPage';

// Constants
import { isIOS, isAndroid, isWeb} from './UI/Constants/Constants';

// Redux
import { Provider } from 'react-redux';
import store from './UI/Redux/Store';

// Async Storage
import AsyncStorage from '@react-native-async-storage/async-storage';

// Enable screens for performance
enableScreens();

// Create Native Stack Navigator
const Stack = createNativeStackNavigator();

const App = () => {

  // Loading State
  const [isLoading, setIsLoading] = useState(true);

  // Navigation State
  const [navigationState, setNavigationState] = useState<any>(undefined);

  // Current Route
  const [currentPage, setCurrentPage] = useState<string | null>("Buzzstation");

  // Reload to Previous Page or Default Home Page
  useEffect(() => {
    const restoreNavigationState = async () => {
      const savedStateString = await AsyncStorage.getItem('NAVIGATION_STATE');
      if (savedStateString) {
        const savedState = JSON.parse(savedStateString);
        setNavigationState(savedState);

        // Set the initial route name based on saved state
        if (savedState.routes.length > 0) {
          setCurrentPage(savedState.routes[savedState.index].name);
        }
      }

      setIsLoading(false); // Set loading to false after restoring state
    };

    restoreNavigationState();
  }, []);

  // Check if the Current Page has changed
  const onStateChange = (state: any) => {
    setNavigationState(state);
    AsyncStorage.setItem('NAVIGATION_STATE', JSON.stringify(state));
  };

  // Loading
  if (isLoading) {
    return null; // Or a loading spinner/component
  }

  return (
    <>
    {/* Web */}
    {isWeb && (
      <Provider store={store}>
      <NavigationContainer
        initialState={navigationState} // Use initialState to restore the navigation state
        onStateChange={onStateChange} // onStateChange is now correctly used here
      >
        <Stack.Navigator initialRouteName={currentPage || "Buzzstation"}>
          <Stack.Screen name="Buzzstation" component={MainPage} options={{ headerShown: false }} />
          <Stack.Screen name="Tuning" component={TuningPage} options={{ headerShown: false }} />
          <Stack.Screen name="Telemetry" component={TelemetryPage} options={{ headerShown: false }} />
          <Stack.Screen name="Logs" component={LogsPage} options={{ headerShown: false }} />
          <Stack.Screen name="Controller" component={ControllerPage} options={{ headerShown: false }} />
          <Stack.Screen name="Help" component={HelpPage} options={{ headerShown: false }} />
        </Stack.Navigator>
      </NavigationContainer>
    </Provider>
    )}
    {/* App */}
    {isAndroid || isIOS && (
    <Provider store={store}>
      <NavigationContainer>
        <Stack.Navigator initialRouteName={"Buzzstation"}>
          <Stack.Screen name="Buzzstation" component={MainPage} options={{ headerShown: false }} />
          <Stack.Screen name="Tuning" component={TuningPage} options={{ headerShown: false }} />
          <Stack.Screen name="Telemetry" component={TelemetryPage} options={{ headerShown: false }} />
          <Stack.Screen name="Logs" component={LogsPage} options={{ headerShown: false }} />
          <Stack.Screen name="Controller" component={ControllerPage} options={{ headerShown: false }} />
          <Stack.Screen name="Help" component={HelpPage} options={{ headerShown: false }} />
        </Stack.Navigator>
      </NavigationContainer>
    </Provider>
    )}
    </>
  );
};

export default App;
