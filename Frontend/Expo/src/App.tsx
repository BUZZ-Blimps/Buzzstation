// Main File //

import React from 'react';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import MainPage from './UI/Pages/MainPage';
import TuningPage from './UI/Pages/TuningPage';
import TelemetryPage from './UI/Pages/TelemetryPage';
import LogsPage from './UI/Pages/LogsPage';
import ControllerPage from './UI/Pages/ControllerPage';
import HelpPage from './UI/Pages/HelpPage';

const Stack = createNativeStackNavigator();

const App = () => {
  return (
    <NavigationContainer>
      <Stack.Navigator initialRouteName="Buzzstation">
        <Stack.Screen name="Buzzstation" component={MainPage} options={{ headerShown: false }} />
        <Stack.Screen name="Tuning" component={TuningPage} options={{ headerShown: false }} />
        <Stack.Screen name="Telemetry" component={TelemetryPage} options={{ headerShown: false }} />
        <Stack.Screen name="Logs" component={LogsPage} options={{ headerShown: false }} />
        <Stack.Screen name="Controller" component={ControllerPage} options={{ headerShown: false }} />
        <Stack.Screen name="Help" component={HelpPage} options={{ headerShown: false }} />
      </Stack.Navigator>
    </NavigationContainer>
  );
};

export default App;
