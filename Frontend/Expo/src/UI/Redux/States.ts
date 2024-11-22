// States.ts

import { createSlice, PayloadAction } from '@reduxjs/toolkit';

// Define the shape of your state
interface AppState {
  userID: string | null;
  names: string[];
  nameColors: { [key: string]: string };
  nameLastHeartbeats: { [key: string]: number };
  stateValues: { [key: string]: string };
  catchValues: { [key: string]: number };
  modeColors: { [key: string]: string };
  calibrateColors: { [key: string]: string };
  calibrateTexts: { [key: string]: string };
  visionColors: { [key: string]: string };
  catchIcons: { [key: string]: boolean };
  shootIcons: { [key: string]: boolean };
  goalColor: string;
  enemyColor: string;
  barometerColor: string;
  barometerText: string;
  sidebarMenuState: boolean;
  overlayImageState: boolean;
}

// Initial state with default values
const initialState: AppState = {
  userID: null,  // Set initial state for userID to null
  names: [],
  nameColors: {},
  nameLastHeartbeats: {},
  stateValues: {},
  catchValues: {},
  modeColors: {},
  calibrateColors: {},
  calibrateTexts: {},
  visionColors: {},
  catchIcons: {},
  shootIcons: {},
  goalColor: '',
  enemyColor: '',
  barometerColor: '',
  barometerText: '',
  sidebarMenuState: false,
  overlayImageState: false,
};

// Create a slice with actions
const appSlice = createSlice({
  name: 'app',
  initialState,
  reducers: {
    setUserID(state, action: PayloadAction<string | null>) {
      state.userID = action.payload;  // Expecting a string or null as payload
    },
    setNames(state, action: PayloadAction<string[]>) {
      state.names = action.payload;
    },
    setNameColors(state, action: PayloadAction<{ [key: string]: string }>) {
      Object.assign(state.nameColors, action.payload);
    },
    setNameLastHeartbeats(state, action: PayloadAction<{ [key: string]: number }>) {
      Object.assign(state.nameLastHeartbeats, action.payload);
    },
    setStateValues(state, action: PayloadAction<{ [key: string]: string }>) {
      Object.assign(state.stateValues, action.payload);
    },
    setCatchValues(state, action: PayloadAction<{ [key: string]: string }>) {
      Object.assign(state.catchValues, action.payload);
    },
    setModeColors(state, action: PayloadAction<{ [key: string]: string }>) {
      Object.assign(state.modeColors, action.payload);
    },
    setCalibrateColors(state, action: PayloadAction<{ [key: string]: string }>) {
      Object.assign(state.calibrateColors, action.payload);
    },
    setCalibrateTexts(state, action: PayloadAction<{ [key: string]: string }>) {
      Object.assign(state.calibrateTexts, action.payload);
    },
    setVisionColors(state, action: PayloadAction<{ [key: string]: string }>) {
      Object.assign(state.visionColors, action.payload);
    },
    setCatchIcons(state, action: PayloadAction<{ [key: string]: boolean }>) {
      Object.assign(state.catchIcons, action.payload);
    },
    setShootIcons(state, action: PayloadAction<{ [key: string]: boolean }>) {
      Object.assign(state.shootIcons, action.payload);
    },
    setGoalColor(state, action: PayloadAction<string>) {
      state.goalColor = action.payload;
    },
    setEnemyColor(state, action: PayloadAction<string>) {
      state.enemyColor = action.payload;
    },
    setBarometerColor(state, action: PayloadAction<string>) {
      state.barometerColor = action.payload;
    },
    setBarometerText(state, action: PayloadAction<string>) {
      state.barometerText = action.payload;
    },
    setSidebarMenuState(state, action: PayloadAction<boolean>) {
      state.sidebarMenuState = action.payload;
    },
    setOverlayImageState(state, action: PayloadAction<boolean>) {
      state.overlayImageState = action.payload;
    },
  },
});

// Export actions and reducer
export const {
  setUserID,
  setNames,
  setNameColors,
  setNameLastHeartbeats,
  setStateValues,
  setCatchValues,
  setModeColors,
  setCalibrateColors,
  setCalibrateTexts,
  setVisionColors,
  setCatchIcons,
  setShootIcons,
  setGoalColor,
  setEnemyColor,
  setBarometerColor,
  setBarometerText,
  setSidebarMenuState,
  setOverlayImageState,
} = appSlice.actions;

export default appSlice.reducer;
