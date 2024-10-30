// Store.ts

import { configureStore } from '@reduxjs/toolkit';
import appReducer from './States';

const store = configureStore({
  reducer: {
    app: appReducer,
  },
});

// Export the store and RootState type
export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;

export default store;
