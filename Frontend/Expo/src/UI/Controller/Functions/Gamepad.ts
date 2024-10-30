// Gamepad.ts
import { useGamepads } from 'react-gamepads';
import { isWeb } from '../../Constants/Constants';
import { useEffect, useState } from 'react';

export const useFirstGamepad = (): Gamepad | undefined => {
  const [gamepads, setGamepads] = useState<{ [index: number]: Gamepad }>({});

  useEffect(() => {
      if (isWeb) {
          useGamepads((gamepads) => setGamepads(gamepads));
      }
  }, []);

  return Object.values(gamepads)[0];
};