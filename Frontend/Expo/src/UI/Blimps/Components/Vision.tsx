// Vision.tsx

// React and React Native
import { useState, useEffect,useRef,useCallback } from 'react';
import { StyleSheet } from 'react-native';
import ReactHlsPlayer from 'react-hls-player';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../../Constants/Constants';

interface CameraStreamProps {
  name:string // blimp name
}


// Map camera names to their HLS stream URLs
const cameraUrlMap: Record<string, string> = {
  BurnCream : "http://192.168.0.200:8888/cam1/video1_stream.m3u8",
  SillyAh: "http://192.168.0.200:8888/cam2/video1_stream.m3u8",
  Turbo: "http://192.168.0.200:8888/cam3/video1_stream.m3u8",
  GameChamber: "http://192.168.0.200:8888/cam4/video1_stream.m3u8",
  GravyLongWay: "http://192.168.0.200:8888/cam5/video1_stream.m3u8",
  SuperBeef: "http://192.168.0.200:8888/cam6/video1_stream.m3u8",
};

const CameraStream :React.FC<CameraStreamProps> = ({ name}) =>  {
  const playerRef = useRef(null);
  const streamUrl = cameraUrlMap[name];
  if (!streamUrl) {
    return <div>Camera stream for "{name}" not found.</div>;
  }
  return (
    <div>
      <div>Camera Stream for {name}</div>
      <ReactHlsPlayer
        playerRef={playerRef}
        src= {streamUrl} //"http://192.168.0.200:8888/cam4/video1_stream.m3u8"  // Ensure your URL is a valid m3u8 stream
        autoPlay={true}
        controls={true}
        width="100%"
        height="auto"
        hlsConfig={{}}
      />
    </div>
  );
};

export const useVision = () => {
  
    // Vision Button Colors
    const [visionColors, setVisionColors] = useState<{ [key: string]: string }>({});
    // Show vision stream state
    const [showCameraStream, setShowCameraStream] = useState<{ [key: string]: boolean }>({});
  
    // Update Vision Button Color
    useEffect(() => {

      // Event handler for 'update_button_color'
      const handleUpdateButtonColor = (val: { [key: string]: string }) => {

        const receivedName: string = val['name'];
        const receivedButtonKey: string = val['key'];
        const receivedButtonColor: string = val['color'];

        let newColor = '#E11C1C'; // Default color 
        if (receivedButtonColor === 'red') {
          newColor = '#E11C1C';
        } else if (receivedButtonColor === 'green') {
          newColor = 'green';
        }

        if (receivedButtonKey === 'vision') {
          // Update visionColors with the new color for the specific blimp
          setVisionColors(prevVisionColors => ({
              ...prevVisionColors,
              [receivedName]: newColor,
          }));

          // Testing
          console.log(`${receivedButtonKey} for ${receivedName} changed to ${receivedButtonColor}`);
        }

      };

      if (socket) {
        // Listen for 'update_button_color' events
        socket.on('update_button_color', handleUpdateButtonColor);

        // Cleanup to remove the listener when the component is unmounted
        return () => {
            socket.off('update_button_color', handleUpdateButtonColor);
        };
      }

    }, [socket]);

    // Vision Button Click
    const handleVisionClick = (name: string) => {

      const val = { name: name, key: 'vision'};
      
      if (socket) {
        socket.emit('toggle_blimp_button_color', val);
      }

      // Testing
      console.log("Vision Clicked");
      setShowCameraStream((prev) => ({
        ...prev,
        [name]: !prev[name], // Toggle only the relevant blimp's vision state
      }));

    };

    // Vision Button Style
    const visionButtonStyle = StyleSheet.create({
      button: {
        width: 110,
        height: 40,
        padding: isAndroid || isIOS ? 9 : 7, // Center Text Vertically
        borderRadius: 5,
        borderWidth: 2,
        borderColor: 'black',
        marginHorizontal: 5, // Space between buttons
        justifyContent: 'center',
        alignItems: 'center',
      },
      buttonText: {
        fontWeight: 'bold',
        color: 'white', // Text color
        fontSize: 14, // Text size
        textAlign: 'center', // Center the text
        textShadowColor: 'black', // Outline color
        textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
        textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
      },
    });
  
    return {
      visionColors,
      visionButtonStyle,
      handleVisionClick,
      showCameraStream,
      CameraStream
    };

};
