import React, { useEffect, useState } from 'react';
import { StyleSheet, Text, View, Image, TouchableOpacity } from 'react-native';
import { NavigationContainer } from '@react-navigation/native';
import { createBottomTabNavigator } from '@react-navigation/bottom-tabs';
import * as Notifications from 'expo-notifications';

const Tab = createBottomTabNavigator();

Notifications.setNotificationHandler({
  handleNotification: async () => ({
    shouldShowAlert: true,
    shouldPlaySound: true,
    shouldSetBadge: false,
  }),
});

function ImageView() {
  const [imageUrl, setImageUrl] = useState('');

  useEffect(() => {
    const fetchImage = async () => {
      try {
        const response = await fetch('http://46.101.186.178:5001/latest_image_url');
        const responseText = await response.text();
        console.log('Response text:', responseText);
        const data = JSON.parse(responseText);
        const imageUri = data.image_url;

        setImageUrl(imageUri);
        await Notifications.scheduleNotificationAsync({
          content: { title: 'New image received' },
          trigger: { seconds: 10, repeats: true },
        });
      } catch (error) {
        console.error('Error fetching image:', error);
      }
    };

    fetchImage();
    const interval = setInterval(fetchImage, 10000);

    return () => clearInterval(interval);
  }, []);

  return (
    <View style={styles.centered}>
      {imageUrl ? (
        <Image source={{ uri: imageUrl }} style={styles.image} />
      ) : (
        <Text>Loading image...</Text>
      )}
    </View>
  );
}

function RobotControl() {
  const sendRequest = async (url) => {
    try {
      await fetch(url);
    } catch (error) {
      console.error('Error sending request:', error);
    }
  };

  return (
    <View style={styles.centered}>
      <TouchableOpacity
        style={styles.button}
        onPress={() => sendRequest('http://192.168.4.1:3000/mode/auto')}>
        <Text>Auto Mode</Text>
      </TouchableOpacity>
      <TouchableOpacity
        style={styles.button}
        onPress={() => sendRequest('http://192.168.4.1:3000/mode/man')}>
        <Text>Manual Mode</Text>
      </TouchableOpacity>
      <TouchableOpacity
        style={styles.button}
        onPress={() => sendRequest('http://192.168.4.1:3000/mode/off')}>
        <Text>Off Mode</Text>
      </TouchableOpacity>
    </View>
  );
}

export default function App() {
  return (
    <NavigationContainer>
      <Tab.Navigator>
        <Tab.Screen name="ImageView" component={ImageView} />
        <Tab.Screen name="RobotControl" component={RobotControl} />
      </Tab.Navigator>
    </NavigationContainer>
  );
}

const styles = StyleSheet.create({
  centered: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },
  image: {
    width: '100%',
    height: '100%',
  },
  button: {
    backgroundColor: 'lightblue',
    paddingHorizontal: 20,
    paddingVertical: 10,
    borderRadius: 5,
    marginBottom: 10,
  },
});

