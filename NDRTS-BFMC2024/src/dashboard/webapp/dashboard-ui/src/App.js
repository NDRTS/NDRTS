import React from 'react';
import MantineEllipticLayout from './Components/Dashboard/MantineEllipticLayout';
import { MantineProvider } from '@mantine/core';


function App() {
  return (
    <MantineProvider theme={{ colorScheme: 'dark' }}>
      <div style={{ width: '100vw', height: '100vh' }}>
        <MantineEllipticLayout />
      </div >
    </MantineProvider>

  );
}

export default App;
