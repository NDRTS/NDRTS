import React from 'react';
import { Box } from '@mantine/core';
import TrafficSign from './TrafficSign';
import { createStyles } from '@mantine/styles';

const useStyles = createStyles((theme) => ({
    topBarContainer: {
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
    },
}));

function TopBar({ detected_class }) {
    const { classes } = useStyles();

    return (
        <Box className={classes.topBarContainer}>
            {/* Stop Sign: Initially Hidden */}
            <TrafficSign label="Stop" isVisible={detected_class === "Stopsign,"} />
            <TrafficSign label="Semafor" isVisible={detected_class === "s,"} />
            <TrafficSign label="Parcare" isVisible={detected_class === "Parkingsign,"} />
        </Box>
    );
}

export default TopBar;
