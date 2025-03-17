// TopBar.jsx
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

function TopBar() {
    const { classes } = useStyles();

    return (
        <Box className={classes.topBarContainer}>
            {/* Example traffic signs */}
            <TrafficSign label="Stop" />
            <TrafficSign label="Limit" />
            <TrafficSign label="Yield" />
        </Box>
    );
}

export default TopBar;
