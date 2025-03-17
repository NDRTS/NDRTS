// MantineEllipticLayout.jsx
import React from 'react';
import { Container, Box } from '@mantine/core';
import { createStyles } from '@mantine/styles';

// Import our subcomponents
import TopBar from './TopBar';
import SideIndicators from './SideIndicators';
import VideoCenter from './VideoCenter';

const useStyles = createStyles((theme) => ({
    dashboard: {
        width: '100vw',
        height: '100vh',
        backgroundColor: theme.colors.dark[8],
        color: theme.white,
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',
    },
    topBar: {
        height: '10%',
        borderBottom: `1px solid ${theme.colors.gray[7]}`,
    },
    mainContent: {
        flex: 1,
        display: 'flex',
        flexDirection: 'row',
    },
    leftSide: {
        width: '10%',
        borderRight: `1px solid ${theme.colors.gray[7]}`,
        display: 'flex',
        justifyContent: 'center',
    },
    rightSide: {
        width: '10%',
        borderLeft: `1px solid ${theme.colors.gray[7]}`,
        display: 'flex',
        justifyContent: 'center',
    },
    centerArea: {
        flex: 1,
        position: 'relative',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
    },
}));

function MantineEllipticLayout() {
    const { classes } = useStyles();

    return (
        <Container fluid className={classes.dashboard}>
            {/* Top bar for traffic signs */}
            <Box className={classes.topBar}>
                <TopBar />
            </Box>

            {/* Main content area */}
            <Box className={classes.mainContent}>
                {/* Left side: Battery and Speed indicators */}
                <Box className={classes.leftSide}>
                    <SideIndicators
                        indicators={[
                            {
                                label: 'Battery',
                                value: '80%',
                                customStyles: {
                                    borderRadius: '14% 14% 0% 0%', // left-side style
                                    backgroundColor: '#00b894',    // or use theme colors via your custom mapping
                                    width: '100%', // Fill the parent width
                                },
                            },
                            {
                                label: 'Speed',
                                value: '30 km/h',
                                customStyles: {
                                    borderRadius: '0% 0% 14% 14%', // same as Battery if desired
                                    backgroundColor: '#00b894',
                                    width: '100%', // Fill the parent width
                                },
                            },
                        ]}
                    />
                </Box>

                {/* Center area: Video feed + distance measurements */}
                <Box className={classes.centerArea}>
                    <VideoCenter leftDistance="40 cm" rightDistance="20 cm" />
                </Box>

                {/* Right side: CPU and RAM indicators */}
                <Box className={classes.rightSide}>
                    <SideIndicators
                        indicators={[
                            {
                                label: 'CPU',
                                value: '50%',
                                customStyles: {
                                    borderRadius: '14% 14% 0% 0%', // right-side style
                                    backgroundColor: '#d63031',
                                    width: '100%', // Fill the parent width
                                },
                            },
                            {
                                label: 'RAM',
                                value: '2 GB',
                                customStyles: {
                                    borderRadius: '0% 0% 14% 14%', // right-side style
                                    backgroundColor: '#d63031',
                                    width: '100%', // Fill the parent width
                                },
                            },
                        ]}
                    />
                </Box>
            </Box>
        </Container>
    );
}

export default MantineEllipticLayout;
