import React from 'react';
import { Container, Box } from '@mantine/core';
import { createStyles } from '@mantine/styles';
import TopBar from './TopBar';
import SideIndicators from './SideIndicators';
import VideoCenter from './VideoCenter';
import BottomBar from './BottomBar';
import useROSData from '../../hooks/useROSData';
import WaypointsMap from './WaypointsMap';

const useStyles = createStyles((theme) => ({
    dashboard: {
        width: '100vw',
        height: '100vh',
        backgroundColor: theme.colors.dark[8],
        color: theme.white,
        display: 'flex',
        flexDirection: 'row', // Left/Right indicators remain fixed
        overflow: 'hidden',
    },
    sideColumn: {
        width: '15%',
        height: '100vh', // Full height for left & right indicators
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'space-between',
        padding: '10px 0',
    },
    centerContainer: {
        flex: 1, // Takes remaining width between left & right indicators
        display: 'flex',
        flexDirection: 'column',
        height: '100vh', // Full viewport height
    },
    topBar: {
        height: '10%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        borderBottom: `1px solid ${theme.colors.gray[7]}`,
    },
    videoCenter: {
        flex: 1, // Takes remaining space between TopBar & BottomBar
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
    },
    centerStack: {
        flex: 1,
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',            // keep children inside
    },
    mapPane: {
        flex: '0 0 50%',
        boxSizing: 'border-box',
        overflow: 'hidden',
    },
    videoPane: {
        flex: '0 0 50%',
        boxSizing: 'border-box',
        overflow: 'hidden',
    },

    bottomBar: {
        height: '10%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        borderTop: `1px solid ${theme.colors.gray[7]}`,
    },
}));

function MantineEllipticLayout() {
    const { classes } = useStyles();
    const { data, sendToggleLaneDetection, sendAdjustSpeed } = useROSData();
    // console.log('MantineEllipticLayout', data);

    return (
        <Container fluid className={classes.dashboard}>
            {/* Left Indicators */}
            <Box className={classes.sideColumn}>
                <SideIndicators indicators={[
                    {
                        label: 'Battery',
                        value: `${data.battery}%`,
                        customStyles: {
                            backgroundColor:
                                data.battery < 20 ? '#d63031' : data.battery < 50 ? '#f1c40f' : '#00b894',
                        },
                    },
                    { label: 'Speed', value: `${data.speed} km/h` }
                ]} />

            </Box>

            {/* Main Center Layout (TopBar, VideoCenter, BottomBar) */}
            <Box className={classes.centerContainer}>
                {/* Top Bar */}
                <Box className={classes.topBar}>
                    <TopBar detected_class={data.detected_class} />
                </Box>

                {/* Video Feed */}
                <Box className={classes.centerStack}>
                    <Box className={classes.mapPane}>
                        <WaypointsMap waypoints={data.waypoints} carPos={data.car_position} />
                    </Box>
                    <Box className={classes.videoPane}>
                        <VideoCenter videoFeed={data.camera_feed} />
                    </Box>
                </Box>


                {/* Bottom Bar */}
                <Box className={classes.bottomBar}>
                    <BottomBar
                        data={data}
                        sendToggleLaneDetection={sendToggleLaneDetection}
                        sendAdjustSpeed={sendAdjustSpeed}
                    />
                </Box>
            </Box>

            {/* Right Indicators */}
            <Box className={classes.sideColumn}>
                <SideIndicators indicators={[
                    {
                        label: 'CPU',
                        value: `${data.cpu.toFixed(2)}%`,
                        customStyles: {
                            backgroundColor:
                                data.cpu > 80 ? '#d63031' : data.cpu > 50 ? '#f1c40f' : '#00b894', // 🔴 High usage = red
                        },
                    },
                    {
                        label: 'RAM',
                        value: `${data.ram.toFixed(2)}%`,
                        customStyles: {
                            backgroundColor:
                                data.ram > 80 ? '#d63031' : data.ram > 50 ? '#f1c40f' : '#00b894', // 🔴 High usage = red
                        },
                    },
                ]} />
            </Box>
        </Container>
    );
}

export default MantineEllipticLayout;
