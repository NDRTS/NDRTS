import React from 'react';
import { Box, Button, Group } from '@mantine/core';
import { createStyles } from '@mantine/styles';
import { IconPlayerPlay, IconPlayerStop, IconArrowUp, IconArrowDown } from '@tabler/icons-react';

const useStyles = createStyles((theme) => ({
    bottomBarContainer: {
        width: '100%',
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        backgroundColor: theme.colors.gray[8],
        color: theme.white,
    },
}));

const handleStartCar = async () => {
    try {
        const response = await fetch('/api/start-car', { method: 'POST' });
        const result = await response.json();
        if (result.success) {
            console.log("🚗 Car started:", result.message);
        } else {
            console.error("❌ Failed to start car:", result.message);
        }
    } catch (err) {
        console.error("⚠️ Network error:", err);
    }
};


function BottomBar({ data, sendToggleLaneDetection, sendAdjustSpeed }) {
    const { classes } = useStyles();

    const handleSpeedChange = (delta) => {
        const newSpeed = data.speed + delta;
        sendAdjustSpeed(newSpeed);
    };

    return (
        <Box className={classes.bottomBarContainer}>
            <Group spacing="lg">
                <Button
                    size="md"
                    radius="xl"
                    variant="light"
                    color="yellow"
                    onClick={handleStartCar}
                >
                    Start Car
                </Button>

                <Button
                    size="md"
                    radius="xl"
                    variant="light"
                    leftIcon={data.laneDetectionOn ? <IconPlayerStop size={18} /> : <IconPlayerPlay size={18} />}
                    color={data.laneDetectionOn ? 'red' : 'green'}
                    onClick={sendToggleLaneDetection}
                >
                    {data.laneDetectionOn ? 'Stop Lane Detection' : 'Start Lane Detection'}
                </Button>

                <Group spacing="sm">
                    <Button
                        size="md"
                        radius="xl"
                        variant="light"
                        color="blue"
                        leftIcon={<IconArrowUp size={18} />}
                        onClick={() => handleSpeedChange(50)}
                    >
                        Increase Speed
                    </Button>

                    <Button
                        size="md"
                        radius="xl"
                        variant="light"
                        color="blue"
                        leftIcon={<IconArrowDown size={18} />}
                        onClick={() => handleSpeedChange(-50)}
                    >
                        Decrease Speed
                    </Button>
                </Group>
            </Group>
        </Box>
    );
}

export default BottomBar;