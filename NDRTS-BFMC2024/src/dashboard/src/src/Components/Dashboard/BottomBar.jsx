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
                        onClick={() => handleSpeedChange(10)}
                    >
                        Increase Speed
                    </Button>

                    <Button
                        size="md"
                        radius="xl"
                        variant="light"
                        color="blue"
                        leftIcon={<IconArrowDown size={18} />}
                        onClick={() => handleSpeedChange(-10)}
                    >
                        Decrease Speed
                    </Button>
                </Group>
            </Group>
        </Box>
    );
}

export default BottomBar;