// VideoCenter.jsx
import React from 'react';
import { Box, Text } from '@mantine/core';
import { createStyles } from '@mantine/styles';

const useStyles = createStyles((theme) => ({
    distanceTextLeft: {
        position: 'absolute',
        left: '10%',
        color: theme.colors.blue[3],
        fontSize: 18,
    },
    distanceTextRight: {
        position: 'absolute',
        right: '10%',
        color: theme.colors.blue[3],
        fontSize: 18,
    },
    videoFeed: {
        width: '60%',
        height: '60%',
        backgroundColor: theme.colors.gray[9],
        border: `2px solid ${theme.colors.gray[7]}`,
        borderRadius: theme.radius.md,
        position: 'relative',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
    },
}));

function VideoCenter({ leftDistance, rightDistance }) {
    const { classes } = useStyles();

    return (
        <>
            <Text className={classes.distanceTextLeft}>{leftDistance}</Text>
            <Text className={classes.distanceTextRight}>{rightDistance}</Text>

            <Box className={classes.videoFeed}>
                <Text align="center">Video Feed Here</Text>
            </Box>
        </>
    );
}

export default VideoCenter;
