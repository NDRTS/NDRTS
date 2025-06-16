import React from 'react';
import { Box, Text, Image } from '@mantine/core';
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
        width: 'auto',
        height: '100%',
        backgroundColor: theme.colors.gray[9],
        border: `2px solid ${theme.colors.gray[7]}`,
        borderRadius: theme.radius.md,
        position: 'relative',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
    },
    image: {
        width: '100%',
        height: '100%',
        objectFit: 'contain', // Keep aspect ratio
        borderRadius: theme.radius.md,
    },
}));

function VideoCenter({ leftDistance, rightDistance, videoFeed }) {
    const { classes } = useStyles();
    return (
        <>
            {/* <Text className={classes.distanceTextLeft}>{leftDistance}</Text>
            <Text className={classes.distanceTextRight}>{rightDistance}</Text> */}

            <Box className={classes.videoFeed}>
                {videoFeed ? (
                    <Image src={videoFeed} className={classes.image} alt="Lane Detection" />
                ) : (
                    <Text align="center">Waiting for Video...</Text>
                )}
            </Box>
        </>
    );
}

export default VideoCenter;
