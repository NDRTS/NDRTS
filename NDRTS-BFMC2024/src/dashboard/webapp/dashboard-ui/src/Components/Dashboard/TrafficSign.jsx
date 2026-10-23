import React from 'react';
import { Box, Text } from '@mantine/core';
import { createStyles } from '@mantine/styles';

const useStyles = createStyles((theme, { isVisible }) => ({
    sign: {
        width: 60,
        height: 60,
        backgroundColor: theme.colors.gray[7],
        borderRadius: '50%',
        margin: '0 8px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        opacity: isVisible ? 1 : 0, // Hide when not detected
        transition: 'opacity 0.5s ease-in-out', // Smooth fade-in effect
    },
}));

function TrafficSign({ label, isVisible }) {
    const { classes } = useStyles({ isVisible });

    return (
        <Box className={classes.sign}>
            <Text size="xs" align="center">{label}</Text>
        </Box>
    );
}

export default TrafficSign;
