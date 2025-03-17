// TrafficSign.jsx
import React from 'react';
import { Box, Text } from '@mantine/core';
import { createStyles } from '@mantine/styles';

const useStyles = createStyles((theme) => ({
    sign: {
        width: 60,
        height: 60,
        backgroundColor: theme.colors.gray[7],
        borderRadius: '50%',
        margin: '0 8px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
    },
}));

function TrafficSign({ label }) {
    const { classes } = useStyles();
    return (
        <Box className={classes.sign}>
            <Text size="xs" align="center">
                {label}
            </Text>
        </Box>
    );
}

export default TrafficSign;
