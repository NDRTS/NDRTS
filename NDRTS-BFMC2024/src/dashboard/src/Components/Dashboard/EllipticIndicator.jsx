import React from 'react';
import { Box, Text } from '@mantine/core';
import { createStyles } from '@mantine/styles';

const useStyles = createStyles((theme, { customStyles }) => ({
    ellipse: {
        width: customStyles?.width,             // Fixed width
        height: '100%',        // Fill parent's height (e.g. 100vh)
        backgroundColor: customStyles?.backgroundColor || theme.colors.gray[6],
        position: 'relative',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        // If a clipPath is provided, use it. Otherwise, fall back to borderRadius.
        borderRadius: customStyles?.borderRadius,
    },
    ellipseLabel: {
        position: 'absolute',
        bottom: -5,
        textAlign: 'center',
        width: '100%',
    },
}));

function EllipticIndicator({ label, value, customStyles }) {
    const { classes } = useStyles({ customStyles });
    return (
        <Box className={classes.ellipse}>
            <Text>{value}</Text>
            <Text className={classes.ellipseLabel}>{label}</Text>
        </Box>
    );
}

export default EllipticIndicator;
