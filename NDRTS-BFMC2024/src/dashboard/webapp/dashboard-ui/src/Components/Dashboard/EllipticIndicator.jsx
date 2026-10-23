import React, { useState, useEffect } from 'react';
import { Box, Text } from '@mantine/core';
import { createStyles } from '@mantine/styles';

// Function to determine color based on percentage
const getBackgroundColor = (value) => {
    if (value <= 30) return '#00b894'; // Green for low usage
    if (value <= 70) return '#f1c40f'; // Yellow for medium usage
    return '#d63031'; // Red for high usage
};

const useStyles = createStyles((theme, { customStyles, fillHeight }) => ({
    container: {
        width: customStyles?.width || '100%',
        height: '100%',
        position: 'relative',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'flex-end',
        borderRadius: customStyles?.borderRadius || '14%',
        overflow: 'hidden',
        border: `2px solid ${customStyles?.backgroundColor || '#00b894'}`,
    },
    fill: {
        width: '100%',
        height: `${fillHeight}%`,
        backgroundColor: customStyles?.backgroundColor || '#00b894',
        transition: 'height 0.5s ease-in-out, background-color 0.5s ease-in-out',
        position: 'absolute',
        bottom: 0,
        left: 0,
    },
    text: {
        position: 'absolute',
        top: '50%',
        transform: 'translateY(-50%)',
        textAlign: 'center',
        width: '100%',
        color: theme.white,
        fontWeight: 'bold',
    },
    label: {
        position: 'absolute',
        bottom: -5,
        textAlign: 'center',
        width: '100%',
    },
}));


function EllipticIndicator({ label, value, customStyles }) {
    const numericValue = parseFloat(value);
    const [fillHeight, setFillHeight] = useState(numericValue);

    useEffect(() => {
        setFillHeight(numericValue);
    }, [numericValue]);

    const { classes } = useStyles({ customStyles, fillHeight });

    return (
        <Box className={classes.container}>
            <Box className={classes.fill} />
            <Text className={classes.text}>{value}</Text>
            <Text className={classes.label}>{label}</Text>
        </Box>
    );
}


export default EllipticIndicator;
