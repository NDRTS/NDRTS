import React from 'react';
import { Box } from '@mantine/core';
import { createStyles } from '@mantine/styles';
import EllipticIndicator from './EllipticIndicator';

const useStyles = createStyles(() => ({
    sideContainer: {
        width: '100%',
        display: 'flex',
        flexDirection: 'column', // Stack indicators vertically
        alignItems: 'center',
        justifyContent: 'center',
        height: '100%', // Fill the parent height
        // gap: 10, // Add some space between indicators
    },
}));

function SideIndicators({ indicators = [] }) {
    const { classes } = useStyles();

    return (
        <Box className={classes.sideContainer}>
            {indicators.map((item, idx) => (
                <EllipticIndicator
                    key={idx}
                    label={item.label}
                    value={item.value}
                    customStyles={item.customStyles} // Pass individual styles
                />
            ))}
        </Box>
    );
}

export default SideIndicators;
