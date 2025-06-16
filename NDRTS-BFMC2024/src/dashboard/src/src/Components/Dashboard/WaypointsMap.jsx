import React, { useMemo, useRef, useState, useCallback } from 'react';
import { Box } from '@mantine/core';

function WaypointsMap({ waypoints, carPos }) {
    const pts = waypoints?.points ?? [];
    const edge = waypoints?.edges ?? [];
    console.log("carPos", carPos);
    /* ───── 1. bounds ───── */
    const { minX, maxX, minY, maxY } = useMemo(() => {
        if (!pts.length) return { minX: 0, maxX: 1, minY: 0, maxY: 1 };
        const xs = pts.map(p => p.x);
        const ys = pts.map(p => p.y);
        return {
            minX: Math.min(...xs), maxX: Math.max(...xs),
            minY: Math.min(...ys), maxY: Math.max(...ys),
        };
    }, [pts]);

    /* ───── 2. view state ───── */
    const fullW = maxX - minX || 1;
    const fullH = maxY - minY || 1;
    const [view, setView] = useState({ x: minX, y: minY, w: fullW, h: fullH });

    /* ───── 3. wheel zoom & drag pan (unchanged) ───── */
    const handleWheel = useCallback((e) => {
        if (!e.deltaY) return;
        e.preventDefault();
        const rect = e.currentTarget.getBoundingClientRect();
        const px = view.x + (e.nativeEvent.offsetX / rect.width) * view.w;
        const py = view.y + (1 - e.nativeEvent.offsetY / rect.height) * view.h;

        const z = e.deltaY < 0 ? 0.8 : 1.25;
        const nw = Math.max(fullW * 0.05, Math.min(fullW, view.w * z));
        const nh = Math.max(fullH * 0.05, Math.min(fullH, view.h * z));

        setView({
            x: px - (e.nativeEvent.offsetX / rect.width) * nw,
            y: py - (1 - e.nativeEvent.offsetY / rect.height) * nh,
            w: nw, h: nh,
        });
    }, [view, fullW, fullH]);

    const drag = useRef(null);
    const onDown = e => drag.current = {
        x: e.clientX, y: e.clientY, view,
        w: e.currentTarget.clientWidth,
        h: e.currentTarget.clientHeight
    };
    const onMove = e => {
        if (!drag.current) return;
        const dx = e.clientX - drag.current.x;
        const dy = e.clientY - drag.current.y;
        const { w, h } = drag.current.view;
        const { w: pw, h: ph } = drag.current;

        setView(v => ({
            ...v,
            x: v.x - (dx / pw) * w,
            y: v.y + (dy / ph) * h
        }));
        drag.current.x = e.clientX;
        drag.current.y = e.clientY;
    };
    const onUp = () => drag.current = null;

    /* ───── 4. render ───── */
    return (
        <Box style={{ width: '100%', height: '100%' }}>
            <svg viewBox={`${view.x} ${view.y} ${view.w} ${view.h}`}
                preserveAspectRatio="xMidYMid meet"
                style={{
                    width: '100%', height: '100%',
                    cursor: drag.current ? 'grabbing' : 'grab'
                }}
                onWheel={handleWheel}
                onMouseDown={onDown} onMouseMove={onMove}
                onMouseUp={onUp} onMouseLeave={onUp}>
                {/* edges */}
                {edge.map(([a, b], i) => (
                    <line key={i}
                        x1={pts[a].x} y1={pts[a].y}
                        x2={pts[b].x} y2={pts[b].y}
                        stroke="#0984e3" strokeWidth={0.01 * fullW} />
                ))}
                {/* points */}
                {pts.map((p, i) => (
                    <circle key={i} cx={p.x} cy={p.y}
                        r={0.004 * fullW} fill="#00b894" />
                ))}
                {carPos && (
                    <circle cx={carPos.x} cy={carPos.y}
                        r={0.01 * fullW} fill="#e74c3c" />
                )}

            </svg>
        </Box>
    );
}

export default WaypointsMap;
