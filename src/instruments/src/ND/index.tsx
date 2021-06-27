import React, { useState } from 'react';
import { DisplayUnit } from '@instruments/common/displayUnit';
import { FlightPlanProvider } from '@instruments/common/flightplan';
import { useSimVar } from '@instruments/common/simVars';
import { Knots } from '@typings/types';
import { render } from '../Common';
import { ArcMode } from './ArcMode';
import { WindIndicator } from './WindIndicator';
import { SpeedIndicator } from './SpeedIndicator';
import { RadioNavInfo } from './RadioNavInfo';
import { Chrono } from './Chrono';
import { NavigationDisplayMessages } from './utils/NavigationDisplayMessages';

import './styles.scss';

export type RangeSetting = 10 | 20 | 40 | 80 | 160 | 320;

export const rangeSettings: RangeSetting[] = [10, 20, 40, 80, 160, 320];

export enum Mode {
    ROSE_ILS,
    ROSE_VOR,
    ROSE_NAV,
    ARC,
    PLAN,
}

const NavigationDisplay: React.FC = () => {
    const [displayIndex] = useState(() => {
        const url = document.getElementsByTagName('a32nx-nd')[0].getAttribute('url');

        return parseInt(url?.substring(url.length - 1) ?? '1', 10);
    });

    const side = displayIndex === 1 ? 'L' : 'R';

    const [rangeIndex] = useSimVar(side === 'L' ? 'L:A32NX_EFIS_L_ND_RANGE' : 'L:A32NX_EFIS_R_ND_RANGE', 'number', 100);
    const [modeIndex] = useSimVar(side === 'L' ? 'L:A32NX_EFIS_L_ND_MODE' : 'L:A32NX_EFIS_R_ND_MODE', 'number', 100);

    const [adirsState] = useSimVar('L:A320_Neo_ADIRS_STATE', 'Enum', 500);
    const [tas] = useSimVar('AIRSPEED TRUE', 'knots', 500);

    return (
        <DisplayUnit
            electricitySimvar={displayIndex === 1 ? 'L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED' : 'L:A32NX_ELEC_AC_2_BUS_IS_POWERED'}
            potentiometerIndex={displayIndex === 1 ? 89 : 91}
        >
            <FlightPlanProvider>
                <svg className="pfd-svg" version="1.1" viewBox="0 0 768 768">
                    <SpeedIndicator adirsState={adirsState} tas={tas} />
                    <WindIndicator adirsState={adirsState} tas={tas} />

                    <ArcMode side={side} />

                    <Chrono side={side} />

                    <NavigationDisplayMessages rangeSetting={rangeSettings[rangeIndex]} mode={modeIndex} />

                    <RadioNavInfo index={1} side={side} />
                    <RadioNavInfo index={2} side={side} />
                </svg>
            </FlightPlanProvider>
        </DisplayUnit>
    );
};

export type AdirsTasDrivenIndicatorProps = {
    adirsState: number,
    tas: Knots,
}

render(<NavigationDisplay />);
