/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Working Title, FlyByWire Simulations
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import { LateralMode, VerticalMode } from '@shared/autopilot';
import { GuidanceComponent } from '../GuidanceComponent';
import { Leg, TFLeg } from '../Geometry';
import { GuidanceController } from '../GuidanceController';

export class VnavDriver implements GuidanceComponent {
    private guidanceController: GuidanceController;

    constructor(guidanceController: GuidanceController) {
        this.guidanceController = guidanceController;
    }

    init(): void {
        console.log('[FMGC/Guidance] VnavDriver initialized!');
    }

    update(_deltaTime: number): void {
        // TODO: stuff
    }
}

class VerticalWaypoint {
    constructor(index = undefined, ident = undefined, isClimb = false) {
        /** 
         * The waypoint's index in the lateral flight plan. 
         * @type {number}
         */
        this.indexInFlightPlan = index;

        /**
         * The ident of the vertical waypoint.
         * @type {string}
         */
        this.ident = ident;

        /**
         * The calculated flight path angle TO the waypoint.
         * @type {number}
         */
        this.waypointFPA = undefined;

        /**
         * The calculated flight plan target altitude for the waypoint.
         * @type {number}
         */
        this.waypointFPTA = undefined;

        /**
         * The highest altitude allowed at this vertical wapyoint.
         * @type {number}
         */
        this.upperConstraintAltitude = undefined;

        /**
         * The lowest altitude allowed at this vertical wapyoint.
         * @type {number}
         */
        this.lowerConstraintAltitude = undefined;

        /**
         * The FPA from the upper constraint altitude to the next fixed vnav target.
         * @type {number}
         */
        this.upperConstraintFPA = undefined;

        /**
         * The FPA from the lower constraint altitude to the next fixed vnav target.
         * @type {number}
         */
        this.lowerConstraintFPA = undefined;

        /**
         * The leg distance from the prior waypoint to this waypoint.
         * @type {number}
         */
        this.legDistanceTo = undefined;

        /**
         * Whether this waypoint is part of the climb or not.
         * @type {boolean}
         */
        this.isClimb = isClimb;

        /**
         * Whether this waypoint is an AT constraint.
         * @type {boolean}
         */
        this.isAtConstraint = false;

        /**
         * Whether this waypoint has a constraint.
         * @type {boolean}
         */
        this.hasConstraint = false;

        /**
         * Which vertical path segment is this waypoint part of.
         * @type {number}
         */
        this.segment = undefined;
    }
}

class VerticalSegment {
    constructor(startIndex = undefined, targetIndex = undefined, fpa = undefined, distanceToNextTod = 0, segmentStartsLevel = false, segmentEndsLevel = false) {
        /**
         * The first waypoint index of this segment.
         * @type {number}
         */
        this.startIndex = startIndex;

        /**
         * The last waypoint index of this segment and the vertical target of this segment.
         * @type {number}
         */
        this.targetIndex = targetIndex;

        /**
        * The segment flight path angle (fpa).
        * @type {number}
        */
        this.fpa = fpa;

        /**
         * The distance from the end of this segment to the next TOD;
         * 0 if it is a continuous descent or at the end of the path.
         * @type {number}
         */
        this.distanceToNextTod = distanceToNextTod;

        /**
         * If the segment starts flat/with a TOD.
         * @type {boolean}
         */
        this.segmentStartsLevel = segmentStartsLevel;

        /**
         * If the segment ends flat/level.
         * @type {boolean}
         */
        this.segmentEndsLevel = segmentEndsLevel;
    }
}