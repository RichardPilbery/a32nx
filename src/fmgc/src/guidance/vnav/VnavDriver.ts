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

// Auto-added?
import { WayPoint } from '@fmgc/types/fstypes/FSTypes';

import { LateralMode, VerticalMode } from '@shared/autopilot';
import { ManagedFlightPlan } from '@fmgc/flightplanning/ManagedFlightPlan';
import { FlightPlanManager } from '@fmgc/flightplanning/FlightPlanManager';
import { SegmentType } from '@fmgc/flightplanning/FlightPlanSegment';
import { GuidanceComponent } from '../GuidanceComponent';
import { Leg, TFLeg } from '../Geometry';
import { GuidanceController } from '../GuidanceController';

export class VnavDriver implements GuidanceComponent {
    private guidanceController: GuidanceController;

    public fpm: FlightPlanManager;

    public activeWaypoint: WayPoint;

    public verticalFlightPlan: VerticalWaypoint[];

    public verticalFlightPlanSegments: VerticalSegment[];

    public firstPossibleDescentWaypointIndex: number;

    public lastClimbWaypointIndex: number;

    public firstPathSegmentIndex: number;

    public pathExists: boolean;

    public activeConstraint: Constraint;

    public firstDescentConstraintIndex: number;

    private fpChecksum: number;

    constructor(guidanceController: GuidanceController, fpm: FlightPlanManager) {
        this.guidanceController = guidanceController;
        this.fpm = fpm;

        this.activeWaypoint = undefined;
        this.verticalFlightPlan = [];
        this.verticalFlightPlanSegments = [];

        this.firstPossibleDescentWaypointIndex = 0;
        this.lastClimbWaypointIndex = 0;

        // The index of the first path segment (path segments run in reverse order to 0,
        // so the first one is the highest number).
        this.firstPathSegmentIndex = 0;

        // Whether or not a descent path has been calculated/exists
        this.pathExists = false;

        // The next constraint
        this.activeConstraint = {};

        // The flight plan index of the waypoint with the first descent constraint
        this.firstDescentConstraintIndex = undefined;

        // The checksum to compare against the flight plan.
        this.fpChecksum = -1;
    }

    /**
     * The active flight plan.
     * @type {ManagedFlightPlan}
     */
    get flightplan(): ManagedFlightPlan {
        return this.fpm.getFlightPlan(0);
    }

    get currentWaypoints(): WayPoint[] {
        return this.flightplan.waypoints.slice(this.flightplan.activeWaypointIndex);
    }

    get allWaypoints(): WayPoint[] {
        return this.flightplan.waypoints;
    }


    init(): void {
        console.log('[FMGC/Guidance] VnavDriver initialized!');
    }

    update(_deltaTime: number): void {
        // TODO: Write code for following operations:

        // Update active lateral waypoint
        // Get vertical segment from active waypoint

        // Check if conditions for VNAV to update/run are valid
        // If VNAV can be run:
        // - Update current position and distance from active waypoint
        // - If flight plan is changed:
        //      - Re-build vertical flight plan
        // - Manage constraints
        // - Calculate ToD
        //      - ToD present if we haven't descended to first
    }

    buildVerticalFlightPlan(): void {
        const waypointCount = this.allWaypoints.length;

        this.verticalFlightPlan = [];
        this.activeConstraint = {};
        this.firstDescentConstraintIndex = undefined;

        let lastClimbWaypointIndex = 0;
        let firstPossibleDescentWaypointIndex = 0;

        for (let i = 0; i < waypointCount; i++) {
            const segmentType = this.fpm.getSegmentFromWaypoint(this.allWaypoints[i]).type;
            const isClimb = !!((segmentType === SegmentType.Departure || segmentType === SegmentType.Missed));
            const constraints = this.parseConstraints(this.allWaypoints[i]);
            const VWP = new VerticalWaypoint(i, this.allWaypoints[i].ident, isClimb);

            VWP.legDistanceTo = i > 0 ? this.allWaypoints[i].cumulativeDistanceInFP - this.allWaypoints[i - 1].cumulativeDistanceInFP : 0;
            VWP.upperConstraintAltitude = constraints.upperConstraint;
            VWP.lowerConstraintAltitude = constraints.lowerConstraint;
            VWP.isAtConstraint = constraints.isAtConstraint;
            VWP.hasConstraint = constraints.hasConstraint;

            // TODO: Constraint logic goes here

            // TODO: Build vertical segments
        }
    }

    parseConstraints(waypoint: WayPoint): Constraint {
        const constraints = {
            upperConstraint: Infinity,
            lowerConstraint: 0,
            isAtConstraint: false,
            hasConstraint: false,
        };
        switch (waypoint.legAltitudeDescription) {
        case 1:
            // AT constraint
            constraints.upperConstraint = Math.floor(waypoint.legAltitude1);
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude1);
            constraints.isAtConstraint = true;
            constraints.hasConstraint = true;
            break;
        case 2:
            // AT OR ABOVE constraint
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude1);
            constraints.hasConstraint = true;
            break;
        case 3:
            // AT OR BELOW constraint
            constraints.upperConstraint = Math.floor(waypoint.legAltitude1);
            constraints.hasConstraint = true;
            break;
        case 4:
            // Altitude range constraint
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude2);
            constraints.upperConstraint = Math.floor(waypoint.legAltitude1);
            constraints.hasConstraint = true;
            break;
        case 5:
            // AT OR ABOVE constraint, using alt2
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude2);
            constraints.hasConstraint = true;
            break;
        default:
            break;
        }
        return constraints;
    }
}

class VerticalWaypoint {
    public indexInFlightPlan: number;

    public ident: string;

    public waypointFPA: number;

    public waypointFPTA: number;

    public upperConstraintAltitude: number;

    public lowerConstraintAltitude: number;

    public upperConstraintFPA: number;

    public lowerConstraintFPA: number;

    public legDistanceTo: number;

    public isClimb: boolean;

    public isAtConstraint: boolean;

    public hasConstraint: boolean;

    public segment: VerticalSegment;

    constructor(index = undefined, ident = undefined, isClimb = false) {
        // The waypoint's index in the lateral flight plan
        this.indexInFlightPlan = index;

        // The ident of the vertical waypoint.
        this.ident = ident;

        // The calculated flight path angle TO the waypoint.
        this.waypointFPA = undefined;

        // The calculated flight plan target altitude for the waypoint.
        this.waypointFPTA = undefined;

        // The highest altitude allowed at this vertical wapyoint.
        this.upperConstraintAltitude = undefined;

        // The lowest altitude allowed at this vertical wapyoint.
        this.lowerConstraintAltitude = undefined;

        // The FPA from the upper constraint altitude to the next fixed vnav target.
        this.upperConstraintFPA = undefined;

        // The FPA from the lower constraint altitude to the next fixed vnav target.
        this.lowerConstraintFPA = undefined;

        // The leg distance from the prior waypoint to this waypoint.
        this.legDistanceTo = undefined;

        // Whether this waypoint is part of the climb or not.
        this.isClimb = isClimb;

        // Whether this waypoint is an AT constraint.
        this.isAtConstraint = false;

        // Whether this waypoint has a constraint.
        this.hasConstraint = false;

        // Which vertical path segment is this waypoint part of.
        this.segment = undefined;
    }
}

class VerticalSegment {
    // TODO: Add support for IDLE and repressurization segments

    public startIndex: number;

    public targetIndex: number;

    public fpa: number;

    public distanceToNextTod: number;

    public segmentStartsLevel: boolean;

    public segmentEndsLevel: boolean;

    constructor(startIndex = undefined, targetIndex = undefined, fpa = undefined, distanceToNextTod = 0, segmentStartsLevel = false, segmentEndsLevel = false) {
        // The first waypoint index of this segment.
        this.startIndex = startIndex;

        // The last waypoint index of this segment and the vertical target of this segment.
        this.targetIndex = targetIndex;

        // The segment flight path angle (fpa).
        this.fpa = fpa;

        // The distance from the end of this segment to the next TOD;
        // 0 if it is a continuous descent or at the end of the path.
        this.distanceToNextTod = distanceToNextTod;

        // If the segment starts flat/with a TOD.
        this.segmentStartsLevel = segmentStartsLevel;

        // If the segment ends flat/level.
        this.segmentEndsLevel = segmentEndsLevel;
    }
}

class FlightModel {

}

interface Constraint {
    upperConstraint: number;
    lowerConstraint: number;
    isAtConstraint: boolean;
    hasConstraint: boolean;
}